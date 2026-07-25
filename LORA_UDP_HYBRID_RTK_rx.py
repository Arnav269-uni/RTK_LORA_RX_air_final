"""
LORA_UDP_HYBRID_RTK_rx.py
---------------------
Hybrid RX script for LORA_HYBRIDtx.py.

Listens on BOTH:
1. UDP port 5010 (for fast BeiDou/1124 WiFi RTCM JSON packets).
2. LoRa COM5 (for 1005+1074+1084+1094+1230 AT+RCV ASCII packets).

Design notes (matched to LORA_HYBRIDtx.py wire format):
- LoRa wire format: "<msg_type>:<full_hex>"  (no embedded SEQ tag -- TX does
  not inject one, so the reassembler uses the raw hex directly).
- UDP wire format: JSON with keys "type", "msg_type", "epoch_seq", "hex", "ts".
  The epoch_seq field is used as the dedup key on the UDP side.
- LoRa and UDP carry DISJOINT message sets (1005/1074/1084/1094/1230 vs 1124),
  so cross-channel deduplication is not needed. A monotonic per-channel counter
  is used as the dedup key on the LoRa side to allow consecutive same-type
  messages through without false positives.
- Retains the LoRa Autonomous Dynamic Reset (5-second silence timeout).
"""

import time
import socket
import json
import serial
import threading
import queue
from pymavlink import mavutil

# ==================== CONFIG ====================
LORA_PORT = "COM5"
LORA_BAUD = 115200

WIFI_RTCM_PORT = 5010

FC_PORT = "COM6"
FC_BAUD = 115200

MAVLINK_RTCM_MAX_FRAG_LEN = 180   
MAVLINK_RTCM_MAX_FRAGMENTS = 4    

# ==================== MAVLink / pymavlink RTCM forwarding ====================
def send_rtcm_to_fc(mav, frame: bytes, seq_id: int):
    max_frag = MAVLINK_RTCM_MAX_FRAG_LEN

    if len(frame) > max_frag * MAVLINK_RTCM_MAX_FRAGMENTS:
        return None

    seq_bits = (seq_id & 0x1F) << 3
    sent = []

    if len(frame) <= max_frag:
        chunk = frame + b"\x00" * (max_frag - len(frame))
        mav.mav.gps_rtcm_data_send(seq_bits, len(frame), chunk)
        sent.append((seq_bits, len(frame)))
        return sent

    offset = 0
    fragment_id = 0
    while offset < len(frame) and fragment_id < MAVLINK_RTCM_MAX_FRAGMENTS:
        piece = frame[offset: offset + max_frag]
        flags = 1 | ((fragment_id & 0x03) << 1) | seq_bits
        chunk = piece + b"\x00" * (max_frag - len(piece))
        mav.mav.gps_rtcm_data_send(flags, len(piece), chunk)
        sent.append((flags, len(piece)))
        offset += max_frag
        fragment_id += 1

    if len(frame) % max_frag == 0 and fragment_id < MAVLINK_RTCM_MAX_FRAGMENTS:
        flags = 1 | ((fragment_id & 0x03) << 1) | seq_bits
        mav.mav.gps_rtcm_data_send(flags, 0, b"\x00" * max_frag)
        sent.append((flags, 0))

    return sent


# ==================== Dedup Tracker ====================
class DedupTracker:
    def __init__(self, ttl_seconds=10.0):
        self.seen = {}
        self.ttl = ttl_seconds
        self.lock = threading.Lock()
    
    def is_new_and_mark(self, msg_type, seq):
        now = time.time()
        with self.lock:
            # Cleanup old memory
            to_delete = [k for k, ts in self.seen.items() if now - ts > self.ttl]
            for k in to_delete:
                del self.seen[k]
                
            key = (msg_type, seq)
            if key in self.seen:
                return False
                
            self.seen[key] = now
            return True


# ==================== LoRa Driver & Reassembler ====================
class RYLR998:
    def _auto_config_lora(self, port):
        print(f"\n[*] Auto-configuring RX LoRa module on {port}...")
        common_bauds = [115200, 57600, 38400, 9600, 230400]
        for b in common_bauds:
            try:
                temp_ser = serial.Serial(port, b, timeout=0.5)
                temp_ser.reset_input_buffer()
                temp_ser.write(b"AT\r\n")
                time.sleep(0.3)
                reply = temp_ser.read(temp_ser.in_waiting or 128).decode("ascii", errors="ignore")
                if "+OK" in reply:
                    print(f"[+] Found LoRa module at {b} baud.")
                    
                    if b != 115200:
                        print("    -> Setting baud to 115200...")
                        temp_ser.write(b"AT+IPR=115200\r\n")
                        time.sleep(0.3)
                        temp_ser.close()
                        temp_ser = serial.Serial(port, 115200, timeout=0.5)
                        time.sleep(0.2)
                    
                    params = [
                        ("ADDRESS",   "AT+ADDRESS=200"),
                        ("NETWORKID", "AT+NETWORKID=5"),
                        ("BAND",      "AT+BAND=865000000"),
                        ("PARAMETER", "AT+PARAMETER=9,9,1,12"),
                        ("BAUD",      "AT+IPR=115200"),
                    ]
                    for label, cmd in params:
                        temp_ser.reset_input_buffer()
                        temp_ser.write((cmd + "\r\n").encode("ascii"))
                        time.sleep(0.3)
                        resp = temp_ser.read(temp_ser.in_waiting or 128).decode("ascii", errors="ignore")
                        status = f"Success! (Response: {resp.strip()!r})" if "+OK" in resp else f"FAIL ({resp.strip()!r})"
                        print(f"    {label}: {status}")
                    
                    temp_ser.close()
                    print("[+] RX LoRa configuration complete.")
                    return
                temp_ser.close()
            except serial.SerialException:
                pass
        print("[-] Could not find LoRa module or auto-config failed.")

    def __init__(self, port, baud=115200, timeout=1):
        self._auto_config_lora(port)
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def receive(self, timeout=1.0):
        deadline = time.time() + timeout
        prefix = b"+RCV="
        buf = b""

        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            buf += b
            if len(buf) > len(prefix):
                buf = buf[-len(prefix):]
            if buf == prefix:
                deadline = time.time() + 0.5
                break
        else:
            return None
        if buf != prefix:
            return None

        addr_str = self._read_until_comma(deadline)
        length_str = self._read_until_comma(deadline)
        if addr_str is None or length_str is None:
            return None
        try:
            src = int(addr_str)
            length = int(length_str)
        except ValueError:
            return None

        data = self._read_exact(length, deadline)
        if data is None:
            return None

        self._read_exact(1, deadline)
        rssi_str = self._read_until_comma(deadline)
        snr_str = self._read_until(b"\r\n", deadline)

        try:
            rssi = int(rssi_str) if rssi_str is not None else None
        except ValueError:
            rssi = None
        try:
            snr = int(snr_str) if snr_str is not None else None
        except ValueError:
            snr = None

        return src, data, rssi, snr

    def _read_exact(self, n, deadline):
        out = b""
        while len(out) < n and time.time() < deadline:
            chunk = self.ser.read(n - len(out))
            out += chunk
        return out if len(out) == n else None

    def _read_until_comma(self, deadline):
        out = b""
        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            if b == b",":
                return out.decode(errors="ignore")
            out += b
        return None

    def _read_until(self, marker, deadline):
        out = b""
        while time.time() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            out += b
            if out.endswith(marker):
                return out[: -len(marker)].decode(errors="ignore")
        return out.decode(errors="ignore") if out else None

    def close(self):
        self.ser.close()


class RtcmReassembler:
    def __init__(self):
        self._pending = {}  

    def feed(self, ascii_payload: str):
        """
        Parse a LoRa ASCII payload from LORA_HYBRIDtx.py.

        Wire format (no embedded SEQ -- TX does not inject one):
            Single:  "<msg_type>:<full_hex>"
            Part 1:  "<msg_type>_1:<full_hex_chunk>"
            Part 2:  "<msg_type>_2:<full_hex_remainder>"

        Returns (msg_type, raw_bytes) on success, (None, None) otherwise.
        The caller is responsible for assigning a seq/dedup key.
        """
        if ":" not in ascii_payload:
            return None, None

        header, hexdata = ascii_payload.split(":", 1)

        if "_" in header:
            type_str, part_str = header.split("_", 1)
            try:
                msg_type = int(type_str)
                part_num = int(part_str)
            except ValueError:
                return None, None

            if part_num == 1:
                self._pending[msg_type] = hexdata
                return None, None
            elif part_num == 2:
                if msg_type not in self._pending:
                    print(f"  [rx] WARNING: got part 2 of {msg_type} with no part 1 -- dropping")
                    return None, None
                first_hex = self._pending.pop(msg_type)
                full_hex = first_hex + hexdata
                try:
                    raw = bytes.fromhex(full_hex)
                    return msg_type, raw
                except ValueError:
                    print(f"  [rx] WARNING: reassembled {msg_type} has invalid hex -- dropping")
                    return None, None
            else:
                print(f"  [rx] WARNING: unexpected part number in {header!r}")
                return None, None
        else:
            try:
                msg_type = int(header)
                raw = bytes.fromhex(hexdata)
                return msg_type, raw
            except ValueError:
                print(f"  [rx] WARNING: bad single-part payload: {ascii_payload!r}")
                return None, None


# ==================== Threads ====================
class PacketItem:
    def __init__(self, source, msg_type, seq, raw_bytes, meta=""):
        self.source = source
        self.msg_type = msg_type
        self.seq = seq
        self.raw_bytes = raw_bytes
        self.meta = meta

def udp_listener_worker(port, out_queue, stop_event):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", port))
    s.settimeout(1.0)
    
    while not stop_event.is_set():
        try:
            data, addr = s.recvfrom(2048)
        except socket.timeout:
            continue
        except OSError:
            break
            
        try:
            payload = json.loads(data.decode("ascii"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
            
        if payload.get("type") != "RTCM":
            continue
            
        msg_type = payload.get("msg_type")
        # TX sends "epoch_seq" (LORA_HYBRIDtx.py); fall back to "seq" for
        # compatibility with any older TX variant that used that key name.
        epoch_seq = payload.get("epoch_seq") or payload.get("seq")
        hex_data = payload.get("hex")

        if not all(v is not None for v in (msg_type, epoch_seq, hex_data)):
            continue
            
        try:
            raw = bytes.fromhex(hex_data)
            out_queue.put(PacketItem("WIFI", msg_type, epoch_seq, raw, f"from {addr[0]}"))
        except ValueError:
            continue
    s.close()

def lora_listener_worker(port, baud, out_queue, stop_event):
    try:
        lora = RYLR998(port, baud)
    except Exception as e:
        print(f"[!] LoRa Initialization failed: {e}")
        return
        
    reassembler = RtcmReassembler()
    last_rx = time.time()
    TIMEOUT_SECONDS = 5.0
    # Monotonic counter used as the dedup key for LoRa packets.
    # LoRa and UDP carry disjoint message sets so there is no cross-channel
    # overlap to deduplicate -- we just need a unique key so consecutive
    # same-type LoRa messages are never mistakenly suppressed.
    lora_seq = 0

    while not stop_event.is_set():
        packet = lora.receive(timeout=1.0)

        if packet is None:
            if time.time() - last_rx > TIMEOUT_SECONDS:
                print(f"\n[!] WARNING: LoRa silence for {TIMEOUT_SECONDS}s. Hardware Resetting...")
                lora.close()
                time.sleep(0.5)
                try:
                    lora = RYLR998(port, baud)
                except Exception:
                    pass
                reassembler = RtcmReassembler()
                last_rx = time.time()
                print("[+] LoRa Reset Complete.\n")
            continue

        last_rx = time.time()
        src, data, rssi, snr = packet

        try:
            ascii_payload = data.decode("ascii")
        except UnicodeDecodeError:
            continue

        # RtcmReassembler.feed() now returns (msg_type, raw) -- no embedded SEQ.
        msg_type, raw = reassembler.feed(ascii_payload)

        if msg_type is not None:
            lora_seq = (lora_seq + 1) % 100000
            out_queue.put(PacketItem("LORA", msg_type, lora_seq, raw, f"RSSI={rssi} SNR={snr}"))

    lora.close()


# ==================== Main ====================
def main():
    print(f"Hardcoded LoRa port: {LORA_PORT}")
    print(f"Hardcoded Pixhawk port: {FC_PORT} @ {FC_BAUD} baud")

    print(f"[mavlink] Connecting to {FC_PORT} ...")
    mav = mavutil.mavlink_connection(FC_PORT, baud=FC_BAUD)
    mav.wait_heartbeat(timeout=10)
    print(f"[mavlink] Heartbeat received from system {mav.target_system}, "
          f"component {mav.target_component}.")

    stop_event = threading.Event()
    msg_queue = queue.Queue()
    tracker = DedupTracker()

    # Start Listeners
    udp_thread = threading.Thread(target=udp_listener_worker, args=(WIFI_RTCM_PORT, msg_queue, stop_event), daemon=True)
    lora_thread = threading.Thread(target=lora_listener_worker, args=(LORA_PORT, LORA_BAUD, msg_queue, stop_event), daemon=True)
    
    udp_thread.start()
    lora_thread.start()

    print("\nListening for RTCM3 packets simultaneously on LoRa (COM5) and WiFi (UDP 5010).")
    print("Deduplication is ACTIVE. Press Ctrl+C to stop.\n")
    
    fc_seq_id = 0
    try:
        while True:
            # Pop whoever receives data first
            pkt = msg_queue.get()
            
            # Check Deduplication (Have we seen this msg_type + epoch_seq combination recently?)
            if tracker.is_new_and_mark(pkt.msg_type, pkt.seq):
                # New Data! Forward to Pixhawk
                result = send_rtcm_to_fc(mav, pkt.raw_bytes, fc_seq_id)
                fc_seq_id = (fc_seq_id + 1) % 32
                
                print(f"[{pkt.source:4s}] msg {pkt.msg_type:4d} | seq {pkt.seq:02d} | {len(pkt.raw_bytes):3d}B -> SENT to FC")
                if result is None:
                    print(f"  --> DROPPED (exceeds max fragmentable size)") 
            else:
                # Duplicate Data! Safely drop it
                print(f"[{pkt.source:4s}] msg {pkt.msg_type:4d} | seq {pkt.seq:02d} | {len(pkt.raw_bytes):3d}B -> DEDUPED (silently dropped)")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        stop_event.set()

if __name__ == "__main__":
    main()
