"""
LORA_RTK_RX_id_200_v2_logs.py  (ADDRESS=200)  --  DIAGNOSTIC / EMI logging build
ITERATION: RTK_LORA_ITERATION / v2
---------------------
Identical to LORA_RTK_RX_id_200_v2.py but with full RSSI/SNR logging enabled
for in-flight EMI diagnosis.

Additional features over the production script
-----------------------------------------------
- Every received packet is printed with its RSSI and SNR values.
- WARNING  banner when RSSI enters the degraded zone (-90 to -105 dBm).
- CRITICAL banner when RSSI drops below -105 dBm (near-loss territory).
- WARNING  banner when SNR drops below -10 dB.

Thresholds
----------
  RSSI_WARN_THRESHOLD     = -90   dBm  (below this: WARNING)
  RSSI_CRITICAL_THRESHOLD = -105  dBm  (below this: CRITICAL)
  SNR_WARN_THRESHOLD      = -10   dB   (below this: WARNING)

COM Ports are hardcoded: LoRa=COM5, Pixhawk=COM6 (115200 baud).
Stops on Ctrl+C.
"""

import time
import serial
from datetime import datetime
from pymavlink import mavutil

# ==================== CONFIG ====================
LORA_BAUD = 115200
LORA_BAND = 915000000         # must match TX (was 865000000)
LORA_NETWORK_ID = 5
LORA_PARAMETER = (9, 9, 1, 12)  # SF9 -- must match TX (was SF7)

MAVLINK_RTCM_MAX_FRAG_LEN = 180   
MAVLINK_RTCM_MAX_FRAGMENTS = 4    

# ==================== LOGGING THRESHOLDS ====================
RSSI_WARN_THRESHOLD     = -90    # dBm: below this prints WARNING
RSSI_CRITICAL_THRESHOLD = -105   # dBm: below this prints CRITICAL
SNR_WARN_THRESHOLD      = -10    # dB:  below this prints WARNING


# ==================== LoRa (RYLR998) driver ====================
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
                        ("BAND",      "AT+BAND=915000000"),   # fixed: was 865
                        ("PARAMETER", "AT+PARAMETER=9,9,1,12"),  # fixed: was SF7
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

    def __init__(self, port, baud=LORA_BAUD, timeout=1):
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

        self._read_exact(1, deadline)  # consume the comma
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


# ==================== TX wire-format decoder ====================
class RtcmReassembler:
    """
    Parses v2 wire format: "<TYPE>:<SEQ><hex>" / "<TYPE>_1:<SEQ><hex>" / "<TYPE>_2:<SEQ><hex>"
    <SEQ> is a 2-digit decimal (00-31) stamped by the TX for the current epoch.
    Returns (msg_type, raw_bytes, seq_id) so the caller can pass the TX-stamped
    seq_id directly to send_rtcm_to_fc(), keeping all 4 FCs in sync.
    """
    def __init__(self):
        self._pending = {}   # msg_type -> (hex_str, seq_id) awaiting part 2

    def feed(self, ascii_payload: str):
        """
        Returns (msg_type, raw_bytes, seq_id) for a complete message,
        or (None, None, None) if malformed or still waiting for part 2.
        """
        if ":" not in ascii_payload:
            print(f"  [rx] WARNING: malformed payload (no ':'): {ascii_payload!r}")
            return None, None, None

        header, raw_data = ascii_payload.split(":", 1)

        # Strip the 2-digit TX-stamped seq prefix before hex data
        if len(raw_data) < 2:
            print(f"  [rx] WARNING: payload too short for SEQ prefix: {ascii_payload!r}")
            return None, None, None
        try:
            seq_id  = int(raw_data[:2])   # "03" -> 3
        except ValueError:
            print(f"  [rx] WARNING: non-numeric SEQ prefix in: {ascii_payload!r}")
            return None, None, None
        hexdata = raw_data[2:]             # actual hex starts after seq prefix

        if "_" in header:
            type_str, part_str = header.split("_", 1)
            try:
                msg_type = int(type_str)
                part_num = int(part_str)
            except ValueError:
                print(f"  [rx] WARNING: bad split header: {header!r}")
                return None, None, None

            if part_num == 1:
                self._pending[msg_type] = (hexdata, seq_id)   # store hex + seq
                return None, None, None
            elif part_num == 2:
                entry = self._pending.pop(msg_type, None)
                if entry is None:
                    print(f"  [rx] WARNING: got part 2 of {msg_type} with no part 1 - dropping")
                    return None, None, None
                first_hex, stored_seq = entry
                full_hex = first_hex + hexdata
                try:
                    raw = bytes.fromhex(full_hex)
                except ValueError:
                    print(f"  [rx] WARNING: reassembled {msg_type} has invalid hex - dropping")
                    return None, None, None
                return msg_type, raw, stored_seq   # use seq from part 1
            else:
                print(f"  [rx] WARNING: unexpected part number in {header!r}")
                return None, None, None

        try:
            msg_type = int(header)
            raw = bytes.fromhex(hexdata)
        except ValueError:
            print(f"  [rx] WARNING: bad single-part payload: {ascii_payload!r}")
            return None, None, None
        return msg_type, raw, seq_id


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


# ==================== Diagnostic logger ====================
def log_packet(msg_type, raw_len, rssi, snr, fc_result_str):
    """
    Evaluate RSSI/SNR against thresholds, print the per-packet line with
    any warnings to the terminal.
    """
    # ── RSSI evaluation ──────────────────────────────────────────────────
    if rssi is None:
        rssi_flag = "N/A"
    elif rssi <= RSSI_CRITICAL_THRESHOLD:
        rssi_flag = "CRITICAL"
    elif rssi <= RSSI_WARN_THRESHOLD:
        rssi_flag = "WARN"
    else:
        rssi_flag = "OK"

    # ── SNR evaluation ────────────────────────────────────────────────────
    if snr is None:
        snr_flag = "N/A"
    elif snr < SNR_WARN_THRESHOLD:
        snr_flag = "WARN"
    else:
        snr_flag = "OK"

    # ── Console line ────────────────────────────────────────────────────
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]   # HH:MM:SS.mmm
    rssi_str = f"{rssi} dBm" if rssi is not None else "N/A"
    snr_str  = f"{snr} dB"  if snr  is not None else "N/A"
    print(f"  [{ts}] msg {msg_type:4d} | {raw_len:3d}B | "
          f"RSSI={rssi_str:>10s} [{rssi_flag:8s}] | "
          f"SNR={snr_str:>8s} [{snr_flag:4s}] | FC: {fc_result_str}")

    # ── Warning banners ──────────────────────────────────────────────────
    if rssi_flag == "CRITICAL":
        print(f"  *** CRITICAL: RSSI={rssi} dBm is below {RSSI_CRITICAL_THRESHOLD} dBm "
              f"-- signal near loss threshold! Check antenna / separation ***")
    elif rssi_flag == "WARN":
        print(f"  !!! WARNING:  RSSI={rssi} dBm is in degraded zone "
              f"({RSSI_WARN_THRESHOLD} to {RSSI_CRITICAL_THRESHOLD} dBm) "
              f"-- possible EMI or range issue")

    if snr_flag == "WARN":
        print(f"  !!! WARNING:  SNR={snr} dB is below {SNR_WARN_THRESHOLD} dB "
              f"-- high noise floor, likely EMI interference")


# ==================== Main ====================
def main():
    print("Hardcoded module ADDRESS: 200  [DIAGNOSTIC LOGGING BUILD]")
    lora_port = "COM5"
    fc_conn_str = "COM6"
    fc_baud = 115200

    print(f"Hardcoded LoRa port: {lora_port}")
    print(f"Hardcoded Pixhawk port: {fc_conn_str} @ {fc_baud} baud")
    print(f"RSSI thresholds  -- WARN: ≤{RSSI_WARN_THRESHOLD} dBm | "
          f"CRITICAL: ≤{RSSI_CRITICAL_THRESHOLD} dBm")
    print(f"SNR threshold    -- WARN: < {SNR_WARN_THRESHOLD} dB\n")

    lora = RYLR998(lora_port, LORA_BAUD)

    print(f"[mavlink] Connecting to {fc_conn_str} ...")
    mav = mavutil.mavlink_connection(fc_conn_str, baud=fc_baud)
    mav.wait_heartbeat(timeout=10)
    print(f"[mavlink] Heartbeat received from system {mav.target_system}, "
          f"component {mav.target_component}.")

    reassembler = RtcmReassembler()
    # seq_id now comes from the TX-stamped value inside each packet (no local counter)

    last_rx_time = None
    TIMEOUT_SECONDS = 5.0

    print("\nListening for RTCM3 packets over LoRa. Press Ctrl+C to stop.")
    print("Dynamic Reset is ACTIVE. If a packet is received, but no subsequent")
    print("packets arrive within 5 seconds, the LoRa module will be hard reset.\n")
    
    try:
        while True:
            packet = lora.receive(timeout=1.0)
            
            if packet is None:
                # Timeout occurred during receive (1 second elapsed). Check for 5-second silence.
                if last_rx_time is not None and (time.time() - last_rx_time > TIMEOUT_SECONDS):
                    print(f"\n[!] WARNING: No data received for {TIMEOUT_SECONDS} seconds.")
                    print("[!] Triggering Hardware Reset on LoRa Module...")
                    
                    # 1. Close current port
                    lora.close()
                    time.sleep(0.5)
                    
                    # 2. Re-instantiate LoRa (triggers AT configuration blast)
                    lora = RYLR998(lora_port, LORA_BAUD)
                    
                    # 3. Wipe reassembler buffers
                    reassembler = RtcmReassembler()

                    # 4. Reset tracker so we don't infinitely reset while waiting for first packet
                    last_rx_time = None
                    print("[+] Reset Complete. Resuming listening...\n")
                continue
                
            # If we reach here, we successfully received a packet!
            last_rx_time = time.time()
            src, data, rssi, snr = packet

            try:
                ascii_payload = data.decode("ascii")
            except UnicodeDecodeError:
                print(f"From {src} | non-ASCII payload ({len(data)} bytes) - skipping | "
                      f"RSSI={rssi} SNR={snr}")
                continue

            print(f"\nFrom addr={src} | payload: {ascii_payload!r}")

            msg_type, raw, tx_seq = reassembler.feed(ascii_payload)
            if msg_type is None:
                continue

            # Use TX-stamped seq so all 4 FCs use the same seq for each epoch
            result = send_rtcm_to_fc(mav, raw, tx_seq)

            # ── Build FC result string for logging ───────────────────────────────
            if result is None:
                fc_result_str = "DROPPED-too-large"
            elif len(result) == 1:
                flags, length = result[0]
                fc_result_str = f"OK-1pkt-{length}B"
            else:
                fc_result_str = f"OK-{len(result)}frags"

            log_packet(msg_type, len(raw), rssi, snr, fc_result_str)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        lora.close()


if __name__ == "__main__":
    main()
