"""
RTK_Lora_TX_v2.py  --  GNSS Base -> filter RTK corrections -> LoRa broadcast
ITERATION: RTK_LORA_ITERATION / v2

Changes from baseline (LORA_RTK_uf_tx+config.py):
──────────────────────────────────────────────────
1. SEQ-ID EMBEDDING  (primary fix for cross-drone Fix/Float inconsistency)
   The TX now stamps a 2-digit epoch sequence counter (00-31) as the FIRST
   two characters of every LoRa packet's data field:

     Old wire format:  AT+SEND=0,<len>,<TYPE>:<hex>
     New wire format:  AT+SEND=0,<len>,<TYPE>:<SEQ><hex>
                                              ^^^^
                                         2-digit decimal, zero-padded (00-31)

   Example: "1074:03abcdef..." -> type=1074, seq=03, hex="abcdef..."

   All 4 drones receive the SAME seq for every epoch because the counter
   originates at the TX (epoch_num % 32), NOT independently at each RX.
   This eliminates the seq_id desync that causes ArduPilot to mis-reassemble
   BeiDou 1124 fragments on drones that dropped a prior packet.

   Split messages carry the same SEQ on both parts:
     AT+SEND=0,<len>,1124_1:<SEQ><hex[:231]>
     AT+SEND=0,<len>,1124_2:<SEQ><hex[231:]>

   ── RX SIDE CHANGE REQUIRED ────────────────────────────────────────────
   Update RtcmReassembler.feed() on the OBC RX script:

       header, raw_data = ascii_payload.split(":", 1)
       seq_id  = int(raw_data[:2])   # extract the 2-digit stamped seq
       hexdata = raw_data[2:]         # rest is the actual hex payload

   Then pass seq_id (instead of a local counter) to send_rtcm_to_fc().
   ───────────────────────────────────────────────────────────────────────

2. BUG FIX: Corrected misleading comment
   time.sleep(0.02) comment previously said "200ms" -- corrected to "20ms".

3. MAX_HEX_CHARS reduced from 233 to 231
   The 2-char SEQ prefix occupies 2 of the 240-char AT+SEND budget.
   Single-send budget: 240 - 5 ("TYPE:") - 2 (SEQ) = 233 -> now correctly 231.

LoRa parameters (SF9/BW500kHz/CR1/Preamble12):
    AT+PARAMETER=9,9,1,12  ->  ~857ms total epoch airtime, fits 0.5Hz window
    AT+BAND=915000000       ->  915 MHz
    AT+NETWORKID=5
    AT+IPR=115200

Message filter (ALL 4 MSM4 constellations):
    1005  base reference position    25B raw   ->  57 chars  -> 1 send
    1074  GPS MSM4 observations     115B raw   -> 237 chars  -> 1 send
    1084  GLONASS MSM4 observations  69B raw   -> 145 chars  -> 1 send
    1094  Galileo MSM4 observations 100B raw   -> 207 chars  -> 1 send
    1124  BeiDou MSM4 observations  173B raw   -> 348 chars  -> 2 sends  <- largest
    1230  GLONASS Inter-Frequency Biases
    Everything else (NMEA, UBX, MSM7) is dropped.

Per-message LoRa packet format (v2):
    AT+SEND=0,<len>,<TYPE>:<SEQ><hex>
    e.g.  AT+SEND=0,57,1005:03aabb...
    Split messages:
    AT+SEND=0,242,1124_1:<SEQ><231 hex chars>
    AT+SEND=0,<n>, 1124_2:<SEQ><remaining hex>
    The RX strips the 2-char SEQ, hex-decodes the rest, reassembles, forwards.

Install deps:
    pip install pyserial pyrtcm pyubx2
"""

from serial import Serial, SerialException
from pyrtcm import RTCMReader
from pyubx2 import UBXMessage, UBXReader, POLL
import threading
import queue
import time
import logging

# Surfaces pyrtcm's internal parse errors (previously fully silent with
# quitonerror=0) so we can see WHY a message type fails to parse, instead
# of it just vanishing with no trace.
logging.basicConfig(level=logging.ERROR, format="%(name)s: %(message)s")

# ── RTCM message filter ─────────────────────────────────────────────────────
# All 4 MSM4 constellations + 1230 GLONASS Bias -- fixes high HDOP when the air unit tracks more
# constellations than the base was providing corrections for.
KEEP_TYPES     = {1005, 1074, 1084, 1094, 1124, 1230}
# 1084 removed from EPOCH_COMPLETE so we don't get "incomplete epoch" warnings if GLONASS drops out
EPOCH_COMPLETE = {1005, 1074, 1094, 1124}  # 1230 included to prevent epoch boundary bleed (validated in LORA_BASE_sim.py)

# ── LoRa config ──────────────────────────────────────────────────────────────
NETWORK_ID    = 5
BAND_HZ       = 915000000
LORA_BAUD     = 115200

# CHOOSE YOUR LORA AND GNSS RATE COMBINATION:
# Option A (Standard Range, 1Hz rate): SF7 / BW500kHz / CR1 / Preamble12
# LORA_PARAM    = "7,9,1,12"
# GNSS_RATE_HZ  = 1.0

# Option B (Longer Range, 0.5Hz rate): SF9 / BW500kHz / CR1 / Preamble12
# Option B (Longer Range, 0.5Hz rate): SF9 / BW500kHz / CR1 / Preamble12
# Since SF9 airtime is 4x longer than SF7 (~1.69s total per epoch), we MUST slow down
# the GNSS rate to 0.5Hz (one epoch every 2s) so the LoRa transmitter can keep up.
LORA_PARAM    = "9,9,1,12"
GNSS_RATE_HZ  = 0.5

# ── Timing ───────────────────────────────────────────────────────────────────
# Max single-send airtime at SF9/BW500kHz with 240-char payload ~312ms.
# Timeout = airtime + safety margin. Increased to 2.0s to ensure ACK has plenty of time.
SEND_TIMEOUT  = 2.0   # seconds

# Max hex chars in one AT+SEND payload:
# 240 limit - 5 ("TYPE:") - 2 (SEQ prefix) = 233 -> 231
MAX_HEX_CHARS = 231

# Queue size: 6 message types per epoch now (with 1230 added),
# so increase headroom to cover ~3 full epochs without dropping.
QUEUE_MAXSIZE = 24


# ── Serial helpers ────────────────────────────────────────────────────────────

def send_at(ser, cmd, wait=0.5):
    """Send an AT command with a fixed wait (used for config, not data)."""
    ser.write((cmd.strip() + "\r\n").encode("ascii"))
    time.sleep(wait)
    return ser.read(ser.in_waiting or 128).decode("ascii", errors="replace").strip()


def wait_for_ok(ser, timeout=SEND_TIMEOUT):
    """
    Block until the LoRa module replies +OK (or +ERR), or timeout expires.
    The module sends +OK only AFTER the packet is fully transmitted over the air.
    DO NOT call reset_input_buffer() between write() and this function --
    that would discard the +OK we are waiting for.

    This is fine to block on now: it runs on the LoRa-sending side only,
    never on the same thread that's reading the GNSS.
    """
    deadline = time.time() + timeout
    buf = ""
    while time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting).decode("ascii", errors="replace")
            if "+OK" in buf or "+ERR" in buf:
                break
        time.sleep(0.005)
    return buf.strip()


# ── Hardware Auto-Configuration ─────────────────────────────────────────────

def auto_configure_base(gnss_port):
    """
    Sweeps common baud rates to find the receiver, forces it to 115200, 
    disables NMEA/MSM7 to save bandwidth, and activates MSM4 + GLONASS Bias (1230) messages.
    """
    print(f"\n[*] Auto-configuring Base Station on {gnss_port}...")
    print("[*] Forcing 115200 baud, enabling MSM4 + GLONASS Bias (1230), and killing NMEA noise...")
    
    common_bauds = [115200, 38400, 57600, 9600, 230400, 460800]
    
    cfg_data = [
        # 0. Explicitly enable GLONASS constellation tracking in hardware
        ("CFG_SIGNAL_GLO_ENA", 1),
        ("CFG_SIGNAL_GLO_L1_ENA", 1),
        ("CFG_SIGNAL_GLO_L2_ENA", 1),

        # 1. Force Baud to 115200 on UART1
        ("CFG_UART1_BAUDRATE", 115200),
        
        # 2. Protocol out: RTCM3 and UBX on, NMEA off (kills bandwidth bloat)
        ("CFG_UART1OUTPROT_RTCM3X", 1),
        ("CFG_UART1OUTPROT_UBX", 1),
        ("CFG_UART1OUTPROT_NMEA", 0),
        ("CFG_USBOUTPROT_RTCM3X", 1),
        ("CFG_USBOUTPROT_UBX", 1),
        ("CFG_USBOUTPROT_NMEA", 0),
        
        # 3. ENABLE MSM4, 1005, AND 1230 GLONASS BIAS (UART1 & USB)
        ("CFG_MSGOUT_RTCM_3X_TYPE1005_UART1", 1), # Base Position
        ("CFG_MSGOUT_RTCM_3X_TYPE1074_UART1", 1), # GPS MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1084_UART1", 1), # GLONASS MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1094_UART1", 1), # Galileo MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1124_UART1", 1), # BeiDou MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1230_UART1", 1), # GLONASS Code-Phase Biases
        
        ("CFG_MSGOUT_RTCM_3X_TYPE1005_USB", 1),
        ("CFG_MSGOUT_RTCM_3X_TYPE1074_USB", 1),   # GPS MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1084_USB", 1),   # GLONASS MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1094_USB", 1),   # Galileo MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1124_USB", 1),   # BeiDou MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1230_USB", 1),   # GLONASS Code-Phase Biases

        # 4. DISABLE MSM7 (UART1 & USB) to prevent 720-byte packet walls
        ("CFG_MSGOUT_RTCM_3X_TYPE1077_UART1", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1087_UART1", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1097_UART1", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1127_UART1", 0),
        
        ("CFG_MSGOUT_RTCM_3X_TYPE1077_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1087_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1097_USB", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1127_USB", 0),
    ]

    for baud in common_bauds:
        try:
            ser = Serial(gnss_port, baud, timeout=0.2)
            msg = UBXMessage.config_set(layers=1, transaction=0, cfgData=cfg_data)
            ser.write(msg.serialize())
            ser.flush()
            time.sleep(0.1)
            ser.close()
        except SerialException:
            pass
            
    print("[+] Base configuration complete. Module locked to 115200 baud.")

def auto_configure_lora(port):
    """
    Auto-detect and fully configure the TX LoRa module.
    Hardcoded from compatibility test results:
        ADDRESS=100, NETWORKID=5, BAND=865000000, PARAMETER=7,9,1,12, BAUD=115200
    """
    print(f"\n[*] Auto-configuring TX LoRa module on {port}...")
    common_bauds = [115200, 57600, 38400, 9600, 230400, 460800]
    
    for baud in common_bauds:
        try:
            ser = Serial(port, baud, timeout=0.5)
            ser.reset_input_buffer()
            ser.write(b"AT\r\n")
            time.sleep(0.3)
            reply = ser.read(ser.in_waiting or 128).decode("ascii", errors="ignore")
            
            if "+OK" in reply:
                print(f"[+] Found LoRa module at {baud} baud.")
                
                # Force baud to 115200 if needed
                if baud != 115200:
                    print("    -> Setting baud to 115200...")
                    ser.write(b"AT+IPR=115200\r\n")
                    time.sleep(0.3)
                    ser.close()
                    ser = Serial(port, 115200, timeout=0.5)
                    time.sleep(0.2)
                
                # Set all parameters from compatibility test
                params = [
                    ("ADDRESS",   "AT+ADDRESS=100"),
                    ("NETWORKID", "AT+NETWORKID=5"),
                    ("BAND",      "AT+BAND=915000000"),
                    ("PARAMETER", "AT+PARAMETER=9,9,1,12"),
    
                    ("BAUD",      "AT+IPR=115200"),
                ]
                for label, cmd in params:
                    ser.reset_input_buffer()
                    ser.write((cmd + "\r\n").encode("ascii"))
                    time.sleep(0.3)
                    resp = ser.read(ser.in_waiting or 128).decode("ascii", errors="ignore")
                    status = f"Success! (Response: {resp.strip()!r})" if "+OK" in resp else f"FAIL ({resp.strip()!r})"
                    print(f"    {label}: {status}")
                
                ser.close()
                print("[+] TX LoRa configuration complete.")
                return True
            ser.close()
        except SerialException:
            pass
            
    print("[-] Could not find LoRa module or auto-config failed.")
    return False

# ── LoRa configuration ────────────────────────────────────────────────────────

def configure_lora(ser, my_address):
    steps = [
        ("NETWORKID", f"AT+NETWORKID={NETWORK_ID}"),
        ("BAND",      f"AT+BAND={BAND_HZ}"),
        ("PARAMETER", f"AT+PARAMETER={LORA_PARAM}"),
        ("ADDRESS",   f"AT+ADDRESS={my_address}"),
    ]
    for label, cmd in steps:
        reply = send_at(ser, cmd)
        status = "OK" if "+OK" in reply else f"FAIL ({reply!r})"
        print(f"  {label}: {status}")


# ── Survey functions ──────────────────────────────────────────────────────────

def send_cfg(port, baud, cfg):
    ser = Serial(port, baud, timeout=2)
    msg = UBXMessage.config_set(layers=1, transaction=0, cfgData=cfg)
    ser.write(msg.serialize())
    ser.close()


def start_survey(gnss_port, gnss_baud, duration):
    acc_raw = int(round(50000 / 0.1))   # 5m in 0.1mm units -- effectively ignored
    cfg = [
        ("CFG_TMODE_MODE",         1),
        ("CFG_TMODE_SVIN_MIN_DUR", duration),
        ("CFG_TMODE_SVIN_ACC_LIMIT", acc_raw),
    ]
    send_cfg(gnss_port, gnss_baud, cfg)
    print(f"Survey-In started -- will complete after {duration}s.")


def set_gnss_output_rate(gnss_port, gnss_baud, rate_hz):
    """
    Sets the GNSS measurement/nav rate via UBX CFG_RATE_MEAS (period in ms)
    and CFG_RATE_NAV (nav solutions per measurement, left at 1). e.g.
    rate_hz=0.5 -> one epoch every 2000ms, giving the LoRa radio twice as
    much time per epoch to get everything out over the air.
    """
    period_ms = int(round(1000 / rate_hz))
    cfg = [
        ("CFG_RATE_MEAS", period_ms),
        ("CFG_RATE_NAV", 1),
    ]
    send_cfg(gnss_port, gnss_baud, cfg)
    print(f"GNSS output rate set to {rate_hz}Hz (measurement period {period_ms}ms).")


def poll_svin(ser):
    ser.reset_input_buffer()
    ser.write(UBXMessage("NAV", "NAV-SVIN", POLL).serialize())
    ubr = UBXReader(ser)
    for _ in range(200):
        _, parsed = ubr.read()
        if parsed and parsed.identity == "NAV-SVIN":
            return parsed
    return None


def check_survey(gnss_port, gnss_baud, poll_interval=5):
    print(f"\nPolling survey status every {poll_interval}s (Ctrl+C to stop)...\n")
    ser = Serial(gnss_port, gnss_baud, timeout=2)
    try:
        while True:
            p = poll_svin(ser)
            if p is None:
                print("  No NAV-SVIN response.")
            else:
                raw_acc = getattr(p, "meanAcc", None)
                acc_mm  = round(raw_acc / 10, 1) if isinstance(raw_acc, (int, float)) else "?"
                valid   = getattr(p, "valid", "?")
                print(f"  active={getattr(p,'active','?')}  valid={valid}  "
                      f"dur={getattr(p,'dur','?')}s  meanAcc={acc_mm}mm")
                if str(valid) in ("1", "True"):
                    print("\n Survey converged (valid=1).\n")
                    return
            time.sleep(poll_interval)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


# ── LoRa transmit ─────────────────────────────────────────────────────────────

def lora_send_message(lora_ser, msg_type: int, raw_bytes: bytes, epoch_seq: int) -> bool:
    """
    Send one RTCM3 message over LoRa with TX-stamped seq_id. (v2)

    Wire format:
        Single:  AT+SEND=0,<len>,<TYPE>:<SEQ><hex>
        Split 1: AT+SEND=0,<len>,<TYPE>_1:<SEQ><hex[:231]>
        Split 2: AT+SEND=0,<len>,<TYPE>_2:<SEQ><hex[231:]>

    <SEQ> is a zero-padded 2-digit decimal (00-31) = epoch_seq % 32.
    It is the FIRST 2 characters of every data field so all drones receive
    the same value and pass it verbatim to GPS_RTCM_DATA -- keeping
    ArduPilot fragment reassembly in sync across all 4 FCs even when
    individual drones drop packets.

    The OBC RX script must strip the 2-char SEQ prefix before hex.fromhex().
    See module docstring for the exact RX parsing change.

    Returns True if all sends got +OK.
    """
    hex_str = raw_bytes.hex()
    hex_len = len(hex_str)
    seq_tag = f"{epoch_seq % 32:02d}"   # "00".."31"

    if hex_len <= MAX_HEX_CHARS:
        sends = [(f"{msg_type}:", seq_tag + hex_str)]
    else:
        sends = [
            (f"{msg_type}_1:", seq_tag + hex_str[:MAX_HEX_CHARS]),
            (f"{msg_type}_2:", seq_tag + hex_str[MAX_HEX_CHARS:]),
        ]

    lora_ser.reset_input_buffer()

    all_ok = True
    for header, data in sends:
        payload = header + data
        cmd = f"AT+SEND=0,{len(payload)},{payload}\r\n"
        t0 = time.time()
        lora_ser.write(cmd.encode("ascii"))
        reply = wait_for_ok(lora_ser)
        elapsed = time.time() - t0
        ok = "+OK" in reply
        all_ok = all_ok and ok
        status = "OK" if ok else f"FAIL({reply!r})"
        print(f"  TX {header[:-1]:8s} | seq={seq_tag} | {len(payload):3d} chars | "
              f"{elapsed:.2f}s | {status}")
        time.sleep(0.02)   # 20ms gap between chunks (corrected from misleading '200ms' comment)

    return all_ok


# ── GNSS reader thread ────────────────────────────────────────────────────────

def gnss_reader_worker(gnss_ser, out_queue, stop_event):
    """
    Runs on its own thread. Continuously reads RTCM3 messages off the GNSS
    serial port and pushes the ones we care about onto out_queue.
    """
    rtr = RTCMReader(gnss_ser, quitonerror=0)
    for raw, parsed in rtr:
        if stop_event.is_set():
            break
        if parsed is None or raw is None:
            continue
        try:
            mt = int(parsed.identity)
        except (TypeError, ValueError):
            continue
        if mt not in KEEP_TYPES:
            continue

        try:
            out_queue.put_nowait((mt, raw))
        except queue.Full:
            try:
                out_queue.get_nowait()  # drop oldest
            except queue.Empty:
                pass
            try:
                out_queue.put_nowait((mt, raw))
            except queue.Full:
                pass


def stream_loop(gnss_port, gnss_baud, lora_port, lora_baud, lora_address):
    print(f"\nSetting GNSS output rate to {GNSS_RATE_HZ}Hz...")
    set_gnss_output_rate(gnss_port, gnss_baud, GNSS_RATE_HZ)
    time.sleep(0.5)  # give the receiver a moment to apply the new rate

    gnss_ser = Serial(gnss_port, gnss_baud, timeout=2)
    
    # [FIX] The LoRa auto-config took ~2 seconds, during which GNSS was spitting out 
    # data into the OS serial buffer. We MUST flush it so the RTCMReader doesn't 
    # read a stale, cut-off message (which causes the 1124_1 dropped packets!)
    gnss_ser.reset_input_buffer()
    time.sleep(0.1)
    
    lora_ser = Serial(lora_port, lora_baud, timeout=1)

    time.sleep(0.2)
    lora_ser.reset_input_buffer()

    # [FIX] Removed duplicate configure_lora() call here - already done in auto_config

    print(f"\nStreaming 1005+1074+1084+1094+1124+1230 from {gnss_port} -> LoRa {lora_port} "
          f"(Ctrl+C to stop)\n")

    msg_queue = queue.Queue(maxsize=QUEUE_MAXSIZE)
    stop_event = threading.Event()
    reader_thread = threading.Thread(
        target=gnss_reader_worker,
        args=(gnss_ser, msg_queue, stop_event),
        daemon=True,
    )
    reader_thread.start()

    epoch_buf = {}
    epoch_num = 0

    try:
        while True:
            try:
                mt, raw = msg_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            if mt in epoch_buf:
                print(f"\n[Epoch {epoch_num} incomplete, flushing]\n")
                epoch_buf = {}

            epoch_buf[mt] = raw

            print(f"\nEpoch {epoch_num + 1} | msg {mt} | {len(raw)}B raw "
                  f"| seq={epoch_num % 32:02d} | queue depth {msg_queue.qsize()}")

            # Pass current epoch_num as seq source -- all 4 drones receive the
            # same seq for this epoch regardless of per-drone packet loss.
            lora_send_message(lora_ser, mt, raw, epoch_seq=epoch_num)

            # 10ms gap between messages (wait_for_ok already gates on +OK)
            time.sleep(0.01)

            if EPOCH_COMPLETE.issubset(epoch_buf):
                epoch_num += 1
                print(f"--- Epoch {epoch_num} complete (next seq={epoch_num % 32:02d}) ---\n")
                epoch_buf = {}

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        stop_event.set()
        gnss_ser.close()
        lora_ser.close()


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("RTK LoRa TX v2  --  All 4 MSM4 constellations RTCM3 broadcast [SEQ-STAMPED]\n")

    gnss_port  = input("GNSS base COM port (e.g. COM3): ").strip()
    
    # FORCE HARDWARE AUTO-CONFIGURATION BEFORE LOAD
    auto_configure_base(gnss_port)
    
    gnss_baud_s = input("GNSS baud [Enter for 115200]: ").strip()
    gnss_baud  = int(gnss_baud_s) if gnss_baud_s else 115200

    while True:
        print("""
  1) Start Survey-In   (duration only)
  2) Check Survey-In status
  3) Start RTCM stream over LoRa
  4) Exit
""")
        choice = input("> ").strip()

        if choice == "4":
            print("Exiting.")
            break

        try:
            if choice == "1":
                dur = int(input("Survey duration (seconds, e.g. 60): ").strip())
                start_survey(gnss_port, gnss_baud, dur)

            elif choice == "2":
                iv = input("Poll interval [Enter for 5s]: ").strip()
                check_survey(gnss_port, gnss_baud, int(iv) if iv else 5)

            elif choice == "3":
                lora_port   = input("LoRa TX COM port (e.g. COM5): ").strip()
                print("Hardcoded module ADDRESS: 100")
                lora_addr   = 100
                
                auto_configure_lora(lora_port)
                stream_loop(gnss_port, gnss_baud, lora_port, LORA_BAUD, lora_addr)

            else:
                print("Unrecognized option.")

        except SerialException as e:
            print(f"Serial error: {e}")


if __name__ == "__main__":
    main()