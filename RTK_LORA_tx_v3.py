"""
RTK_Lora_TX_v3.py  --  GNSS Base -> filter RTK corrections -> LoRa broadcast
ITERATION: RTK_LORA_ITERATION / v3 (ESR)

Changes from v2 (RTK_Lora_TX_v2.py):
─────────────────────────────────────
v3 replaces the v2 per-epoch SEQ-stamping scheme with an ESR (Enumeration
Segmentation Reconstruction) wire format, based on Mayer et al.,
"RTK-LoRa: High-Precision, Long-Range and Energy-Efficient Localization
for Mobile IoT Devices" (ETH Zurich).

Why the change:
  v2 stamped a 2-digit SEQ (epoch_num % 32) shared across all message types
  in one epoch. The RX reassembler keyed _pending by msg_type alone, with no
  way to detect a stale part-1 fragment (e.g. part-2 was dropped, and a new
  epoch's part-1 for the same type silently overwrites the old entry). ESR
  fixes this by using a per-message MSGID that increments on every message
  sent, plus a segments-remaining countdown instead of a fixed _1/_2 suffix.

ESR wire format (v3):
─────────────────────
  All segments:  AT+SEND=0,<len>,<MSGID>:<TYPEID>:<SEGREM>:<hexchunk>

  Fields:
    MSGID   — 2-digit decimal (00-99), zero-padded, wraps at MSG_ID_MODULUS=100.
              Increments once per RTCM message (not per epoch, not per segment).
              All segments of a split message carry the SAME MSGID.
    TYPEID  — RTCM message type (1005, 1074, 1124, etc.), unchanged in meaning.
    SEGREM  — Segments remaining after this one.
              Single-send: SEGREM=0.
              2-part split: part 1 has SEGREM=1, part 2 has SEGREM=0.
              Generalizes to N-part splits: SEGREM=N-1, N-2, ..., 0.
    hexchunk — Hex-encoded slice of the RTCM payload for this segment.

  Examples:
    Single-send 1005 (25B raw, 50 hex chars, msg_id=3):
      AT+SEND=0,60,03:1005:0:aabbccdd...

    Single-send 1074 (115B raw, 230 hex chars, msg_id=4):
      AT+SEND=0,240,04:1074:0:aabbccdd...

    Split-send 1124 (173B raw, 346 hex chars, msg_id=5):
      AT+SEND=0,240,05:1124:1:aabbccdd...    (first 230 hex chars)
      AT+SEND=0,126,05:1124:0:eeff0011...    (remaining 116 hex chars)

  Header budget:
    MSGID(2) + :(1) + TYPEID(4) + :(1) + SEGREM(1) + :(1) = 10 chars overhead
    MAX_HEX_CHARS = 240 - 10 = 230

  ── RX SIDE CHANGE REQUIRED ────────────────────────────────────────────
  Update RtcmReassembler.feed() on the OBC RX script (RTK_LORA_rx_v3.py):

      fields = ascii_payload.split(":", 3)
      msg_id  = int(fields[0])
      type_id = int(fields[1])
      seg_rem = int(fields[2])
      hexdata = fields[3]

  The reassembler keys _pending by msg_id (not msg_type). SEGREM > 0
  stores a pending entry; SEGREM == 0 completes it (or is a standalone
  single-segment message if no pending entry exists for that msg_id).
  ───────────────────────────────────────────────────────────────────────

Counters (two separate, do NOT conflate):
  epoch_num — per-epoch, for EPOCH_COMPLETE tracking and console display.
              Unchanged from v2.
  msg_id    — per-message, increments once per RTCM message sent (whether
              that message results in 1 or 2 AT+SEND calls). Wraps at
              MSG_ID_MODULUS. This is the ESR fragment-grouping identity.

LoRa parameters (unchanged from v2):
    AT+PARAMETER=7,9,1,12  (SF7/BW500kHz/CR1/Preamble12)
    AT+BAND=915000000
    AT+NETWORKID=5
    AT+IPR=115200

Message filter (GPS + BeiDou MSM4, unchanged from v2):
    1005  base reference position
    1074  GPS MSM4 observations
    1124  BeiDou MSM4 observations

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
# GPS + BeiDou MSM4 only -- GLONASS excluded because the air unit
# has GLONASS disabled, and Galileo is filtered out.
# 1084, 1094, and 1230 are dropped here AND at the base output
# level (see auto_configure_base). EPOCH_COMPLETE drives the per-epoch counter.
KEEP_TYPES     = {1005, 1074, 1124}
# 1084/1094/1230 not included -- GLONASS disabled, Galileo filtered
EPOCH_COMPLETE = {1005, 1074, 1124}

# ── LoRa config ──────────────────────────────────────────────────────────────
NETWORK_ID    = 5
BAND_HZ       = 915000000
LORA_BAUD     = 115200
LORA_PARAM    = "7,9,1,12"

# ── GNSS rate ────────────────────────────────────────────────────────────────
GNSS_RATE_HZ  = 1.0

# ── Timing ───────────────────────────────────────────────────────────────────
# Max single-send airtime at SF7/BW500kHz with 240-char payload ~96ms.
# Timeout = airtime + safety margin.
SEND_TIMEOUT  = 1  # seconds

# ── ESR constants ────────────────────────────────────────────────────────────
# MSGID wraps at this modulus. 100 is large enough that it cannot wrap around
# within the lifetime of a single message's fragments (a message has at most
# 2 segments, so even at 3 messages/epoch * 1Hz = 3 msg_id increments/sec,
# 100/3 ≈ 33 seconds before wrap -- far longer than any pending reassembly).
MSG_ID_MODULUS = 100

# Max hex chars in one AT+SEND payload (ESR header budget):
# Header: MSGID(2) + :(1) + TYPEID(4) + :(1) + SEGREM(1) + :(1) = 10 chars
# 240 limit - 10 (header) = 230
MAX_HEX_CHARS = 230

# Queue size: 3 message types per epoch,
# headroom to cover ~3 full epochs without dropping.
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
    disables NMEA/MSM7 to save bandwidth, and activates GPS + BeiDou
    MSM4 messages only. GLONASS (1084), Galileo (1094), and GLONASS
    biases (1230) are disabled because the air unit has GLONASS tracking
    turned off and Galileo is filtered out.
    """
    print(f"\n[*] Auto-configuring Base Station on {gnss_port}...")
    print("[*] Forcing 115200 baud, enabling GPS+BeiDou MSM4, disabling GLONASS and Galileo output...")
    
    common_bauds = [115200, 38400, 57600, 9600, 230400, 460800]
    
    cfg_data = [
        # 0. GLONASS signal tracking: left enabled so the base still uses
        #    GLONASS for its own PVT solution. Corrections are disabled below.
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
        
        # 3. ENABLE GPS + BeiDou MSM4 and base position (UART1 & USB)
        ("CFG_MSGOUT_RTCM_3X_TYPE1005_UART1", 1), # Base Position
        ("CFG_MSGOUT_RTCM_3X_TYPE1074_UART1", 1), # GPS MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1084_UART1", 0), # GLONASS MSM4 -- DISABLED
        ("CFG_MSGOUT_RTCM_3X_TYPE1094_UART1", 0), # Galileo MSM4 -- DISABLED
        ("CFG_MSGOUT_RTCM_3X_TYPE1124_UART1", 1), # BeiDou MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1230_UART1", 0), # GLONASS Code-Phase Biases -- DISABLED
        
        ("CFG_MSGOUT_RTCM_3X_TYPE1005_USB", 1),
        ("CFG_MSGOUT_RTCM_3X_TYPE1074_USB", 1),   # GPS MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1084_USB", 0),   # GLONASS MSM4 -- DISABLED
        ("CFG_MSGOUT_RTCM_3X_TYPE1094_USB", 0),   # Galileo MSM4 -- DISABLED
        ("CFG_MSGOUT_RTCM_3X_TYPE1124_USB", 1),   # BeiDou MSM4
        ("CFG_MSGOUT_RTCM_3X_TYPE1230_USB", 0),   # GLONASS Code-Phase Biases -- DISABLED

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
        ADDRESS=100, NETWORKID=5, BAND=915000000, PARAMETER=7,9,1,12, BAUD=115200
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
                    ("PARAMETER", "AT+PARAMETER=7,9,1,12"),
    
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


# ── LoRa transmit (ESR format) ───────────────────────────────────────────────

def lora_send_message(lora_ser, msg_type: int, raw_bytes: bytes, msg_id: int) -> bool:
    """
    Send one RTCM3 message over LoRa using the ESR wire format. (v3)

    Wire format:
        All segments:  AT+SEND=0,<len>,<MSGID>:<TYPEID>:<SEGREM>:<hexchunk>

    MSGID is a zero-padded 2-digit decimal (00-99) = msg_id % MSG_ID_MODULUS.
    All segments of a split message carry the SAME MSGID.
    SEGREM counts down: for a 2-part split, part 1 has SEGREM=1, part 2 has
    SEGREM=0. For a single-send message, SEGREM=0.

    msg_id increments once per call to this function (once per RTCM message),
    NOT once per epoch and NOT once per AT+SEND. The caller is responsible
    for incrementing msg_id between calls.

    Returns True if all sends got +OK.
    """
    hex_str = raw_bytes.hex()
    hex_len = len(hex_str)
    mid_tag = f"{msg_id % MSG_ID_MODULUS:02d}"   # "00".."99"

    if hex_len <= MAX_HEX_CHARS:
        # Single segment: SEGREM=0
        sends = [(f"{mid_tag}:{msg_type}:0:", hex_str)]
    else:
        # Multi-segment: first segment SEGREM=1, last SEGREM=0
        sends = [
            (f"{mid_tag}:{msg_type}:1:", hex_str[:MAX_HEX_CHARS]),
            (f"{mid_tag}:{msg_type}:0:", hex_str[MAX_HEX_CHARS:]),
        ]

    lora_ser.reset_input_buffer()

    all_ok = True
    for i, (header, data) in enumerate(sends):
        payload = header + data
        cmd = f"AT+SEND=0,{len(payload)},{payload}\r\n"
        t0 = time.time()
        lora_ser.write(cmd.encode("ascii"))
        reply = wait_for_ok(lora_ser)
        elapsed = time.time() - t0
        ok = "+OK" in reply
        all_ok = all_ok and ok
        status = "OK" if ok else f"FAIL({reply!r})"
        seg_rem = len(sends) - 1 - i
        print(f"  TX mid={mid_tag} type={msg_type:4d} seg_rem={seg_rem} | "
              f"{len(payload):3d} chars | {elapsed:.2f}s | {status}")
        time.sleep(0.02)   # 20ms gap between chunks

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
    # read a stale, cut-off message (which causes the 1124 split dropped packets!)
    gnss_ser.reset_input_buffer()
    time.sleep(0.1)
    
    lora_ser = Serial(lora_port, lora_baud, timeout=1)

    time.sleep(0.2)
    lora_ser.reset_input_buffer()

    # [FIX] Removed duplicate configure_lora() call here - already done in auto_config

    print(f"\nStreaming 1005+1074+1124 (GPS+BeiDou) [ESR format] from {gnss_port} -> LoRa {lora_port} "
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
    # ESR per-message counter -- separate from epoch_num.
    # Increments once per RTCM message sent (each call to lora_send_message),
    # NOT once per epoch or once per AT+SEND segment.
    msg_id = 0

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
                  f"| msg_id={msg_id % MSG_ID_MODULUS:02d} | queue depth {msg_queue.qsize()}")

            # Send this message using the current msg_id.
            lora_send_message(lora_ser, mt, raw, msg_id=msg_id)

            # Increment msg_id AFTER the send (all segments of this message
            # used the same msg_id; the next message gets the next one).
            msg_id += 1

            # 10ms gap between messages (wait_for_ok already gates on +OK)
            time.sleep(0.01)

            if EPOCH_COMPLETE.issubset(epoch_buf):
                epoch_num += 1
                print(f"--- Epoch {epoch_num} complete (next msg_id={msg_id % MSG_ID_MODULUS:02d}) ---\n")
                epoch_buf = {}

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        stop_event.set()
        gnss_ser.close()
        lora_ser.close()


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("RTK LoRa TX v3 (ESR)  --  GPS+BeiDou RTCM3 broadcast [MSGID+SEGREM]\n")

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
