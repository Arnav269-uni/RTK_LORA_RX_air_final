"""
RTK_LORA_rx_v3_id_500.py  (ADDRESS=500)  --  DIAGNOSTIC / EMI logging build
ITERATION: RTK_LORA_ITERATION / v3 (ESR)
─────────────────────────────────────────
Identical to LORA_RTK_RX_id_200_v2_logs.py but with the message reconstruction
logic replaced by an ESR (Enumeration Segmentation Reconstruction) scheme to
match the v3 TX script (RTK_LORA_tx_v3.py).

Changes from v2 (LORA_RTK_RX_id_200_v2_logs.py):
──────────────────────────────────────────────────
v3 replaces the v2 per-epoch SEQ-stamping reassembler with an ESR-based
reassembler, based on Mayer et al., "RTK-LoRa: High-Precision, Long-Range
and Energy-Efficient Localization for Mobile IoT Devices" (ETH Zurich).

Why the change:
  v2's RtcmReassembler keyed _pending by msg_type alone, with no way to
  detect a stale part-1 fragment (e.g., part-2 was dropped, and a new
  epoch's part-1 for the same type silently overwrites the old entry with
  no warning). ESR fixes this by keying on a per-message MSGID that
  increments on every message sent, plus a segments-remaining countdown
  instead of a fixed _1/_2 suffix scheme.

ESR wire format parsed by this script (v3):
───────────────────────────────────────────
  All segments:  +RCV=<addr>,<len>,<MSGID>:<TYPEID>:<SEGREM>:<hexchunk>,...

  Fields:
    MSGID   — 2-digit decimal (00-99), zero-padded, wraps at MSG_ID_MODULUS=100.
              All segments of a split message carry the SAME MSGID.
              This is the fragment-grouping key (replaces v2's msg_type keying).
    TYPEID  — RTCM message type (1005, 1074, 1124, etc.), unchanged in meaning.
    SEGREM  — Segments remaining after this one.
              SEGREM=0 → either a single-segment message, or the final segment
                         of a multi-part message. Completes reassembly.
              SEGREM>0 → intermediate segment, stored in _pending under MSGID.
    hexchunk — Hex-encoded slice of the RTCM payload for this segment.

  Examples:
    Single-send 1005 (msg_id=3):
      +RCV=100,60,03:1005:0:aabbccdd..., -45, 12

    Split-send 1124 part 1 (msg_id=5):
      +RCV=100,240,05:1124:1:aabbccdd..., -45, 12
    Split-send 1124 part 2 (msg_id=5):
      +RCV=100,126,05:1124:0:eeff0011..., -45, 12

  Reassembly rules:
    1. SEGREM > 0 → store (msg_type, hexchunk) under _pending[msg_id]
    2. SEGREM == 0 AND _pending[msg_id] exists → concatenate, complete, pop
    3. SEGREM == 0 AND no _pending[msg_id] → standalone single-segment message
    4. If _pending grows beyond MAX_PENDING → evict oldest, log WARNING

  ── msg_id vs MAVLink seq_bits ──────────────────────────────────────────
  send_rtcm_to_fc() applies (seq_id & 0x1F) << 3, i.e. a 5-bit mask (mod 32).
  Feeding msg_id % 100 (range 0-99) through & 0x1F causes values 32-99 to
  alias with 0-31. This is acceptable: MAVLink's GPS_RTCM_DATA fragment
  field is a separate concern from the application-layer msg_id. The 5-bit
  seq field only needs to distinguish fragments of the *current* RTCM message
  from fragments of the *previous* message, and msg_id always differs between
  consecutive messages. This function is NOT modified from v2.
  ────────────────────────────────────────────────────────────────────────

Additional features (unchanged from v2_logs build)
───────────────────────────────────────────────────
- Every received packet is printed with its RSSI and SNR values.
- WARNING  banner when RSSI enters the degraded zone (-90 to -105 dBm).
- CRITICAL banner when RSSI drops below -105 dBm (near-loss territory).
- WARNING  banner when SNR drops below -10 dB.
- FC forwarding logic is UNCHANGED from v2.
- Dynamic LoRa hardware reset on 5-second silence.

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
LORA_BAND = 915000000         # must match TX
LORA_NETWORK_ID = 5
LORA_PARAMETER = (7, 9, 1, 12)  # SF7/BW500kHz/CR1/Preamble12 -- must match TX

MAVLINK_RTCM_MAX_FRAG_LEN = 180   
MAVLINK_RTCM_MAX_FRAGMENTS = 4    

# ==================== LOGGING THRESHOLDS ====================
RSSI_WARN_THRESHOLD     = -90    # dBm: below this prints WARNING
RSSI_CRITICAL_THRESHOLD = -105   # dBm: below this prints CRITICAL
SNR_WARN_THRESHOLD      = -10    # dB:  below this prints WARNING

# ==================== ESR CONSTANTS ====================
# Must match TX's MSG_ID_MODULUS. Used only for display/comments; the
# reassembler itself doesn't need to know the modulus since it keys on
# the raw msg_id value from each packet.
MSG_ID_MODULUS = 100

# Maximum number of unresolved pending entries in the reassembler before
# the oldest is evicted. Prevents unbounded growth over long flights
# when fragments are lost.
MAX_PENDING = 8


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
                        ("ADDRESS",   "AT+ADDRESS=500"),
                        ("NETWORKID", "AT+NETWORKID=5"),
                        ("BAND",      "AT+BAND=915000000"),
                        ("PARAMETER", "AT+PARAMETER=7,9,1,12"),
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


# ==================== TX wire-format decoder (ESR) ====================
class RtcmReassembler:
    """
    Parses the v3 ESR wire format:

        <MSGID>:<TYPEID>:<SEGREM>:<hexchunk>

    MSGID   — 2-digit decimal (00-99), the per-message counter from the TX.
              This is the fragment-grouping key (replaces v2's msg_type keying).
    TYPEID  — RTCM message type (1005, 1074, 1124, etc.).
    SEGREM  — Segments remaining after this one. 0 = final (or standalone).
    hexchunk — Hex-encoded RTCM payload slice for this segment.

    _pending is keyed by msg_id (not msg_type as in v2). Each entry stores
    (msg_type, accumulated_hex). When SEGREM==0, the entry is completed
    (or if no pending entry exists, it's treated as a standalone message).

    If _pending grows beyond MAX_PENDING, the oldest unresolved entry is
    evicted with a WARNING log.

    Returns (msg_type, raw_bytes, msg_id) on successful reassembly, or
    (None, None, None) if waiting for more segments or on parse error.
    """
    def __init__(self):
        # msg_id -> (msg_type, accumulated_hex)
        # Using a regular dict; insertion order is preserved in Python 3.7+
        # so we can evict the oldest entry by popping the first key.
        self._pending = {}

    def feed(self, ascii_payload: str):
        """
        Feed one received payload string. Returns:
          (msg_type, raw_bytes, msg_id) — on complete message
          (None, None, None)           — if waiting for more segments or error
        """
        # ── Parse ESR header ─────────────────────────────────────────────
        parts = ascii_payload.split(":", 3)
        if len(parts) != 4:
            print(f"  [rx] WARNING: malformed ESR payload (expected 4 colon-separated fields): "
                  f"{ascii_payload!r}")
            return None, None, None

        mid_str, type_str, segrem_str, hexdata = parts

        try:
            msg_id  = int(mid_str)
        except ValueError:
            print(f"  [rx] WARNING: non-numeric MSGID in: {ascii_payload!r}")
            return None, None, None
        try:
            msg_type = int(type_str)
        except ValueError:
            print(f"  [rx] WARNING: non-numeric TYPEID in: {ascii_payload!r}")
            return None, None, None
        try:
            seg_rem = int(segrem_str)
        except ValueError:
            print(f"  [rx] WARNING: non-numeric SEGREM in: {ascii_payload!r}")
            return None, None, None

        # ── SEGREM > 0: intermediate segment → store in _pending ─────────
        if seg_rem > 0:
            # Defensive: check if a DIFFERENT msg_type is already stored under
            # this msg_id (would indicate a TX-side bug or extreme corruption).
            if msg_id in self._pending:
                existing_type, _ = self._pending[msg_id]
                if existing_type != msg_type:
                    print(f"  [rx] WARNING: msg_id={msg_id:02d} already pending with "
                          f"type={existing_type}, but got type={msg_type} — "
                          f"overwriting (possible TX bug or corruption)")

            self._pending[msg_id] = (msg_type, hexdata)

            # Enforce MAX_PENDING cap: evict oldest if too many entries.
            while len(self._pending) > MAX_PENDING:
                evicted_mid = next(iter(self._pending))
                evicted_type, _ = self._pending.pop(evicted_mid)
                print(f"  [rx] WARNING: _pending overflow (>{MAX_PENDING} entries) — "
                      f"evicting stale msg_id={evicted_mid:02d} type={evicted_type}")

            return None, None, None

        # ── SEGREM == 0: final segment (or standalone) ───────────────────
        if msg_id in self._pending:
            # Completing a multi-segment message.
            stored_type, stored_hex = self._pending.pop(msg_id)

            # Defensive: check type consistency between segments.
            if stored_type != msg_type:
                print(f"  [rx] WARNING: msg_id={msg_id:02d} type mismatch: "
                      f"part-1 had type={stored_type}, final segment has type={msg_type} "
                      f"— using part-1's type")
                msg_type = stored_type

            full_hex = stored_hex + hexdata
        else:
            # Standalone single-segment message (no pending entry).
            full_hex = hexdata

        # ── Decode hex → raw bytes ───────────────────────────────────────
        try:
            raw = bytes.fromhex(full_hex)
        except ValueError:
            print(f"  [rx] WARNING: msg_id={msg_id:02d} type={msg_type} has invalid hex — dropping")
            return None, None, None

        return msg_type, raw, msg_id


# ==================== MAVLink / pymavlink RTCM forwarding ====================
def send_rtcm_to_fc(mav, frame: bytes, msg_id: int):
    """
    Forward one raw RTCM3 frame to the FC via GPS_RTCM_DATA MAVLink messages.

    NOTE ON msg_id vs seq_bits (v3 / ESR):
    This function applies (msg_id & 0x1F) << 3, which is a 5-bit mask (mod 32).
    With MSG_ID_MODULUS=100, msg_id ranges from 0 to 99, so values 32-99 will
    alias with 0-31 through the & 0x1F mask. This is acceptable: MAVLink's
    GPS_RTCM_DATA fragment field is a separate concern from the application-layer
    msg_id. The 5-bit seq field only needs to distinguish fragments of the
    *current* RTCM message from fragments of the *previous* message, and msg_id
    always differs between consecutive messages. This function is NOT modified
    from v2 — only the parameter name changed (seq_id → msg_id) and this
    comment was added.
    """
    max_frag = MAVLINK_RTCM_MAX_FRAG_LEN

    if len(frame) > max_frag * MAVLINK_RTCM_MAX_FRAGMENTS:
        return None

    seq_bits = (msg_id & 0x1F) << 3
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
    print("Hardcoded module ADDRESS: 500  [v3 ESR / DIAGNOSTIC LOGGING BUILD]")
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
    # msg_id now comes from the TX-stamped MSGID field inside each packet
    # (no local counter on the RX side)

    last_rx_time = None
    TIMEOUT_SECONDS = 5.0

    print("\nListening for RTCM3 packets over LoRa (ESR format). Press Ctrl+C to stop.")
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

            msg_type, raw, tx_msg_id = reassembler.feed(ascii_payload)
            if msg_type is None:
                continue

            # Use TX-stamped msg_id so all 4 FCs use the same value for each message.
            # NOTE: msg_id % 100 fed through & 0x1F will alias (see send_rtcm_to_fc
            # docstring for why this is acceptable).
            result = send_rtcm_to_fc(mav, raw, tx_msg_id)

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
