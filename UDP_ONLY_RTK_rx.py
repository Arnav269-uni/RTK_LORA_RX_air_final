"""
UDP_ONLY_RTK_rx.py
---------------------
Matches UDP_ONLY_RTK_tx_config.py's wire format over WiFi UDP (JSON envelope).
Forwarding logic:
    packet arrives at RX via UDP broadcast -> parsed from JSON -> forward to FC immediately

COM Ports are hardcoded: Pixhawk=COM6 (115200 baud).

Stops on Ctrl+C.
"""

import time
import socket
import json
from pymavlink import mavutil

# ==================== CONFIG ====================
WIFI_RTCM_PORT = 5010

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


# ==================== Main ====================
def main():
    fc_conn_str = "COM6"
    fc_baud = 115200

    print(f"Hardcoded Pixhawk port: {fc_conn_str} @ {fc_baud} baud")

    print(f"[mavlink] Connecting to {fc_conn_str} ...")
    mav = mavutil.mavlink_connection(fc_conn_str, baud=fc_baud)
    mav.wait_heartbeat(timeout=10)
    print(f"[mavlink] Heartbeat received from system {mav.target_system}, "
          f"component {mav.target_component}.")

    # Bind UDP socket to listen to broadcasts on all interfaces
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", WIFI_RTCM_PORT))
    
    seq_id = 0

    print(f"\nListening for RTCM3 JSON packets over UDP port {WIFI_RTCM_PORT}. Press Ctrl+C to stop.\n")
    
    try:
        while True:
            # UDP doesn't need to be polled byte-by-byte like serial, recvfrom gives whole datagrams
            data, addr = s.recvfrom(2048) 
            
            try:
                payload = json.loads(data.decode("ascii"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
                
            if payload.get("type") != "RTCM":
                continue
                
            msg_type = payload.get("msg_type")
            epoch_seq = payload.get("seq")
            hex_data = payload.get("hex")
            
            if not all(v is not None for v in (msg_type, epoch_seq, hex_data)):
                continue
                
            try:
                raw = bytes.fromhex(hex_data)
            except ValueError:
                print(f"  [rx] WARNING: invalid hex data for msg {msg_type}")
                continue
                
            print(f"From {addr[0]} | UDP msg {msg_type:4d} | seq={epoch_seq:02d} | {len(raw)}B")

            result = send_rtcm_to_fc(mav, raw, seq_id)
            seq_id = (seq_id + 1) % 32

            if result is None:
                print(f"  [{msg_type}] {len(raw)} bytes - DROPPED (exceeds max fragmentable size)")  # KEEP: critical
            elif len(result) == 1:
                flags, length = result[0]
                print(f"  [{msg_type}] {len(raw)} bytes -> sent to FC (1 packet, {length}B) - ack")
            else:
                parts = ", ".join(f"frag{((f>>1)&0x3)}:{l}B" for f, l in result)
                print(f"  [{msg_type}] {len(raw)} bytes -> sent to FC "
                      f"({len(result)} fragments: {parts}) - ack")
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        s.close()

if __name__ == "__main__":
    main()
