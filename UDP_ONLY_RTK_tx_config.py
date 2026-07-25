"""
UDP_ONLY_RTK_tx_config.py  --  GNSS Base -> filter RTK corrections -> WiFi UDP broadcast

This script is a stripped-down version of the hybrid LoRa+UDP script. 
It removes all LoRa dependencies and focuses entirely on reading GNSS data
and broadcasting it over a local WiFi/UDP network.

Features:
- Auto-configures the GNSS base station (forces 115200 baud, enables MSM4, disables NMEA).
- Reads RTCM3 data using pyrtcm on a background thread.
- Broadcasts each message as a JSON envelope over UDP.
- Because UDP has no strict airtime budget, GNSS_RATE_HZ defaults to 1.0Hz.
"""

from serial import Serial, SerialException
from pyrtcm import RTCMReader
from pyubx2 import UBXMessage, UBXReader, POLL
import threading
import queue
import socket
import json
import time
import logging

logging.basicConfig(level=logging.ERROR, format="%(name)s: %(message)s")

# ── RTCM message filter ─────────────────────────────────────────────────────
KEEP_TYPES     = {1005, 1074, 1084, 1094, 1124, 1230}
EPOCH_COMPLETE = {1005, 1074, 1094, 1124}

# ── UDP Configuration ───────────────────────────────────────────────────────
WIFI_RTCM_SUBNET   = "192.168.50"    # Must match your AP subnet
WIFI_RTCM_PORT     = 5010
WIFI_RTCM_BCAST_IP = f"{WIFI_RTCM_SUBNET}.255"

# ── Timing ───────────────────────────────────────────────────────────────────
GNSS_RATE_HZ = 1.0
QUEUE_MAXSIZE = 24

# ── Hardware Auto-Configuration ─────────────────────────────────────────────

def auto_configure_base(gnss_port):
    """
    Sweeps common baud rates to find the receiver, forces it to 115200, 
    disables NMEA/MSM7 to save bandwidth, and activates MSM4 + GLONASS Bias (1230).
    """
    print(f"\n[*] Auto-configuring Base Station on {gnss_port}...")
    
    common_bauds = [115200, 38400, 57600, 9600, 230400, 460800]
    
    cfg_data = [
        ("CFG_SIGNAL_GLO_ENA", 1),
        ("CFG_SIGNAL_GLO_L1_ENA", 1),
        ("CFG_SIGNAL_GLO_L2_ENA", 1),
        ("CFG_UART1_BAUDRATE", 115200),
        ("CFG_UART1OUTPROT_RTCM3X", 1),
        ("CFG_UART1OUTPROT_UBX", 1),
        ("CFG_UART1OUTPROT_NMEA", 0),
        ("CFG_USBOUTPROT_RTCM3X", 1),
        ("CFG_USBOUTPROT_UBX", 1),
        ("CFG_USBOUTPROT_NMEA", 0),
        ("CFG_MSGOUT_RTCM_3X_TYPE1005_UART1", 1), 
        ("CFG_MSGOUT_RTCM_3X_TYPE1074_UART1", 1), 
        ("CFG_MSGOUT_RTCM_3X_TYPE1084_UART1", 1), 
        ("CFG_MSGOUT_RTCM_3X_TYPE1094_UART1", 1), 
        ("CFG_MSGOUT_RTCM_3X_TYPE1124_UART1", 1), 
        ("CFG_MSGOUT_RTCM_3X_TYPE1230_UART1", 1), 
        ("CFG_MSGOUT_RTCM_3X_TYPE1005_USB", 1),
        ("CFG_MSGOUT_RTCM_3X_TYPE1074_USB", 1),   
        ("CFG_MSGOUT_RTCM_3X_TYPE1084_USB", 1),   
        ("CFG_MSGOUT_RTCM_3X_TYPE1094_USB", 1),   
        ("CFG_MSGOUT_RTCM_3X_TYPE1124_USB", 1),   
        ("CFG_MSGOUT_RTCM_3X_TYPE1230_USB", 1),   
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


# ── Survey functions ──────────────────────────────────────────────────────────

def send_cfg(port, baud, cfg):
    ser = Serial(port, baud, timeout=2)
    msg = UBXMessage.config_set(layers=1, transaction=0, cfgData=cfg)
    ser.write(msg.serialize())
    ser.close()

def start_survey(gnss_port, gnss_baud, duration):
    acc_raw = int(round(50000 / 0.1))   # 5m in 0.1mm units
    cfg = [
        ("CFG_TMODE_MODE",         1),
        ("CFG_TMODE_SVIN_MIN_DUR", duration),
        ("CFG_TMODE_SVIN_ACC_LIMIT", acc_raw),
    ]
    send_cfg(gnss_port, gnss_baud, cfg)
    print(f"Survey-In started -- will complete after {duration}s.")

def set_gnss_output_rate(gnss_port, gnss_baud, rate_hz):
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


# ── WiFi/UDP transmit ────────────────────────────────────────────────────

def init_wifi_rtcm_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    return s

def wifi_send_message(wifi_sock, msg_type: int, raw_bytes: bytes, epoch_seq: int) -> None:
    payload = {
        "type": "RTCM",
        "msg_type": msg_type,
        "seq": epoch_seq % 32,
        "hex": raw_bytes.hex(),
        "ts": time.time(),
    }
    try:
        wifi_sock.sendto(
            json.dumps(payload).encode("ascii"),
            (WIFI_RTCM_BCAST_IP, WIFI_RTCM_PORT),
        )
        print(f"  TX(WiFi) {msg_type:5d}   | seq={payload['seq']:02d} | "
              f"{len(payload['hex'])} hex chars | sent")
    except OSError as exc:
        print(f"  TX(WiFi) send failed: {exc}")


# ── GNSS reader thread ────────────────────────────────────────────────────────

def gnss_reader_worker(gnss_ser, out_queue, stop_event):
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
                out_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                out_queue.put_nowait((mt, raw))
            except queue.Full:
                pass

def stream_loop(gnss_port, gnss_baud, wifi_sock, gnss_rate_hz=GNSS_RATE_HZ):
    print(f"\nSetting GNSS output rate to {gnss_rate_hz}Hz...")
    set_gnss_output_rate(gnss_port, gnss_baud, gnss_rate_hz)
    time.sleep(0.5)

    gnss_ser = Serial(gnss_port, gnss_baud, timeout=2)
    gnss_ser.reset_input_buffer()
    time.sleep(0.1)
    
    print(f"\nStreaming 1005+1074+1084+1094+1124+1230 from {gnss_port} -> "
          f"UDP Broadcast {WIFI_RTCM_BCAST_IP}:{WIFI_RTCM_PORT} "
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

            if mt == 1005:
                if epoch_buf:
                    print(f"--- Epoch {epoch_num} complete ---\n")
                epoch_buf = {}
                epoch_num += 1
            elif mt in epoch_buf:
                print(f"\n[Warning: Duplicate {mt} in Epoch {epoch_num}, flushing]\n")
                epoch_buf = {}
                epoch_num += 1

            epoch_buf[mt] = raw

            print(f"\nEpoch {epoch_num} | msg {mt} | {len(raw)}B raw "
                  f"| seq={epoch_num % 32:02d} | queue depth {msg_queue.qsize()}")

            wifi_send_message(wifi_sock, mt, raw, epoch_seq=epoch_num)

            # Tiny delay to prevent network stack flooding
            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        stop_event.set()
        gnss_ser.close()


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("RTK UDP TX  --  All 4 MSM4 constellations UDP Broadcast Only\n")

    gnss_port  = input("GNSS base COM port (e.g. COM3): ").strip()
    
    auto_configure_base(gnss_port)
    
    gnss_baud_s = input("GNSS baud [Enter for 115200]: ").strip()
    gnss_baud  = int(gnss_baud_s) if gnss_baud_s else 115200

    while True:
        print("""
  1) Start Survey-In   (duration only)
  2) Check Survey-In status
  3) Start UDP RTCM stream
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
                gnss_rate_s = input(
                    f"GNSS/UDP rate in Hz [Enter for {GNSS_RATE_HZ}]: "
                ).strip()
                gnss_rate_hz = float(gnss_rate_s) if gnss_rate_s else GNSS_RATE_HZ

                wifi_sock = init_wifi_rtcm_socket()
                print(f"[+] UDP Broadcasting ENABLED -> "
                      f"{WIFI_RTCM_BCAST_IP}:{WIFI_RTCM_PORT}")
                print("    (assumes the local network / AP is already connected)")

                stream_loop(gnss_port, gnss_baud, wifi_sock, gnss_rate_hz=gnss_rate_hz)
                
                # Cleanup if stream stops
                if wifi_sock:
                    wifi_sock.close()

            else:
                print("Unrecognized option.")

        except SerialException as e:
            print(f"Serial error: {e}")


if __name__ == "__main__":
    main()
