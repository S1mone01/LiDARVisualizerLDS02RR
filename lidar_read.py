import serial
import serial.tools.list_ports
import time
import threading
import json
import sys
import os
from flask import Flask, render_template
import webview

# Gestione percorsi per PyInstaller
def get_resource_path(relative_path):
    if hasattr(sys, '_MEIPASS'):
        return os.path.join(sys._MEIPASS, relative_path)
    return os.path.join(os.path.abspath("."), relative_path)

template_dir = get_resource_path('templates')
app = Flask(__name__, template_folder=template_dir)

# Stato globale
lidar_data = {
    "scan": [0.0] * 360,
    "intensities": [0] * 360,
    "errors": [False] * 360,
    "points_in_rotation": 0,
    "hz": 0.0,
    "rpm": 0.0,
    "last_rotation_time": time.time(),
    "total_frames": 0,
    "bad_frames": 0
}

# Variabili di controllo thread
serial_thread = None
stop_event = threading.Event()
current_serial = None
main_window = None

def has_valid_crc(frame):
    data_list = []
    for t in range(10):
        data_list.append(frame[2*t] + (frame[2*t+1] << 8))
    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d
    chk32 = (chk32 & 0x7FFF) + (chk32 >> 15)
    chk32 = chk32 & 0x7FFF
    crc = frame[20] + (frame[21] << 8)
    return crc == chk32

def lidar_reader_thread(port):
    global lidar_data, current_serial, main_window
    try:
        current_serial = serial.Serial(port, 115200, timeout=0.1)
        if main_window:
            main_window.evaluate_js(f"updateConnectionStatus('connected', '{port}')")
    except Exception as e:
        if main_window:
            main_window.evaluate_js(f"updateConnectionStatus('error', '{str(e)}')")
        return

    buffer = bytearray()
    current_scan = [0.0] * 360
    current_intensities = [0] * 360
    current_errors = [False] * 360
    
    while not stop_event.is_set():
        try:
            data = current_serial.read(100)
            if not data: continue
            buffer.extend(data)
            
            while len(buffer) >= 22:
                if buffer[0] == 0xFA:
                    frame = buffer[:22]
                    lidar_data["total_frames"] += 1
                    
                    if has_valid_crc(frame):
                        index = frame[1] - 0xA0
                        speed_raw = frame[2] | (frame[3] << 8)
                        lidar_data["rpm"] = speed_raw / 64.0
                        
                        if index == 0:
                            now = time.time()
                            dt = now - lidar_data["last_rotation_time"]
                            if 0.05 < dt < 1.0:
                                if lidar_data["hz"] == 0:
                                    lidar_data["hz"] = 1.0 / dt
                                else:
                                    lidar_data["hz"] = (lidar_data["hz"] * 0.9) + ((1.0 / dt) * 0.1)
                            
                            lidar_data["last_rotation_time"] = now
                            lidar_data["scan"] = list(current_scan)
                            lidar_data["intensities"] = list(current_intensities)
                            lidar_data["errors"] = list(current_errors)
                            points_in_rotation = sum(1 for d in current_scan if d > 0)
                            lidar_data["points_in_rotation"] = points_in_rotation
                            
                            valid_intensities = [i for i, d in zip(current_intensities, current_scan) if d > 0]
                            avg_intensity = sum(valid_intensities) / len(valid_intensities) if valid_intensities else 0
                            
                            payload = {
                                'scan': lidar_data["scan"],
                                'intensities': lidar_data["intensities"],
                                'errors': lidar_data["errors"],
                                'count': lidar_data["points_in_rotation"],
                                'hz': lidar_data["hz"],
                                'rpm': lidar_data["rpm"],
                                'avg_intensity': avg_intensity,
                                'error_rate': (lidar_data["bad_frames"] / lidar_data["total_frames"]) * 100 if lidar_data["total_frames"] > 0 else 0,
                                'max_dist': max(current_scan) if any(current_scan) else 0.0,
                                'fov_coverage': (points_in_rotation / 360.0) * 100.0,
                                'data_rate': points_in_rotation * lidar_data["hz"],
                                'total_frames': lidar_data["total_frames"],
                                'bad_frames': lidar_data["bad_frames"]
                            }
                            
                            if main_window:
                                main_window.evaluate_js(f"updateLidarData({json.dumps(payload)})")

                        for j in range(4):
                            offset = 4 + 4*j
                            b0, b1, b2, b3 = frame[offset:offset+4]
                            invalid = bool(b1 & 0x80)
                            warning = bool(b1 & 0x40)
                            dist = (b0 + ((b1 & 0x3F) << 8)) / 1000.0
                            intensity = b2 | (b3 << 8)
                            angle = index * 4 + j
                            if angle < 360:
                                if not invalid:
                                    current_scan[angle] = dist
                                    current_intensities[angle] = intensity
                                    current_errors[angle] = warning
                                else:
                                    current_scan[angle] = 0.0
                                    current_intensities[angle] = 0
                                    current_errors[angle] = True
                        buffer = buffer[22:]
                    else:
                        lidar_data["bad_frames"] += 1
                        buffer = buffer[1:]
                else:
                    buffer = buffer[1:]
        except Exception:
            break
            
    if current_serial:
        current_serial.close()

# API per il frontend
class Api:
    def get_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect_lidar(self, port):
        global serial_thread, stop_event
        if not port: return False
        
        if serial_thread and serial_thread.is_alive():
            stop_event.set()
            serial_thread.join(timeout=2)
        
        stop_event.clear()
        serial_thread = threading.Thread(target=lidar_reader_thread, args=(port,), daemon=True)
        serial_thread.start()
        return True

@app.route('/')
def index():
    return render_template('index.html')

def run_flask():
    app.run(host='127.0.0.1', port=5000, debug=False, threaded=True)

if __name__ == '__main__':
    # Avvia Flask in un thread separato
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    # Crea l'istanza API
    api = Api()
    
    # Apri finestra desktop
    main_window = webview.create_window(
        'Sistema LiDAR', 
        'http://127.0.0.1:5000', 
        width=1280, 
        height=850,
        js_api=api
    )
    webview.start()
