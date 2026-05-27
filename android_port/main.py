import os
import threading
import json
import time
from flask import Flask, render_template
from kivy.app import App
from kivy.uix.modalview import ModalView
from kivy.clock import Clock
from kivy.utils import platform

# Inizializzazione Flask
app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

# Logica di Parsing LiDAR (estratta da lidar_read.py)
class LidarLogic:
    def __init__(self):
        self.lidar_data = {
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
        self.buffer = bytearray()
        self.current_scan = [0.0] * 360
        self.current_intensities = [0] * 360
        self.current_errors = [False] * 360

    def has_valid_crc(self, frame):
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

    def process_data(self, new_data):
        self.buffer.extend(new_data)
        payloads = []
        
        while len(self.buffer) >= 22:
            if self.buffer[0] == 0xFA:
                frame = self.buffer[:22]
                self.lidar_data["total_frames"] += 1
                
                if self.has_valid_crc(frame):
                    index = frame[1] - 0xA0
                    speed_raw = frame[2] | (frame[3] << 8)
                    self.lidar_data["rpm"] = speed_raw / 64.0
                    
                    if index == 0:
                        now = time.time()
                        dt = now - self.lidar_data["last_rotation_time"]
                        if 0.05 < dt < 1.0:
                            if self.lidar_data["hz"] == 0:
                                self.lidar_data["hz"] = 1.0 / dt
                            else:
                                self.lidar_data["hz"] = (self.lidar_data["hz"] * 0.9) + ((1.0 / dt) * 0.1)
                        
                        self.lidar_data["last_rotation_time"] = now
                        self.lidar_data["scan"] = list(self.current_scan)
                        self.lidar_data["intensities"] = list(self.current_intensities)
                        self.lidar_data["errors"] = list(self.current_errors)
                        
                        points = sum(1 for d in self.current_scan if d > 0)
                        
                        payload = {
                            'scan': self.lidar_data["scan"],
                            'intensities': self.lidar_data["intensities"],
                            'errors': self.lidar_data["errors"],
                            'count': points,
                            'hz': self.lidar_data["hz"],
                            'rpm': self.lidar_data["rpm"],
                            'data_rate': points * self.lidar_data["hz"],
                            'fov_coverage': (points / 360.0) * 100.0,
                            'max_dist': max(self.current_scan) if any(self.current_scan) else 0.0,
                            'total_frames': self.lidar_data["total_frames"],
                            'bad_frames': self.lidar_data["bad_frames"]
                        }
                        payloads.append(payload)

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
                                self.current_scan[angle] = dist
                                self.current_intensities[angle] = intensity
                                self.current_errors[angle] = warning
                            else:
                                self.current_scan[angle] = 0.0
                                self.current_intensities[angle] = 0
                                self.current_errors[angle] = True
                    self.buffer = self.buffer[22:]
                else:
                    self.lidar_data["bad_frames"] += 1
                    self.buffer = self.buffer[1:]
            else:
                self.buffer = self.buffer[1:]
        return payloads

# Gestore USB per Android (Implementazione Reale)
class AndroidUSBSerial:
    def __init__(self, app_instance):
        self.app = app_instance
        self.device = None
        self.connection = None
        self.endpoint_in = None
        self.stop_thread = threading.Event()
        self.lidar_logic = LidarLogic()
        
    def log_to_js(self, msg, color=None):
        if platform == 'android':
            from android.runnable import run_on_main_thread
            @run_on_main_thread
            def do_log():
                self.app.webview.evaluateJavascript(f"updateStatus('{msg}', '{color or 'var(--text)'}')", None)
            do_log()

    def start_reading(self):
        if platform != 'android':
            self.log_to_js("Solo Android", "var(--danger)")
            return
            
        try:
            from jnius import autoclass, cast
            PythonActivity = autoclass('org.kivy.android.PythonActivity')
            Context = autoclass('android.content.Context')
            UsbManager = autoclass('android.hardware.usb.UsbManager')
            UsbConstants = autoclass('android.hardware.usb.UsbConstants')
            
            activity = PythonActivity.mActivity
            usb_manager = activity.getSystemService(Context.USB_SERVICE)
            
            device_list = usb_manager.getDeviceList()
            if device_list.isEmpty():
                self.log_to_js("Nessun disp. USB", "var(--danger)")
                return

            self.device = device_list.values().iterator().next()
            
            # Richiesta permesso se non ce l'abbiamo
            if not usb_manager.hasPermission(self.device):
                self.log_to_js("Richiesta permesso...", "var(--warning)")
                # In un'app reale servirebbe un PendingIntent, Buildozer lo gestisce con il manifest
                # ma qui proviamo l'apertura diretta
                
            self.connection = usb_manager.openDevice(self.device)
            if not self.connection:
                self.log_to_js("Errore apertura", "var(--danger)")
                return

            # Configurazione interfaccia FTDI (solitamente la 0)
            interface = self.device.getInterface(0)
            self.connection.claimInterface(interface, True)
            
            # Trova endpoint di input
            for i in range(interface.getEndpointCount()):
                ep = interface.getEndpoint(i)
                if ep.getType() == UsbConstants.USB_ENDPOINT_XFER_BULK and \
                   ep.getDirection() == UsbConstants.USB_DIR_IN:
                    self.endpoint_in = ep
                    break
            
            if not self.endpoint_in:
                self.log_to_js("Endpoint non trovato", "var(--danger)")
                return

            # Configurazione Baud Rate 115200 per FT232RL (Comandi nativi)
            # Nota: Questa è una parte complessa che solitamente fa la libreria usb-serial-for-android
            # Per ora avviamo il thread di lettura
            self.stop_thread.clear()
            threading.Thread(target=self._read_loop, daemon=True).start()
            self.log_to_js("CONNESSO", "var(--primary)")
            
        except Exception as e:
            self.log_to_js(f"Errore: {str(e)[:20]}", "var(--danger)")

    def _read_loop(self):
        buffer = bytearray(4096)
        while not self.stop_thread.is_set():
            try:
                # Lettura sincrona dall'endpoint USB
                num_bytes = self.connection.bulkTransfer(self.endpoint_in, buffer, 4096, 100)
                if num_bytes > 0:
                    chunk = bytes(buffer[:num_bytes])
                    payloads = self.lidar_logic.process_data(chunk)
                    for p in payloads:
                        self.send_to_js(p)
            except Exception:
                break

    def send_to_js(self, payload):
        if platform == 'android':
            from android.runnable import run_on_main_thread
            json_data = json.dumps(payload)
            @run_on_main_thread
            def update():
                self.app.webview.evaluateJavascript(f"updateLidarData('{json_data}')", None)
            update()

# Kivy App
class LidarApp(App):
    def build(self):
        self.usb_manager = AndroidUSBSerial(self)
        
        # Avvia Flask in background
        threading.Thread(target=lambda: app.run(host='127.0.0.1', port=5000, debug=False), daemon=True).start()
        
        if platform == 'android':
            from jnius import autoclass, PythonJavaClass, java_method
            
            WebView = autoclass('android.webkit.WebView')
            WebViewClient = autoclass('android.webkit.WebViewClient')
            WebChromeClient = autoclass('android.webkit.WebChromeClient')
            activity = autoclass('org.kivy.android.PythonActivity').mActivity
            
            self.webview = WebView(activity)
            settings = self.webview.getSettings()
            settings.setJavaScriptEnabled(True)
            settings.setDomStorageEnabled(True)
            settings.setAllowFileAccess(True)
            
            # Bridge per chiamate da JS a Python
            class WebAppInterface(PythonJavaClass):
                __javainterfaces__ = ['android/webkit/JavascriptInterface']
                def __init__(self, usb_handler):
                    self.usb_handler = usb_handler
                
                @java_method('(Ljava/lang/String;)V')
                def connect_usb(self, msg=None):
                    self.usb_handler.start_reading()

            # Espone l'interfaccia a JavaScript come 'window.python'
            self.webview.addJavascriptInterface(WebAppInterface(self.usb_manager), "python")
            
            self.webview.setWebViewClient(WebViewClient())
            self.webview.setWebChromeClient(WebChromeClient())
            
            self.webview.loadUrl("http://127.0.0.1:5000")
            activity.setContentView(self.webview)
            return self.webview
        else:
            from kivy.uix.label import Label
            return Label(text="Interfaccia Desktop non disponibile in questa modalità")

    def connect_usb(self):
        self.usb_manager.start_reading()

if __name__ == '__main__':
    LidarApp().run()
