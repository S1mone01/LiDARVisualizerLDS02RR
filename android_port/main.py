import os
import threading
import json
import time
import sys
from kivy.app import App
from kivy.utils import platform

# Helper per eseguire codice sul thread principale
if platform == 'android':
    from android.runnable import run_on_ui_thread
else:
    def run_on_ui_thread(func):
        return func

# Logica di Parsing LiDAR
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
                        
                        valid_intensities = [i for i, d in zip(self.current_intensities, self.current_scan) if d > 0]
                        avg_intensity = sum(valid_intensities) / len(valid_intensities) if valid_intensities else 0
                        
                        payload = {
                            'scan': self.lidar_data["scan"],
                            'intensities': self.lidar_data["intensities"],
                            'errors': self.lidar_data["errors"],
                            'count': points,
                            'hz': self.lidar_data["hz"],
                            'rpm': self.lidar_data["rpm"],
                            'avg_intensity': avg_intensity,
                            'error_rate': (self.lidar_data["bad_frames"] / self.lidar_data["total_frames"]) * 100 if self.lidar_data["total_frames"] > 0 else 0,
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

# Gestore USB per Android
class AndroidUSBSerial:
    def __init__(self, app_instance):
        self.app = app_instance
        self.device = None
        self.serial_port = None
        self.stop_thread = threading.Event()
        self.lidar_logic = LidarLogic()
        
    def log_to_js(self, msg, color=None):
        if platform == 'android':
            import base64
            # Usa Base64 per evitare errori di sintassi JS a causa di apici nei messaggi
            safe_msg = base64.b64encode(msg.encode('utf-8')).decode('utf-8')
            safe_color = color or 'var(--text)'
            @run_on_ui_thread
            def do_log():
                try:
                    self.app.webview.loadUrl(f"javascript:updateStatus(atob('{safe_msg}'), '{safe_color}')")
                    self.app.webview.loadUrl(f"javascript:appendLog(atob('{safe_msg}'), '{safe_color}')")
                except:
                    pass
            do_log()

    def start_reading(self):
        if platform != 'android':
            return
            
        @run_on_ui_thread
        def _start_reading_ui():
            try:
                self.log_to_js("Avvio procedura USB...", "#aaa")
                from jnius import autoclass
                from usbserial4a import serial4a

                PythonActivity = autoclass('org.kivy.android.PythonActivity')
                Context = autoclass('android.content.Context')
                PendingIntent = autoclass('android.app.PendingIntent')
                Intent = autoclass('android.content.Intent')
                
                self.log_to_js("Importati moduli jnius.", "#aaa")
                
                activity = PythonActivity.mActivity
                usb_manager = activity.getSystemService(Context.USB_SERVICE)
                
                self.log_to_js("Ottenuto UsbManager.", "#aaa")
                
                device_list = usb_manager.getDeviceList()
                if device_list.isEmpty():
                    self.log_to_js("Nessun disp. USB", "var(--danger)")
                    return
                
                self.device = device_list.values().iterator().next()
                self.log_to_js(f"Trovato: {self.device.getDeviceName()}", "#aaa")
                
                if not usb_manager.hasPermission(self.device):
                    self.log_to_js("Permessi mancanti. Prearo Intent.", "#aaa")
                    
                    ACTION_USB_PERMISSION = "org.lidar.sistemalidar.USB_PERMISSION"
                    intent = Intent(ACTION_USB_PERMISSION)
                    intent.setPackage(activity.getPackageName())
                    self.log_to_js(f"Pkg: {activity.getPackageName()}", "#aaa")
                    
                    flags = 33554432 | 134217728
                    self.log_to_js(f"Flags: {flags}", "#aaa")
                    
                    permission_intent = PendingIntent.getBroadcast(activity, 0, intent, flags)
                    self.log_to_js("PendingIntent creato.", "#aaa")
                    
                    usb_manager.requestPermission(self.device, permission_intent)
                    self.log_to_js("requestPermission eseguito.", "var(--warning)")
                    self.log_to_js("PREMI DI NUOVO DOPO L'OK", "var(--warning)")
                    return

                self.log_to_js("Permessi OK. Apro seriale.", "#aaa")
                device_name = self.device.getDeviceName()
                self.serial_port = serial4a.get_serial_port(device_name, 115200, timeout=0.1)
                
                if not self.serial_port:
                    self.log_to_js("Errore driver FTDI", "var(--danger)")
                    return
                    
                if not self.serial_port.is_open:
                    self.serial_port.open()
                    self.log_to_js("Porta aperta con successo.", "#aaa")

                self.stop_thread.clear()
                threading.Thread(target=self._read_loop, daemon=True).start()
                self.log_to_js("CONNESSO", "var(--primary)")
                
            except Exception as e:
                import traceback
                err_str = traceback.format_exc()
                print(err_str)
                self.log_to_js(f"Err: {err_str}", "var(--danger)")

        _start_reading_ui()

    def _read_loop(self):
        while not self.stop_thread.is_set():
            try:
                # usbserial4a si comporta come pyserial, decodifica da solo i pacchetti FTDI
                data = self.serial_port.read(4096)
                if data and len(data) > 0:
                    payloads = self.lidar_logic.process_data(data)
                    for p in payloads:
                        self.send_to_js(p)
            except Exception as e:
                self.log_to_js("Errore lettura", "var(--danger)")
                break

    def send_to_js(self, payload):
        if platform == 'android':
            import base64
            json_data = json.dumps(payload)
            # Uso Base64 per evitare problemi di escaping nelle stringhe js
            encoded_data = base64.b64encode(json_data.encode('utf-8')).decode('utf-8')
            @run_on_ui_thread
            def update():
                try:
                    self.app.webview.loadUrl(f"javascript:updateLidarData(atob('{encoded_data}'))")
                except:
                    pass
            update()

# Kivy App
class LidarApp(App):
    def build(self):
        try:
            self.usb_manager = AndroidUSBSerial(self)
            
            if platform == 'android':
                @run_on_ui_thread
                def setup_webview():
                    from jnius import autoclass, PythonJavaClass, java_method
                    WebView = autoclass('android.webkit.WebView')
                    WebChromeClient = autoclass('android.webkit.WebChromeClient')
                    CustomWebViewClient = autoclass('org.lidar.CustomWebViewClient')
                    activity = autoclass('org.kivy.android.PythonActivity').mActivity
                    
                    self.webview = WebView(activity)
                    settings = self.webview.getSettings()
                    settings.setJavaScriptEnabled(True)
                    settings.setDomStorageEnabled(True)
                    settings.setAllowFileAccess(True)
                    settings.setAllowContentAccess(True)
                    try:
                        settings.setMixedContentMode(0)
                        settings.setAllowFileAccessFromFileURLs(True)
                        settings.setAllowUniversalAccessFromFileURLs(True)
                    except:
                        pass
                    
                    # Interfaccia Python -> Java per intercettare l'URL
                    class UsbCallback(PythonJavaClass):
                        __javainterfaces__ = ['org/lidar/CustomWebViewClient$Callback']
                        __javacontext__ = 'app'
                        
                        def __init__(self, usb_manager):
                            super().__init__()
                            self.usb_manager = usb_manager
                            
                        @java_method('()V')
                        def onConnectUsb(self):
                            self.usb_manager.start_reading()

                    self.usb_callback = UsbCallback(self.usb_manager)
                    self.webview.setWebViewClient(CustomWebViewClient(self.usb_callback))
                    self.webview.setWebChromeClient(WebChromeClient())
                    
                    # Caricamento file locale
                    base_path = os.path.dirname(os.path.abspath(__file__))
                    index_path = os.path.join(base_path, "templates", "index.html")
                    self.webview.loadUrl("file://" + index_path)
                    
                    activity.setContentView(self.webview)
                
                setup_webview()
                from kivy.uix.widget import Widget
                return Widget() 
            else:
                from kivy.uix.label import Label
                return Label(text="Esegui su Android")
                
        except Exception as e:
            from kivy.uix.label import Label
            import traceback
            return Label(text=f"CRASH:\n{str(e)}\n{traceback.format_exc()}")

if __name__ == '__main__':
    LidarApp().run()
