import os
import threading
import json
import time
import sys
from kivy.app import App
from kivy.utils import platform

# Helper per eseguire codice sul thread principale di Android
def run_on_main_thread(func):
    if platform != 'android':
        return func
    def wrapper(*args, **kwargs):
        from jnius import autoclass, PythonJavaClass, java_method
        PythonActivity = autoclass('org.kivy.android.PythonActivity')
        activity = PythonActivity.mActivity

        class PythonRunnable(PythonJavaClass):
            __javainterfaces__ = ['java/lang/Runnable']
            def __init__(self, callback):
                super(PythonRunnable, self).__init__()
                self.callback = callback
            @java_method('()V')
            def run(self):
                self.callback()

        activity.runOnUiThread(PythonRunnable(lambda: func(*args, **kwargs)))
    return wrapper

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

# Gestore USB per Android
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
            @run_on_main_thread
            def do_log():
                self.app.webview.evaluateJavascript(f"updateStatus('{msg}', '{color or 'var(--text)'}')", None)
            do_log()

    def start_reading(self):
        if platform != 'android':
            return
        try:
            from jnius import autoclass, cast
            PythonActivity = autoclass('org.kivy.android.PythonActivity')
            Context = autoclass('android.content.Context')
            UsbManager = autoclass('android.hardware.usb.UsbManager')
            UsbConstants = autoclass('android.hardware.usb.UsbConstants')
            PendingIntent = autoclass('android.app.PendingIntent')
            Intent = autoclass('android.content.Intent')
            
            activity = PythonActivity.mActivity
            usb_manager = activity.getSystemService(Context.USB_SERVICE)
            device_list = usb_manager.getDeviceList()
            
            if device_list.isEmpty():
                self.log_to_js("Nessun disp. USB", "var(--danger)")
                return
            
            # Prendi il primo dispositivo (spesso è l'unico collegato)
            self.device = device_list.values().iterator().next()
            
            # Controlla i permessi
            if not usb_manager.hasPermission(self.device):
                self.log_to_js("Richiesta Permesso...", "var(--warning)")
                
                ACTION_USB_PERMISSION = "com.android.example.USB_PERMISSION"
                intent = Intent(ACTION_USB_PERMISSION)
                
                # Flag per PendingIntent (richiesto da API 31+)
                flags = 0
                try:
                    flags = PendingIntent.FLAG_MUTABLE
                except:
                    flags = PendingIntent.FLAG_UPDATE_CURRENT
                
                permission_intent = PendingIntent.getBroadcast(activity, 0, intent, flags)
                usb_manager.requestPermission(self.device, permission_intent)
                return

            self.connection = usb_manager.openDevice(self.device)
            if not self.connection:
                self.log_to_js("Errore apertura", "var(--danger)")
                return
            
            interface = self.device.getInterface(0)
            self.connection.claimInterface(interface, True)
            
            # Cerca l'endpoint IN
            self.endpoint_in = None
            for i in range(interface.getEndpointCount()):
                ep = interface.getEndpoint(i)
                if ep.getType() == UsbConstants.USB_ENDPOINT_XFER_BULK and \
                   ep.getDirection() == UsbConstants.USB_DIR_IN:
                    self.endpoint_in = ep
                    break
            
            if not self.endpoint_in:
                self.log_to_js("Endpoint non trovato", "var(--danger)")
                return

            self.stop_thread.clear()
            threading.Thread(target=self._read_loop, daemon=True).start()
            self.log_to_js("CONNESSO", "var(--primary)")
        except Exception as e:
            self.log_to_js(f"Errore: {str(e)[:30]}", "var(--danger)")

    def _read_loop(self):
        buffer = bytearray(4096)
        while not self.stop_thread.is_set():
            try:
                # bulkTransfer(endpoint, buffer, length, timeout)
                num_bytes = self.connection.bulkTransfer(self.endpoint_in, buffer, 4096, 1000)
                if num_bytes > 0:
                    chunk = bytes(buffer[:num_bytes])
                    payloads = self.lidar_logic.process_data(chunk)
                    for p in payloads:
                        self.send_to_js(p)
                elif num_bytes < 0:
                    # Errore o timeout prolungato
                    pass
            except Exception as e:
                self.log_to_js("Errore lettura", "var(--danger)")
                break

    def send_to_js(self, payload):
        if platform == 'android':
            json_data = json.dumps(payload)
            @run_on_main_thread
            def update():
                try:
                    self.app.webview.evaluateJavascript(f"updateLidarData('{json_data}')", None)
                except:
                    pass
            update()

# Kivy App
class LidarApp(App):
    def build(self):
        try:
            self.usb_manager = AndroidUSBSerial(self)
            
            if platform == 'android':
                @run_on_main_thread
                def setup_webview():
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
                    settings.setAllowContentAccess(True)
                    try:
                        settings.setMixedContentMode(0)
                    except:
                        pass
                    
                    class WebAppInterface(PythonJavaClass):
                        __javainterfaces__ = ['android/webkit/JavascriptInterface']
                        def __init__(self, usb_handler):
                            self.usb_handler = usb_handler
                            super().__init__()
                        
                        @java_method('()V')
                        def connect_usb(self):
                            self.usb_handler.start_reading()

                    self.webview.addJavascriptInterface(WebAppInterface(self.usb_manager), "python")
                    self.webview.setWebViewClient(WebViewClient())
                    self.webview.setWebChromeClient(WebChromeClient())
                    
                    # Caricamento file locale invece di Flask
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
