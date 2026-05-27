[app]
title = Sistema LiDAR
package.name = sistemalidar
package.domain = org.lidar
source.dir = .
source.include_exts = py,png,jpg,kv,atlas,html,js,css
version = 0.1

requirements = python3,kivy,pyjnius,android,usb4a,usbserial4a

orientation = landscape
fullscreen = 1
android.permissions = INTERNET
android.add_src = java_src
android.api = 31
android.minapi = 21
android.sdk = 33
android.ndk = 25b

# Importante per il supporto USB
android.manifest.intent_filters = intent_filters.json
android.manifest.meta_data = [ {"name": "android.hardware.usb.action.USB_DEVICE_ATTACHED", "resource": "@xml/device_filter"} ]
android.res_dir = res

[buildozer]
log_level = 2
warn_on_root = 1
