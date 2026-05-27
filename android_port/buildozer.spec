[app]
title = Sistema LiDAR
package.name = sistemalidar
package.domain = org.lidar
source.dir = .
source.include_exts = py,png,jpg,kv,atlas,html,js,css
version = 0.1

requirements = python3,kivy,flask,jinja2,markupsafe,itsdangerous,click,werkzeug,pyjnius,android

orientation = landscape
fullscreen = 1
android.permissions = USB_PERMISSION, INTERNET
android.api = 31
android.minapi = 21
android.sdk = 33
android.ndk = 25b

# Importante per il supporto USB
android.manifest.intent_filters = [ {"action": "android.hardware.usb.action.USB_DEVICE_ATTACHED"} ]
android.manifest.meta_data = [ {"name": "android.hardware.usb.action.USB_DEVICE_ATTACHED", "resource": "@xml/device_filter"} ]

[buildozer]
log_level = 2
warn_on_root = 1
