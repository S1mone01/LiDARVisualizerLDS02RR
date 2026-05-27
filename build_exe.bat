@echo off
echo Pulizia versioni precedenti...
if exist dist\Sistema_LiDAR_Tattico.exe del /f /q dist\Sistema_LiDAR_Tattico.exe
if exist build rmdir /s /q build

echo Aggiornamento pip e installazione dipendenze...
python -m pip install --upgrade pip
python -m pip uninstall -y flask-socketio eventlet gevent
python -m pip install flask pywebview pyserial pyinstaller

echo.
echo Creazione eseguibile in corso (file unico)...
python -m PyInstaller --noconfirm --onefile --windowed --add-data "templates;templates" --name "Sistema_LiDAR" lidar_read.py

echo.
echo Operazione completata! Se non vedi errori sopra, troverai l'eseguibile nella cartella 'dist'.
pause
