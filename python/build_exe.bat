@echo off
setlocal
cd /d %~dp0

python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install pyinstaller

python -m PyInstaller --onefile --windowed --noconfirm --clean --name radar_gui radar_gui.py

echo Build complete. Output: dist\radar_gui.exe
pause
