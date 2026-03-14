@echo off
echo ============================================================
echo   DNA Controller Setup
echo ============================================================
echo.

REM Check Python
python --version 2>nul
if errorlevel 1 (
    echo ERROR: Python not found. Install Python 3.11+ from python.org
    pause
    exit /b 1
)

echo [1/3] Installing Python dependencies...
pip install -r requirements.txt
if errorlevel 1 (
    echo ERROR: Failed to install dependencies
    pause
    exit /b 1
)
echo      Done.
echo.

echo [2/3] Detecting EtherCAT network adapters...
python -c "import pysoem; adapters = pysoem.find_adapters(); print(); [print(f'  [{i}] {a.name} - {a.desc}') for i,a in enumerate(adapters)]; print(f'\n  Found {len(adapters)} adapter(s)')" 2>nul
if errorlevel 1 (
    echo      WARNING: Could not detect adapters. Make sure Npcap is installed.
    echo      Download Npcap from: https://npcap.com/#download
    echo      Install with "WinPcap API-compatible Mode" checked.
)
echo.

echo [3/3] Checking Npcap / WinPcap...
if exist "C:\Windows\System32\Npcap\wpcap.dll" (
    echo      Npcap found.
) else if exist "C:\Windows\System32\wpcap.dll" (
    echo      WinPcap found.
) else (
    echo      WARNING: Npcap/WinPcap not detected!
    echo      EtherCAT requires Npcap to communicate with servo drives.
    echo      Download from: https://npcap.com/#download
    echo      Install with "WinPcap API-compatible Mode" checked.
)

echo.
echo ============================================================
echo   Setup Complete
echo ============================================================
echo.
echo   To run:
echo     1. Open dna_d1.toe in TouchDesigner
echo     2. Run: python ec_server.py
echo     3. Open http://localhost:9981 in browser
echo.
echo   Ports used:
echo     9981 - TouchDesigner UI (served by TD)
echo     9982 - EtherCAT motor server (ec_server.py)
echo.
echo   Requirements:
echo     - Python 3.11+          (installed)
echo     - pip packages           (installed)
echo     - Npcap                 (for EtherCAT)
echo     - TouchDesigner         (for timeline UI)
echo     - EtherCAT adapter      (hardware)
echo     - Servo drives           (EL7-EC / Leadshine)
echo.
echo   First-time adapter setup:
echo     Edit python_soem-main\app\json\config.json
echo     Set "network_interface" to your adapter's device name
echo     (shown in adapter list above)
echo.
pause
