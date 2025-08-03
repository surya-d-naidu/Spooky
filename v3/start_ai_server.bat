@echo off
REM Laptop AI Server Setup Script for Windows
REM Run this on your Windows laptop to start the Ollama AI server

echo 🚀 Starting Laptop AI Server for Robot...

REM Check if Ollama is installed
where ollama >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo ❌ Ollama not found. Please install from https://ollama.ai
    echo Download and run the Windows installer
    pause
    exit /b
)

REM Check if llava model is available
echo 📥 Checking for llava:7b model...
ollama list | findstr "llava:7b" >nul
if %ERRORLEVEL% NEQ 0 (
    echo 📥 Downloading llava:7b model this may take a while...
    ollama pull llava:7b
)

REM Get local IP address
echo 🔍 Detecting network IP...
for /f "tokens=2 delims=:" %%a in ('ipconfig ^| findstr /C:"IPv4 Address"') do (
    for /f "tokens=1" %%b in ("%%a") do (
        set LOCAL_IP=%%b
        goto :found
    )
)
:found

echo 📡 Your laptop IP: %LOCAL_IP%
echo 🔧 Update robot_config.py on the Pi with this IP!

REM Start Ollama server accessible from network
echo 🌐 Starting Ollama server accessible from network...
echo Robot can now connect to: http://%LOCAL_IP%:11434
echo Press Ctrl+C to stop the server

set OLLAMA_HOST=0.0.0.0:11434
ollama serve
