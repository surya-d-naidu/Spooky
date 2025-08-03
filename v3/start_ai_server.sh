#!/bin/bash
# Laptop AI Server Setup Script
# Run this on your laptop to start the Ollama AI server

echo "🚀 Starting Laptop AI Server for Robot..."

# Check if Ollama is installed
if ! command -v ollama &> /dev/null; then
    echo "❌ Ollama not found. Installing..."
    curl -fsSL https://ollama.ai/install.sh | sh
fi

# Check if llava model is available
echo "📥 Checking for llava:7b model..."
if ! ollama list | grep -q "llava:7b"; then
    echo "📥 Downloading llava:7b model (this may take a while)..."
    ollama pull llava:7b
fi

# Get local IP address
echo "🔍 Detecting network IP..."
if [[ "$OSTYPE" == "darwin"* ]]; then
    # macOS
    LOCAL_IP=$(ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $2}' | head -1)
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Linux
    LOCAL_IP=$(hostname -I | awk '{print $1}')
else
    # Windows/Other
    echo "Please manually find your IP address and update robot_config.py"
    LOCAL_IP="YOUR_IP_HERE"
fi

echo "📡 Your laptop IP: $LOCAL_IP"
echo "🔧 Update robot_config.py on the Pi with this IP!"

# Start Ollama server accessible from network
echo "🌐 Starting Ollama server (accessible from network)..."
echo "Robot can now connect to: http://$LOCAL_IP:11434"
echo "Press Ctrl+C to stop the server"

OLLAMA_HOST=0.0.0.0:11434 ollama serve
