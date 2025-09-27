#!/bin/bash

echo "Setting up AI Vision Dog System..."

if ! command -v python3 &> /dev/null; then
    echo "Python3 is required but not installed. Please install Python3."
    exit 1
fi

echo "Installing Python dependencies..."
pip3 install -r requirements.txt

echo "Checking Ollama connection..."
if ! curl -s http://localhost:11434/api/tags > /dev/null; then
    echo "Warning: Ollama server not accessible at localhost:11434"
    echo "Please ensure Ollama is running with the llava model installed:"
    echo "  ollama serve"
    echo "  ollama pull llava"
fi

echo "Testing camera access..."
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera Error'); cap.release()"

echo "Setup complete!"
echo "To run the AI Dog:"
echo "  python3 run_ai_dog.py"
echo ""
echo "Environment variables you can set:"
echo "  OLLAMA_HOST=10.96.2.180 (default: localhost)"
echo "  CAMERA_INDEX=0 (default: 0)"
