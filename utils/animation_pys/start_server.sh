#!/bin/bash

# Quadruped Robot Control Server Startup Script

echo "🤖 Starting Quadruped Robot Control Server..."
echo "========================================"

# Navigate to the Spooky directory and activate existing virtual environment
echo "Navigating to Spooky directory..."
cd /home/spooky/Spooky

echo "Activating virtual environment..."
. venv/bin/activate

# Navigate to animation_pys directory
echo "Navigating to animation_pys directory..."
cd utils/animation_pys

# Install requirements
echo "Installing/checking dependencies..."
pip install -r requirements.txt

# Start the Flask server
echo "Starting Flask server..."
echo "Server will be available at:"
echo "  - Main interface: http://0.0.0.0:5000"
echo "  - Control panel: http://0.0.0.0:5000/control"
echo "  - API endpoints: http://0.0.0.0:5000/api/"
echo ""
echo "Available robot commands:"
echo "  - /api/move/forward"
echo "  - /api/move/backward"
echo "  - /api/move/left"
echo "  - /api/move/right"
echo "  - /api/move/stop"
echo "  - /api/move/stand"
echo "  - /api/move/sit"
echo "  - /api/emergency_stop"
echo ""
echo "Press Ctrl+C to stop the server"
echo "========================================"

python flask_server.py
