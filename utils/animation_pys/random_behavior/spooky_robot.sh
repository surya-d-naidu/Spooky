#!/bin/bash

# Spooky Robot - Random Behavior Startup Script
# This script starts the robot's random behavior system
# Perfect for SSH-free startup and systemd integration

# Set script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Configuration
LOG_FILE="/tmp/spooky_startup.log"
PID_FILE="/tmp/spooky_random_behavior.pid"
PYTHON_SCRIPT="$SCRIPT_DIR/run_continuous_random.py"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging function
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

# Print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
    log "$1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
    log "WARNING: $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
    log "ERROR: $1"
}

print_header() {
    echo -e "${BLUE}"
    echo "=========================================="
    echo "🤖 SPOOKY ROBOT - RANDOM BEHAVIOR STARTUP"
    echo "=========================================="
    echo -e "${NC}"
}

# Check if already running
check_if_running() {
    if [ -f "$PID_FILE" ]; then
        PID=$(cat "$PID_FILE")
        if ps -p "$PID" > /dev/null 2>&1; then
            return 0  # Running
        else
            rm -f "$PID_FILE"  # Stale PID file
            return 1  # Not running
        fi
    fi
    return 1  # Not running
}

# Start function
start_robot() {
    print_header
    
    if check_if_running; then
        print_warning "Robot is already running! (PID: $(cat $PID_FILE))"
        exit 1
    fi
    
    print_status "Starting Spooky Robot Random Behavior System..."
    
    # Check if Python script exists
    if [ ! -f "$PYTHON_SCRIPT" ]; then
        print_error "Python script not found: $PYTHON_SCRIPT"
        exit 1
    fi
    
    # Change to the correct directory
    cd "$SCRIPT_DIR" || {
        print_error "Failed to change to script directory: $SCRIPT_DIR"
        exit 1
    }
    
    print_status "Working directory: $(pwd)"
    print_status "Log file: $LOG_FILE"
    print_status "PID file: $PID_FILE"
    
    # Start the Python script in background
    nohup python3 "$PYTHON_SCRIPT" >> "$LOG_FILE" 2>&1 &
    PID=$!
    
    # Save PID
    echo $PID > "$PID_FILE"
    
    # Wait a moment and check if it started successfully
    sleep 3
    if ps -p "$PID" > /dev/null 2>&1; then
        print_status "✅ Robot started successfully! (PID: $PID)"
        print_status "Monitor logs with: tail -f $LOG_FILE"
        print_status "Stop with: $0 stop"
    else
        print_error "❌ Failed to start robot!"
        rm -f "$PID_FILE"
        exit 1
    fi
}

# Stop function
stop_robot() {
    print_status "Stopping Spooky Robot..."
    
    if check_if_running; then
        PID=$(cat "$PID_FILE")
        print_status "Sending TERM signal to PID: $PID"
        kill -TERM "$PID" 2>/dev/null
        
        # Wait for graceful shutdown
        for i in {1..10}; do
            if ! ps -p "$PID" > /dev/null 2>&1; then
                print_status "✅ Robot stopped gracefully"
                rm -f "$PID_FILE"
                return 0
            fi
            sleep 1
        done
        
        # Force kill if still running
        print_warning "Forcing shutdown..."
        kill -KILL "$PID" 2>/dev/null
        rm -f "$PID_FILE"
        print_status "🛑 Robot stopped (forced)"
    else
        print_warning "Robot is not running"
    fi
}

# Status function
status_robot() {
    if check_if_running; then
        PID=$(cat "$PID_FILE")
        print_status "🟢 Robot is RUNNING (PID: $PID)"
        print_status "Log file: $LOG_FILE"
        echo ""
        echo "Recent log entries:"
        tail -10 "$LOG_FILE" 2>/dev/null || echo "No logs available"
    else
        print_status "🔴 Robot is NOT running"
    fi
}

# Restart function
restart_robot() {
    print_status "Restarting Spooky Robot..."
    stop_robot
    sleep 2
    start_robot
}

# Show logs
show_logs() {
    if [ -f "$LOG_FILE" ]; then
        echo "📄 Showing recent logs (Press Ctrl+C to exit):"
        tail -f "$LOG_FILE"
    else
        print_warning "No log file found at: $LOG_FILE"
    fi
}

# Help function
show_help() {
    echo "Usage: $0 {start|stop|restart|status|logs|help}"
    echo ""
    echo "Commands:"
    echo "  start    - Start the robot's random behavior"
    echo "  stop     - Stop the robot"
    echo "  restart  - Restart the robot"
    echo "  status   - Show current status"
    echo "  logs     - Show live logs (tail -f)"
    echo "  help     - Show this help message"
    echo ""
    echo "Files:"
    echo "  Log file: $LOG_FILE"
    echo "  PID file: $PID_FILE"
    echo "  Script:   $PYTHON_SCRIPT"
}

# Main script logic
case "$1" in
    start)
        start_robot
        ;;
    stop)
        stop_robot
        ;;
    restart)
        restart_robot
        ;;
    status)
        status_robot
        ;;
    logs)
        show_logs
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        echo -e "${RED}Error:${NC} Unknown command '$1'"
        echo ""
        show_help
        exit 1
        ;;
esac

exit 0
