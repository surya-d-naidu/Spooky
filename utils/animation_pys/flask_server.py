from flask import Flask, render_template, request, jsonify
import main
import threading
import time

app = Flask(__name__)

# Robot state
robot_state = {
    'current_action': 'stopped',
    'is_moving': False,
    'last_command': None
}

@app.route('/')
def index():
    """Renders the main landing page."""
    return render_template('index.html')

@app.route('/control')
def control_panel():
    """Renders the robot control panel."""
    return render_template('control.html')

@app.route('/api/move/<direction>')
def move_robot(direction):
    """Handles robot movement commands."""
    global robot_state
    
    try:
        if direction == 'forward':
            main.animation_controller.start_animation('walk_forward')
            robot_state['current_action'] = 'walking_forward'
            robot_state['is_moving'] = True
            
        elif direction == 'backward':
            main.animation_controller.start_reverse_animation('walk_forward')
            robot_state['current_action'] = 'walking_backward'
            robot_state['is_moving'] = True
            
        elif direction == 'left':
            main.animation_controller.start_animation('rotate_left')
            robot_state['current_action'] = 'rotating_left'
            robot_state['is_moving'] = True
            
        elif direction == 'right':
            main.animation_controller.start_reverse_animation('rotate_left')
            robot_state['current_action'] = 'rotating_right'
            robot_state['is_moving'] = True
            
        elif direction == 'stop':
            main.animation_controller.stop_current_animation()
            robot_state['current_action'] = 'stopped'
            robot_state['is_moving'] = False
            
        elif direction == 'stand':
            main.animation_controller.set_idle_position()
            robot_state['current_action'] = 'standing'
            robot_state['is_moving'] = False
            
        elif direction == 'sit':
            main.animation_controller.start_animation('sit')
            robot_state['current_action'] = 'sitting'
            robot_state['is_moving'] = False
            
        else:
            return jsonify({'status': 'error', 'message': 'Invalid direction'}), 400
        
        robot_state['last_command'] = direction
        return jsonify({
            'status': 'success', 
            'action': robot_state['current_action'],
            'message': f'Robot {direction} command executed'
        })
        
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/api/status')
def get_status():
    """Returns the current status of the robot."""
    return jsonify(robot_state)

@app.route('/api/servo_data')
def get_servo_data():
    """Returns the current angles of all servos."""
    try:
        angles = main.animation_controller.get_servo_angles()
        return jsonify({'status': 'success', 'angles': angles})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/api/emergency_stop')
def emergency_stop():
    """Handles the emergency stop command."""
    global robot_state
    try:
        main.animation_controller.stop_current_animation()
        robot_state['current_action'] = 'emergency_stopped'
        robot_state['is_moving'] = False
        return jsonify({'status': 'success', 'message': 'Emergency stop activated'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/api/custom_animation/<animation_name>')
def play_custom_animation(animation_name):
    """Plays a custom animation by name."""
    global robot_state
    try:
        main.animation_controller.start_animation(animation_name)
        robot_state['current_action'] = f'playing_{animation_name}'
        robot_state['is_moving'] = True
        robot_state['last_command'] = animation_name
        return jsonify({
            'status': 'success',
            'message': f'Playing animation: {animation_name}'
        })
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/api/reverse_animation/<animation_name>')
def play_reverse_animation(animation_name):
    """Plays a custom animation in reverse."""
    global robot_state
    try:
        main.animation_controller.start_reverse_animation(animation_name)
        robot_state['current_action'] = f'playing_{animation_name}_reverse'
        robot_state['is_moving'] = True
        robot_state['last_command'] = f'{animation_name}_reverse'
        return jsonify({
            'status': 'success',
            'message': f'Playing animation in reverse: {animation_name}'
        })
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

# Error Handler for 404 Not Found
@app.errorhandler(404)
def not_found_error(error):
    """Renders a custom 404 page."""
    return render_template('404.html'), 404

if __name__ == '__main__':
    print("Starting Quadruped Robot Control Server...")
    print("Available at: http://0.0.0.0:5000")
    print("Control Panel: http://0.0.0.0:5000/control")
    app.run(host='0.0.0.0', port=5000, debug=True)