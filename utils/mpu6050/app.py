from flask import Flask, render_template, request
from flask_socketio import SocketIO, emit
import numpy as np

app = Flask(__name__)
socketio = SocketIO(app)

def compute_squares(q, angle_x_deg, angle_y_deg):
    """Return (s1, s2, distances) where s1/s2 are lists of 4 (x,y,z) corners."""
    # stationary square in XY plane
    h = q / 2
    s1 = np.array([
        [-h, -h, 0],
        [ h, -h, 0],
        [ h,  h, 0],
        [-h,  h, 0],
    ])
    # build rotations
    ax = np.deg2rad(angle_x_deg)
    ay = np.deg2rad(angle_y_deg)
    Rx = np.array([
        [1,        0,         0],
        [0,  np.cos(ax), -np.sin(ax)],
        [0,  np.sin(ax),  np.cos(ax)]
    ])
    Ry = np.array([
        [ np.cos(ay), 0, np.sin(ay)],
        [      0,     1,      0    ],
        [-np.sin(ay), 0, np.cos(ay)]
    ])
    # tilt s1 → s2
    s2 = (Ry @ (Rx @ s1.T)).T
    # distances = how far each corner moved in Z
    distances = [abs(p2[2] - p1[2]) for p1, p2 in zip(s1, s2)]
    return s1.tolist(), s2.tolist(), distances

@app.route("/", methods=["GET"])
def index():
    # read query params (defaults: q=2, angle_x=30°, angle_y=45°)
    q  = float(request.args.get("q", 2.0))
    ax = float(request.args.get("angle_x", 30.0))
    ay = float(request.args.get("angle_y", 45.0))

    s1, s2, dists = compute_squares(q, ax, ay)

    return render_template(
        "index.html",
        q=q,
        angle_x=ax,
        angle_y=ay,
        s1=s1,
        s2=s2,
        distances=dists
    )

@socketio.on('compute')
def handle_compute(data):
    """Compute squares on demand via WebSocket."""
    q = data.get('q', 2.0)
    ax = data.get('angle_x', 30.0)
    ay = data.get('angle_y', 45.0)
    s1, s2, dists = compute_squares(q, ax, ay)
    emit('result', {'s1': s1, 's2': s2, 'distances': dists})

if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000, debug=True)
