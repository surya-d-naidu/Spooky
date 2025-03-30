import servo as ML
from flask import Flask, render_template, request
import os

# Specify the path to the templates folder explicitly
app = Flask(__name__, template_folder=os.path.join(os.getcwd(), 'templates'))

def get_slider_values():
    # Get values for slider_1 to slider_12 from the POST request.
    sliders = {f"slider_{i}": request.form.get(f"slider_{i}", type=int) for i in range(1, 13)}
    return sliders

@app.route('/', methods=['GET', 'POST'])
def index():
    slider_values = {}
    if request.method == 'POST':
        slider_values = get_slider_values()
        # Send the slider values to your robot dog control module.
        ML.setFromDict(slider_values)
    return render_template('index.html', slider_values=slider_values)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001, debug=True)
