from flask import Flask, request, render_template
from flask_socketio import SocketIO, emit
import json

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/metrics', methods=['POST'])
def receive_metrics():
    data = request.get_json()
    # print(data)
    topic = data.get('topic')
    metric_type = data.get('metric')
    print(metric_type)
    socketio.emit('new_data_{}{}'.format(metric_type, topic.replace('/', '_')), json.dumps(data), namespace='/test')
    return "Data received", 200

@socketio.on('connect', namespace='/test')
def test_connect():
    print('Client connected')

if __name__ == '__main__':
    socketio.run(app) 
