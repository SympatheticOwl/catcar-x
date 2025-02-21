import string
from flask import Flask
from flask import render_template
from flask import request, jsonify
from commands import Commands

def greet(name):
    return "Hi " + name + " . Python server sends its regards."

app = Flask(__name__)
greeting = " "
commands = None

@app.before_first_request
def load_global_data():
    global commands
    commands = Commands()
@app.route('/', methods=["GET", "POST"])
def index():
    global greeting

    # recieve message from electron app
    if request.method == "POST":
        json_message = request.get_json()
        print(json_message)
        greeting = greet(json_message)
        return jsonify(server_greet = greeting)        

    # return 'Iot is fun!'
    return jsonify(server_greet = greeting)

@app.route('/commands/<command>', methods=["GET"])
def command(command: string):
    global commands
    if command == 'forward':
        commands.forward()
        return "moving forward"
    elif command == 'stop':
        commands.cancel_movement()
        return "cancel movement"
    elif command == 'scan':
        commands.scan_env()
        return "scanning"
    elif command == 'see':
        commands.start_vision()
        print("starting vision")
        return "starting vision"
    else:
        print("Command not found")
        return "unknown command"

if __name__ == '__main__':
    app.run( host='10.0.0.219', port = 5000, debug=True)
