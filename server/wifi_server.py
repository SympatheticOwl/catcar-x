import socket
from commands import Commands

HOST = "10.0.0.219" # IP address of your Raspberry PI
PORT = 65432          # Port to listen on (non-privileged ports are > 1023)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()

    commands = Commands()

    try:
        while 1:
            client, clientInfo = s.accept()
            print("server recv from: ", clientInfo)
            data = client.recv(1024)      # receive 1024 Bytes of message in binary format
            if data != b"":
                print(data)
                if data == 'forward':
                    commands.forward()
                    client.sendall("moving foward")
                elif data == 'stop':
                    commands.cancel_movement()
                    client.sendall("cancel movement")
                elif data == 'scan':
                    commands.scan_env()
                    client.sendall("scanning")
                elif data == 'see':
                    commands.start_vision()
                    client.sendall("starting vision")
                else:
                    client.sendall("unknown command")


    except:
        print("Closing socket")
        client.close()
        s.close()    