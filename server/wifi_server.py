import socket
from commands import Commands

HOST = "10.0.0.219" # IP address of your Raspberry PI
PORT = 65432          # Port to listen on (non-privileged ports are > 1023)

with (socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s):
    s.bind((HOST, PORT))
    s.listen()

    commands = Commands()

    try:
        while True:
            try:
                client, clientInfo = s.accept()
                print("server recv from: ", clientInfo)
                data = client.recv(1024)      # receive 1024 Bytes of message in binary format
                if decoded_data != b"":
                    print(f"Received: {data}")
                    decoded_data = data.decode('utf-8').strip()
                    if decoded_data == 'forward':
                        commands.forward()
                        client.sendall("moving foward")
                    elif decoded_data == 'stop':
                        commands.cancel_movement()
                        client.sendall("cancel movement")
                    elif decoded_data == 'scan':
                        commands.scan_env()
                        client.sendall("scanning")
                    elif decoded_data == 'see':
                        commands.start_vision()
                        client.sendall("starting vision")
                    else:
                        client.sendall("unknown command")
            except Exception as e:
                print(f"Error handling client: {e}")
            finally:
                client.close()

    except KeyboardInterrupt:
        print("\nShutting down server")
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        s.close()