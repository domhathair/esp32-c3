import socket

HOST = '0.0.0.0'  # Your computer's IP address
PORT = 80  # Port on which the server will run

def get_local_ip():
    try:
        # Create a temporary connection to the Google server
        temp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        temp_socket.connect(("8.8.8.8", 80))
        local_ip = temp_socket.getsockname()[0]
        temp_socket.close()
        return local_ip
    except socket.error:
        return None

# Get the IP address of the computer
local_ip = get_local_ip()

if local_ip:
    print("IP-address of your PC:", local_ip)
    HOST = local_ip
else:
    print("Can not get your local IP-address.")

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print('Server started. Waiting for client...')

client_socket, addr = server_socket.accept()
print('Connected to client:', addr)

while True:
    data = client_socket.recv(PORT)
    if not data:
        break
    print('Data received:', data.decode())

client_socket.close()
server_socket.close()