import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(2)                                      #2 Second Timeout
result = sock.connect_ex(('localhost', 65433))
sock.close()
if result == 0:
  print('port OPEN')
else:
  print('port CLOSED, connect_ex returned: '+str(result))