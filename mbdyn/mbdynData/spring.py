# exec "mbdyn body"
# exec "python spring.py"

import socket
from socket import error as SocketError
import struct
import errno
import numpy as np

# create output socket (1 double: f)
s_out = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM);
s_out.connect("./mbdyn.spring.sock");
s_out_bufsize = 8

# initial value
f = 0.

# step counter
i = 0
while 1:
	# send force to body's file driver
	#f = np.deg2rad(45*np.sin(np.pi/8*i))
	#import ipdb; ipdb.set_trace()
	#f = np.deg2rad(0.01*((1+i)%2))
	if i == 0:
		f =  np.deg2rad(0)
	if 0 < i < 10:
		f =  np.deg2rad(10)
	if 10 < i < 20:
		f =  np.deg2rad(20)
	if 20 < i < 30:
		f =  np.deg2rad(30)
	if 30 < i < 40:
		f =  np.deg2rad(20)
	if 40 < i < 50:
		f =  np.deg2rad(10)
	if 50 < i:
		f =  np.deg2rad(0)
	#f *= 100
	buf_out = bytearray(struct.pack("d",f))
	try:
		rc = s_out.send(buf_out, s_out_bufsize)
	except SocketError as e:
		if e.errno != errno.ECONNRESET:
		    raise # Not error we are looking for
		print("sent %d bytes; closing..." % rc)
		break
	if (rc != s_out_bufsize):
		print("sent %d bytes; closing..." % rc)
		break
	i = i + 1

s_out.close()
