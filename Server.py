#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import socket
import queue
from select import select
import time 
import threading


class Server(object):
	def __init__(self,ip,port):
		self.ip=ip
		self.port=port
		self.mutex = threading.Lock()
		self.SERVER_IP = (ip, port)
		self.message_queue = {}
		self.input_list = []
		self.output_list = []
		self.server=socket.socket()
		self.server.bind(self.SERVER_IP)
		self.server.listen(10)
		self.server.setblocking(False)
		self.input_list.append(self.server)
		self.client=None
		
	def listen(self):
		#while True:
		self.stdinput,self.stdoutput,self.stderr=select(self.input_list,self.output_list,self.input_list)
		
		for obj in self.stdinput:
			if obj == self.server:
				cli,address=self.server.accept()
				print("Client %s connected!" % cli)
				self.client=cli
		
	def start_listen(self):
		
		tcam=threading.Thread(target=self.listen)
		tcam.setDaemon(False)
		tcam.start()

	
	def stop_listen(self):
		self.server.close()
		self.server=None
	def receive(self):
		while True:
			for conn in self.input_list:
				if conn != self.server:
					try:
						recv_data=conn.recv(1024)
						#return recv_data
						if recv_data:
							print("receive data: %s " % recv_data)
							self.message_queue[conn].put(recv_data)
					except ConnectionResetError:
						print("Client %s disconnected !" % conn)
						self.input_list.remove(conn)
						del self.message_queue[conn]
						return
					
					
	def send(self,mes):
		for sendobj in self.output_list:
			try:
				if not self.message_queue[sendobj].empty():
					sendobj.sendall(mes)

				else:
					self.output_list.remove(sendobj)
			except ConnectionResetError:
				print("Client %s disconnected !" % sendobj)
				self.message_queue[sendobj]
				self.output_list.remove(sendobj)
				
'''				
if __name__ == "__main__":
	ser=Server('127.0.0.1',49152)
	ser.start_listen()
	
	time.sleep(20)
	
	for x in range(11):
		ser.receive()
		ser.send("2222222")
'''




