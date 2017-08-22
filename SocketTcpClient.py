#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import logging
import socket,threading
import Log
import time

class TcpSocketClient(object):
	def __init__(self,client):
		#self.ip=ip
		#self.port=port
		self.client=client
		
	def connect(self):
		self.client=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		self.client.connect((self.ip,self.port))
		return self.client
	#接收数据
	def receive(self):
		data=b''
		recv_data=''
		try:
			data=self.client.recv(1024)
			if data==b'':
				recv_data="00"
			if len(data)>0:
				recv_data=data.decode('utf-8')
		except BlockingIOError:
			pass
		except ConnectionAbortedError:
			recv_data="00"
			print('client disconnected receive')
		return recv_data
	#发送数据
	def send(self,mes):
		mes=(mes+"\r").encode('utf-8')
		if self.client!=None:
			try:
				self.client.send(mes)
			except ConnectionAbortedError:
				print('client disconnected send')
	def close(self):
		self.client.close()
	



#类变量是直接声明在类里，成员变量是带self的，局部变量是在方法中的，不带self的
























































