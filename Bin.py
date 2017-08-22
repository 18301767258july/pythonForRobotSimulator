#!/usr/bin/env python3
# -*- coding: utf-8 -*-



from Sensor import Sensor





class Bin(object):
	def __init__(self,num):
		self.buffers=[]
		for i in range(0,num):
			self.buffers.append(Sensor(str(i),False))