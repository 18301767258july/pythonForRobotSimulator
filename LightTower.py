#!/usr/bin/env python3
# -*- coding: utf-8 -*-



class LightTower(object):
	def __init__(self):
		self.__red=False
		self.__yellow=False
		self.__green=True
		self.__buzzer=False
	def get_red(self):
		return self.__red
	def set_red(self,state):
		self.__red=state
	def get_yellow(self):
		return self.__yellow
	def set_yellow(self,state):
		self.__yellow=state
	def get_green(self):
		return self.__green
	def set_green(self,state):
		self.__green=state
	def get_buzzer(self):
		return self.__buzzer
	def set_buzzer(self,state):
		self.__buzzer
		
