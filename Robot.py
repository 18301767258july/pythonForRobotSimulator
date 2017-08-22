#!/usr/bin/env python3
# -*- coding: utf-8 -*-



import Log
from Sensor import Sensor
from Bin import Bin
from LightTower import LightTower
from enum import Enum


Robot=Enum('Robot','Running Stoped')

class ROBOT_ERROR_CODE(Enum):
	UNOVERWRITE=-1
	SUCCESS=11000
	UndefineError=11001
	SuctionTimeout=11002
	BlowTimeout=11003
	UndefineCommand=11006
	NoUnitDetect=11007
	UnitAlreadyExist=11008
	PositionIsUsed=11009
	CAMSTOP=11010
	ManualMode=11011
	EmergencyMode=11012
	MoveActionTimeout=11013
	UnknownState=11014
	NoDetectSignal=11015
	GripperMaskError=11016
	TaskAbort=11017
	BlockDetect=11018
	UndefineDeviceID=11019
	DoorNotClosed=11020
	UndefineAction=11021
	DebugPressed=11022
	SafeGoError=11023
	SetModeTimeout=11024
	UnLoadFailed=11025
	NoFindUUT=11026
	PickFailed=11027
	LoadFailed=11028
	BinFailed=11029



class Robot(object):
	def __init__(self,**versionInfo):
		#command
		
		
		
		self.speed=100
		
		self.home_command="HOME"
		self.abort_command="ABORT"
		self.continue_command="CONTINUE"
		self.get_status_command="GET_STATUS"
		self.get_config_command="GET_CONFIG"
		self.set_config_command="SET_CONFIG"
		self.set_speed_command="SET_SPEED"
		self.get_speed_command="GET_SPEED"
		self.get_io_command="GET_IO"
		self.set_io_command="SET_IO"
		self.get_mode_command="GET_MODE"
		self.set_mode_command="SET_MODE"
		self.request_mode_change_command="REQUEST_MODE_CHANGE"
		self.report_mode_command="REPORT_MODE"
		self.unsolicited_command="UNSOLICITED"
		self.pick_command="PICK"
		self.bin_command="BIN"
		self.load_command="LOAD"
		self.unload_command="UNLOAD"
		self.pl_command="PL"
		self.ub_command="UB"
		self.pulb_command="PULB"
		self.ubpl_command="UBPL"
		self.uull_command="UULL"
		self.ul_command="UL"
		self.acquire_uut_map_command="ACQUIRE_UUT_MAP"
		self.beacon_command="BEACON"
		self.remove_uut_command="REMOVE_UUT"
		self.add_uut_command="ADD_UUT"
		
		#version info
		self.devType=versionInfo['devType']
		self.devIP=versionInfo['devIP']
		self.devOsVer=versionInfo['devosVer']
		self.devAppSw=versionInfo['devAppSw']
		self.devProtoVer=versionInfo['devProtoVer']
		self.robotSerialNum=versionInfo['robotSerialNum']
		
		
		#input io
		self.inbufers=Bin(2)
		self.audits=Bin(4)
		self.process=Bin(4)
		self.ngBin=Bin(4)
		self.air_pressure=Sensor('air',True)
		self.electric=Sensor('electric',True)
		self.estop=Sensor('estop',False)
		
		#output io
		self.light_tower=LightTower()
		
		#error code
		self.ESTOP_ERROR_CODE="01001"
		self.AIR_ERROR_CODE="01003"
		self.ELECTRIC_ERROR_CODE="01002"
		
		#device num
		self.reject_drawer_closed_num="20"
		self.golden_door_closed_num="21"
		self.input_num="51"
		self.output_num="52"
		self.inbuffer_num="21"
		self.ngbuffer_num="11"
		self.processbuffer_num="12"
		self.auditbuffer_num="31"
		self.left_num="101"
		self.right_num="102"
	
	def home(self,*mes):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def abort(self,*mes):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def continu(self,*mes):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def get_status(self,mes="00000",*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def get_config(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def get_speed(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def set_speed(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def unsolicited(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def beacon(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def set_config(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def get_mode(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def set_mode(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def request_mode_change(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	
	def report_mode(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def pick(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def bin(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def load(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def unload(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def pl(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def ub(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def ul(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def acquire_uut_map(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def pulb(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	def ubpl(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
		
	def uull(self,*args):
		return ROBOT_ERROR_CODE.UNOVERWRITE
	
	
	
	
	
	