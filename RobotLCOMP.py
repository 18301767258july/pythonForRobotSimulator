#!/usr/bin/env python3
# -*- coding: utf-8 -*-



from SocketTcpClient import TcpSocketClient
from Server import Server
from Robot import *
from Sensor import Sensor
from Bin import Bin
from datetime import datetime
import threading,time
import logging
import Log
import os,sys


class RobotLCOMP(Robot):
	def __init__(self):
		self.logger=logging.getLogger('testLog')
		self.ip='127.0.0.1'
		self.cam_port=49152
		self.avc_port=49154
		self.action_commands={'PICK':self.pick,'BIN':self.bin,'LOAD':self.load,'UNLOAD':self.unload,'PL':self.pl,'UB':self.ub,'UL':self.ul,'ACQUIRE_UUT_MAP':self.acquire_uut_map,'HOME':self.home}
		
		self.no_action_commands={'GET_CONFIG':self.get_config,'SET_CONFIG':self.set_config,'ABORT':self.abort,\
		'CONTINUE':self.continu,'GET_STATUS':self.get_status,'GET_SPEED':self.get_speed,'SET_SPEED':self.set_speed,\
		'UNSOLICITED':self.unsolicited,'BEACON':self.beacon,'GET_MODE':self.report_mode,'SET_MODE':self.report_mode}
		
		self.avc_commands={'REQUEST_CHANGE_DEVSTATE':self.request_change_dev_state,'CONNECT':self.connect,'DISCONNECT':self.disconnect,\
		'REMOVE_UUT':self.remove_uut,'ADD_UUT':self.add_uut,'MAKE_ERROR':self.make_error,'ESTOP':self.estop,'AIR':self.air,\
		'POWER':self.power,'RESET':self.reset}
		
		self.serverCAM=Server(self.ip,self.cam_port)
		self.serverAVC=Server(self.ip,self.avc_port)
		self.clientCAM=None
		self.clientAVC=None
		versionInfo={'devType':"SACV6",'devIP':self.ip,'devosVer':"RC8",'devAppSw':"v1.0.3",'devProtoVer':'0.32','robotSerialNum':'11R036'}
		super().__init__(**versionInfo)
		self.abort_flag=False
		self.continue_flag=False
		#clamp1
		self.clamp1_closed=Sensor('clamp1_closed',True)
		self.clamp1_opened=Sensor('clamp1_opened',True)
		self.clamp1_dut=Sensor('clamp1_dut',False)
		
		self.clamp2_closed=Sensor('clamp1_closed',True)
		self.clamp2_opened=Sensor('clamp1_opened',True)
		self.clamp2_dut=Sensor('clamp1_dut',False)
		#doors
		self.reject_drawer_closed=Sensor('reject_drawer_closed',True)
		self.reject_request_button_pressed=Sensor('reject_request_button_pressed',False)
		self.golden_door_closed=Sensor('golden_door_closed',True)
		self.golden_request_button_pressed=Sensor('golden_request_button_pressed',False)
		#output
		self.clamp1_close=Sensor('clamp1_close',True)
		self.clamp1_open=Sensor('clamp1_open',True)
		self.clamp2_close=Sensor('clamp1_close',True)
		self.clamp2_open=Sensor('clamp1_open',True)
		self.reject_drawer_lock=Sensor('reject_drawer_lock',True)
		self.golden_door_lock=Sensor('golden_door_lock',True)
		self.golden_door_light=Sensor('golden_door_light',False)
		self.current_position="Input"
		self.target_position=None
		#station 
		self.lefts=Bin(8)
		self.rights=Bin(8)
		
		self.pick_flag=False
		self.unload_flag=False
		self.time_out=15
		self.csv_data={}
		self.place_="place"
		self.pick_="pick"
		self.motion_data_list=[]
		self.locker=threading.Lock()
		self.motion_thread=threading.Thread(target=self.check_motion_data_list)
		self.motion_thread.setDaemon(True)
		
		self.motion_thread_flag=False
		
		#AVC
		self.drawer_flag=False
		self.golden_door_flag=False
		
		
		# load handling time file
		self.load_handling_time_file()
		
	def bool_to_num(self,data):
			if data==True:
				return 1
			else:
				return 0
	def get_input_bitmap(self):
		input_io=[]
		for sensor in self.inbufers.buffers:
			input_io.append(sensor.state)
		for i in range(0,2):
			input_io.append(False)
		for sensor in self.audits.buffers:
			input_io.append(sensor.state)
		for sensor in self.process.buffers:
			input_io.append(sensor.state)
		for sensor in self.ngBin.buffers:
			input_io.append(sensor.state)
		
		input_io.append(self.clamp1_closed.state)
		input_io.append(self.clamp1_opened.state)
		input_io.append(self.clamp1_dut.state)
		
		input_io.append(self.clamp2_closed.state)
		input_io.append(self.clamp2_opened.state)
		input_io.append(self.clamp2_dut.state)
		
		for i in range(0,2):
			input_io.append(False)
		input_io.append(self.reject_drawer_closed.state)
		input_io.append(self.reject_request_button_pressed.state)
		input_io.append(self.golden_door_closed.state)
		input_io.append(self.golden_request_button_pressed.state)
		
		input_io.reverse()
		
		str_io=list(map(str,list(map(self.bool_to_num,input_io))))
		
		result=''.join(str_io)
		x=eval('0b'+result)
		return hex(x)
		
	def get_output_bitmap(self):
		output_io=[]
		output_io.append(self.light_tower.get_red())
		output_io.append(self.light_tower.get_yellow())
		output_io.append(self.light_tower.get_green())
		output_io.append(self.light_tower.get_buzzer())
		
		for x in range(0,8):
			output_io.append(False)
		output_io.append(self.clamp1_close.state)
		output_io.append(self.clamp1_open.state)
		output_io.append(self.clamp2_close.state)
		output_io.append(self.clamp2_open.state)
		output_io.append(False)
		output_io.append(False)
		output_io.append(self.reject_drawer_lock.state)
		output_io.append(self.golden_door_lock.state)
		output_io.append(self.golden_door_light.state)
		
		output_io.reverse()
		
		str_io=''.join(list(map(str,list(map(self.bool_to_num,output_io)))))
		x=eval('0b'+str_io)
		return	hex(x)
		
	def request_change_dev_state(self,*args):
		start_time=datetime.now()
		time=0
		end_time=datetime.now()
		if args[1]==self.reject_drawer_closed_num:
			if args[2]=="1":
				self.reject_request_button_pressed.state=True
				self.get_status()
				self.request_mode_change(*(self.reject_drawer_closed_num,"1","0"))
			else:
				self.reject_request_button_pressed.state=False
				self.get_status()
				self.request_mode_change(*(self.reject_drawer_closed_num,"0","1"))
			while self.drawer_flag==False:
					time.sleep(0.002)
					end_time=datetime.now()
					time=(end_time-start_time).microsecond
					if time>=10000:
						break
			if self.drawer_flag:
				self.send_avc([args[0],"REQUEST_CHANGE_DEVSTATE",args[1],args[2],"0,Success"])
			else:
				self.send_avc([args[0],"REQUEST_CHANGE_DEVSTATE",args[1],args[2],"1,timeout"])
		elif args[1]==self.golden_door_closed_num:
			if args[2]=="1":
				self.golden_request_button_pressed.state=True
				self.get_status()
				self.request_mode_change(*(self.golden_door_closed_num,"1","0"))
			else:
				self.golden_request_button_pressed.state=False
				self.get_status()
				self.request_mode_change(*(self.golden_door_closed_num,"0","1"))
			while self.golden_door_flag==False:
					time.sleep(0.002)
					end_time=datetime.now()
					time=(end_time-start_time).microsecond
					if time>=10000:
						break
			if self.golden_door_flag:
				self.send_avc([args[0],"REQUEST_CHANGE_DEVSTATE",args[1],args[2],"0,Success"])
			else:
				self.send_avc([args[0],"REQUEST_CHANGE_DEVSTATE",args[1],args[2],"1,timeout"])
			
	
		
	def home(self,*mes):
		self.logger.info("IN HOME: ")
		self.continue_flag=False
		self.abort_flag=False
		dt=datetime.now().strftime('%m-%d-%Y %H:%M:%S')
		data=[mes[0],self.home_command,mes[2],dt,"0,success"]
		return data
		
	def abort(self,*mes):
		self.continue_flag=False
		self.abort_flag=True
		data=[mes[0],self.abort_command,"0,Success"]
		return data
		
	def continu(self,*mes):
		self.continue_flag=True
		self.abort_flag=False
		data=[mes[0],self.continue_command,"0,Success"]
		return data

	def get_status(self,mes="00000",*args):
		data=[]
		if len(args)==0:
			data=[mes,self.get_status_command,"Running",self.get_input_bitmap(),self.get_output_bitmap(),"0,Success"]
		else:
			data=[args[0],self.get_status_command,"Running",self.get_input_bitmap(),self.get_output_bitmap(),"0,Success"]
		if self.clientCAM!=None:
			try:
				self.send_data(data)
			except ConnectionAbortedError as e:
				print('ConnectionAbortedError ')
				self.clientCAM=None
				
		if self.clientAVC !=None:
			try:
				self.send_avc(data)
			except ConnectionAbortedError as e:
				print('ConnectionAbortedError')
				self.clientAVC=None
	def check_motion_data_list(self):
		data=''
		result_data=[]
		while True:
			time.sleep(0.02)
			if len(self.motion_data_list)>=1:
				self.logger.info("motion_data_list length: "+str(len(self.motion_data_list)))
				self.locker.acquire()
				data=self.motion_data_list[0]
				if data[1]=="PL":
					self.pl(*data)
				if data[1]=="UB":
					self.ub(*data)
				if data[1]=="UL":
					self.ul(*data)
				if data[1]=="PICK":
					result_data=self.pick(*data)[1]
				if data[1]=="BIN":
					result_data=self.bin(*data)[1]
				if data[1]=="LOAD":
					result_data=self.load(*data)[1]
				if data[1]=="UNLOAD":
					result_data=self.unload(*data)[1]
				if data[1]=="ACQUIRE_UUT_MAP":
					result_data=self.acquire_uut_map(*data)
				if data[1]=="HOME":
					result_data=self.home(*data)
				self.motion_data_list.pop(0)
				self.locker.release()
			if result_data!=[]:
				self.send_data(result_data)
				result_data=[]
			if self.motion_thread_flag==True:
				break
	
	def get_status_timer(self):
		self.get_status()
		global timer
		timer=threading.Timer(10,self.get_status_timer)
		timer.start()
	def get_config(self,*args):
		data=[args[0],self.get_config_command,self.devType,self.devIP,self.devosVer,self.devAppSw,self.devProtoVer,self.robotSerialNum,"0,Success"]
		return data
	def get_speed(self,*args):
		data=[args[0],self.get_speed_command,self.speed,"0,Success"]
		return data
	def set_speed(self,*args):
		self.speed=args[2]
		data=[args[0],self.set_speed_command,self.speed,self.speed,"0,Success"]
		return data
	def unsolicited(self,*args):
		data=["00",self.unsolicited_command]
		data_ex=[]
		if args[0] is self.estop:
			if args[1]==self.ESTOP_ERROR_CODE:
				data_ex=[args[1],"Robot ESTOP engaged"]
			else:
				data_ex=[args[1],"Robot ESTOP released"]
		if args[0] is self.air_pressure:
			if args[1]==self.AIR_ERROR_CODE:
				data_ex=[args[1],"Robot Air UnNormal"]
			else:
				data_ex=[args[1],"Robot Air Normal"]
		if args[0] is self.electric:
			if args[1]==self.ELECTRIC_ERROR_CODE:
				data_ex=[args[1],"Robot power UnNormal"]
			else:
				data_ex=[args[1]+",Robot power Normal"]
		data.extend(data_ex)
		return data
	def beacon(self,*args):
		data=[args[0],self.beacon_command]
		#buzzer
		if args[2]=="Off":
			self.light_tower.set_buzzer(False)
			data=data.append("Off")
		if args[2]=="On":
			self.light_tower.set_buzzer(True)
			data=data.append("On")
		if args[2]=="NoChange":
			sta=self.light_tower.get_buzzer()
			if sta==True:
				data=data.append("On")
			else:
				data=data.append("Off")
		#red
		if args[3]=="Off":
			self.light_tower.set_red(False)
			data=data.append("Off")
		if args[3]=="On":
			self.light_tower.set_red(True)
			data=data.append("On")
		if args[3]=="NoChange":
			sta=self.light_tower.get_red()
			if sta==True:
				data=data.append("On")
			else:
				data=data.append("Off")
		#yellow
		if args[4]=="Off":
			self.light_tower.set_yellow(False)
			data=data.append("Off")
		if args[4]=="On":
			self.light_tower.set_yellow(True)
			data=data.append("On")
		if args[4]=="NoChange":
			sta=self.light_tower.get_yellow()
			if sta==True:
				data=data.append("On")
			else:
				data=data.append("Off")
		if args[4]=="Flashing":
			self.light_tower.set_yellow(True)
			data=data.append("Flashing")
		#green
		if args[5]=="Off":
			self.light_tower.set_green(False)
			data=data.append("Off")
		if args[5]=="On":
			self.light_tower.set_green(True)
			data=data.append("On")
		if args[5]=="NoChange":
			sta=self.light_tower.get_green()
			if sta==True:
				data=data.append("On")
			else:
				data=data.append("Off")
		data=data.append("Off,0,0Success")
		self.get_status()
		return data
	def set_config(self,*args):
		pass
	
	def get_mode(self,*args):
		data=[args[0],self.report_mode_command]
		if args[2]==self.reject_drawer_closed_num:
			data=data.append(self.reject_drawer_closed_num).append(str(self.bool_to_num(self.reject_drawer_closed.state)))
		elif args[2]==self.golden_door_closed_num:
			data=data.append(self.golden_door_closed_num).append(str(self.bool_to_num(self.golden_door_closed.state)))
		data=data.append("0,Success")	
		return data
	def set_mode(self,*args):
		data=[args[0],self.report_mode_command]
		data_ext=[]
		if args[2]==self.reject_drawer_closed_num:#20
			if args[3]=="1":
				self.reject_drawer_lock.state=True
				self.reject_drawer_closed.state=True
				self.drawer_flag=True
				data_ext=[self.reject_drawer_closed_num,str(self.bool_to_num(self.reject_drawer_closed.state)),"0,Success"]
			else:
				if self.reject_drawer_closed.state==True:
					self.reject_drawer_lock.state=False
					self.reject_drawer_closed.state=False
					self.drawer_flag=False
					data_ext=[self.reject_drawer_closed_num,str(self.bool_to_num(self.reject_drawer_closed.state)),"0,Success"]
				else:
					data_ext=[self.reject_drawer_closed_num,str(self.bool_to_num(self.reject_drawer_closed.state)),str(ROBOT_ERROR_CODE.DoorNotClosed.value),ROBOT_ERROR_CODE.DoorNotClosed.name]
		elif args[2]==self.golden_door_closed_num:#21
			if args[3]=="1":
				self.golden_door_closed.state=True
				self.golden_door_lock.state=True
				self.golden_door_light.state=True
				self.golden_door_flag=True
				data_ext=[self.golden_door_closed_num,str(self.bool_to_num(self.golden_door_lock.state)),"0,Success"]
			else:
				if self.golden_door_closed.state==True:
					self.golden_door_closed.state=False
					self.golden_door_lock.state=False
					self.golden_door_light.state=False
					self.golden_door_flag=False
					data_ext=[self.golden_door_closed_num,str(self.bool_to_num(self.golden_door_lock.state)),"0,Success"]
				else:
					data_ext=[self.golden_door_closed_num,str(self.bool_to_num(self.golden_door_lock.state)),str(ROBOT_ERROR_CODE.DoorNotClosed.value),ROBOT_ERROR_CODE.DoorNotClosed.name]
		self.get_status()
		data.extend(data_ext)
		return data
		
	
	def request_mode_change(self,*args):
		data=["00",self.request_mode_change_command,args[0],args[1],args[2]]
		self.send_data(data)
	def report_mode(self,*args):
		data=[]
		if args[1]==self.get_mode_command:
			data=self.get_mode(*args)
		else:
			data=self.set_mode(*args)
		return data
	
	def pick(self,*args):
		flag=ROBOT_ERROR_CODE.UndefineError
		# arg is :00166,PL,01,51,01,
		self.logger.info(" pick args is :"+args[0]+","+args[1]+","+args[2]+","+args[3]+","+args[4])
		if args[3]==self.input_num:
			self.target_position="Input"
			self.delay_time_microseconds(self.csv_data[self.current_position+self.target_position+self.pick_],self.speed)
			flag=self.from_input(int(args[2]),int(args[4]))
			self.logger.info(" pick flag:"+flag.name)
		elif args[3]==self.inbuffer_num:
			flag=self.from_inbuffer(int(args[2]),int(args[4]))
		elif args[3]==self.auditbuffer_num:
			flag=self.from_audit(int(args[2]),int(args[4]))
		if flag==ROBOT_ERROR_CODE.SUCCESS:
			data=[args[0],self.pick_command,args[2],args[3],args[4],"0,Success"]
		else:
			data=[args[0],self.pick_command,args[2],args[3],args[4],"1",flag.name]
		return flag,data
	
	def bin(self,*args):
		flag=ROBOT_ERROR_CODE.UndefineError
		if args[3]==self.output_num:
			self.target_position="Output"
			self.delay_time_microseconds(self.csv_data[self.current_position+self.target_position+self.place_],self.speed)
			end_time=datetime.now()
			total_time=0
			start_time=datetime.now()
			if self.clamp1_dut.state==True:
				while True:
					time.sleep(0.002)
					end_time=datetime.now()
					time=(end_time-start_time).seconds
					if time>=self.time_out:
						flag=ROBOT_ERROR_CODE.MoveActionTimeout
						break
					if self.estop.state==True:
						flag=ROBOT_ERROR_CODE.EmergencyMode
						break
					if self.abort_flag==True:
						self.abort_flag=False
						flag=ROBOT_ERROR_CODE.TaskAbort
						break
					if self.continue_flag==True:
						break
				if self.continue_flag==True and time<self.time_out:
					self.place_uut_use_which_gripper(int(args[2]))
					flag=ROBOT_ERROR_CODE.SUCCESS
					self.continue_flag=False
		if args[3]==self.inbuffer_num:
			flag=self.to_inbuffer(int(args[2]),int(args[4]))
		if args[3]==self.ngbuffer_num:
			flag=self.to_ng(int(args[2]),int(args[4]))
		if args[3]==self.processbuffer_num:
			flag=self.to_process(int(args[2]),int(args[4]))
		if args[3]==self.auditbuffer_num:
			flag=self.to_audit(int(args[2]),int(args[4]))
		if flag==ROBOT_ERROR_CODE.SUCCESS:
			data=args[0]+self.bin_command+args[2]+","+args[3]+","+args[4]+",0,Success"
		else:
			data=args[0]+self.bin_command+args[2]+","+args[3]+","+args[4]+",1,"+flag.name
		return flag,data
		
	def load(self,*args):
		flag=ROBOT_ERROR_CODE.UndefineError
		if args[3]==self.left_num:
			if args[4]=="01" or args[4]=="03" or args[4]=="05" or args[4]=="07":
				self.target_position="L-comp_1"
			elif args[4]=="02" or args[4]=="04" or args[4]=="06" or args[4]=="08":
				self.target_position="L-comp_2"
			self.delay_time_microseconds(self.csv_data[self.current_position+self.target_position+self.place_],self.speed)
			flag=self.to_left(int(args[2]),int(args[4]))
		elif args[3]==self.right_num:
			if args[4]=="01" or args[4]=="03" or args[4]=="05" or args[4]=="07":
				self.target_position="L-comp_3"
			elif args[4]=="02" or args[4]=="04" or args[4]=="06" or args[4]=="08":
				self.target_position="L-comp_4"
			self.delay_time_microseconds(self.csv_data[self.current_position+self.target_position+self.place_],self.speed)
			flag=self.to_right(int(args[2]),int(args[4]))
		if flag==ROBOT_ERROR_CODE.SUCCESS:
			data=args[0]+self.load_command+args[2]+","+args[3]+","+args[4]+",0,Success"
		else:
			data=args[0]+self.load_command+args[2]+","+args[3]+","+args[4]+",1,"+flag.name
		return flag,data
	def unload(self,*args):
		flag=ROBOT_ERROR_CODE.UndefineError
		if args[3]==self.left_num:
			if args[4]=="01" or args[4]=="03" or args[4]=="05" or args[4]=="07":
				self.target_position="L-comp_1"
			elif args[4]=="02" or args[4]=="04" or args[4]=="06" or args[4]=="08":
				self.target_position="L-comp_2"
			self.delay_time_microseconds(self.csv_data[self.current_position+self.target_position+self.pick_],self.speed)
			flag=self.from_left(int(args[2]),int(args[4]))
		elif args[3]==self.right_num:
			if args[4]=="01" or args[4]=="03" or args[4]=="05" or args[4]=="07":
				self.target_position="L-comp_3"
			elif args[4]=="02" or args[4]=="04" or args[4]=="06" or args[4]=="08":
				self.target_position="L-comp_4"
			self.delay_time_microseconds(self.csv_data[self.current_position+self.target_position+self.pick_],self.speed)
			flag=self.from_right(int(args[2]),int(args[4]))
		if flag==ROBOT_ERROR_CODE.SUCCESS:
			data=args[0]+self.bin_command+args[2]+","+args[3]+","+args[4]+",0,Success"
		else:
			data=args[0]+self.bin_command+args[2]+","+args[3]+","+args[4]+",1,"+flag.name
		return flag,data
	
	def pulb(self,*args):
		flag=ROBOT_ERROR_CODE.UndefineError
		arg=args[0:5]
		result=[]
		flag,result=self.pick(*arg)
		self.send_data(result)
		if flag==ROBOT_ERROR_CODE.SUCCESS:
			arg=(args[0],args[1],args[5],args[6],args[7])
			flag,result=self.unload(*arg)
			self.send_data(result)
			if flag==ROBOT_ERROR_CODE.SUCCESS:
				arg=(args[0],args[1],args[8],args[9],args[10])
				flag,result=self.load(*arg)
				self.send_data(result)
				if flag==ROBOT_ERROR_CODE.SUCCESS:
					arg=(args[0],args[1],args[11],args[12],args[13])
					flag,result=self.bin(*arg)
					self.send_data(result)
				else:
					result=[args[0],self.bin_command,args[11],args[12],args[13],"1",flag.name]
					self.send_data(result)
			else:
				flag=ROBOT_ERROR_CODE.TaskAbort
				result=[args[0],self.load_command,args[8],args[9],args[10],"1",flag.name]
				self.send_data(result)
				result=[args[0],self.bin_command,args[11],args[12],args[13],"1",flag.name]
				self.send_data(result)
		else:
			flag=ROBOT_ERROR_CODE.TaskAbort
			result=[args[0],self.unload_command,args[5],args[6],args[7],"1",flag.name]
			self.send_data(result)
			result=[args[0],self.load_command,args[8],args[9],args[10],"1",flag.name]
			self.send_data(result)
			result=[args[0],self.bin_command,args[11],args[12],args[13],"1",flag.name]
			self.send_data(result)
	def ubpl(self,*args):
		flag=ROBOT_ERROR_CODE.UndefineError
		arg=args[0:5]
		result=[]
		flag,result=self.unload(*arg)
		self.send_data(result)
		if flag==ROBOT_ERROR_CODE.SUCCESS:
			arg=(args[0],args[1],args[5],args[6],args[7])
			flag,result=self.bin(*arg)
			self.send_data(result)
			if flag==ROBOT_ERROR_CODE.SUCCESS:
				arg=(args[0],args[1],args[8],args[9],args[10])
				flag,result=self.pick(*arg)
				self.send_data(result)
				if flag==ROBOT_ERROR_CODE.SUCCESS:
					arg=(args[0],args[1],args[11],args[12],args[13])
					flag,result=self.load(*arg)
					self.send_data(result)
				else:
					result=[args[0],self.load_command,args[11],args[12],args[13],"1",flag.name]
					self.send_data(result)
			else:
				flag=ROBOT_ERROR_CODE.TaskAbort
				result=[args[0],self.pick_command,args[8],args[9],args[10],"1",flag.name]
				self.send_data(result)
				result=[args[0],self.load_command,args[11],args[12],args[13],"1",flag.name]
				self.send_data(result)
		else:
			flag=ROBOT_ERROR_CODE.TaskAbort
			result=[args[0],self.bin_command,args[5],args[6],args[7],"1",flag.name]
			self.send_data(result)
			result=[args[0],self.pick_command,args[8],args[9],args[10],"1",flag.name]
			self.send_data(result)
			result=[args[0],self.load_command,args[11],args[12],args[13],"1",flag.name]
			self.send_data(result)
		
	def uull(self,*args):
		flag=ROBOT_ERROR_CODE.UndefineError
		arg=args[0:5]
		result=[]
		flag,result=self.unload(*arg)
		self.send_data(result)
		if flag==ROBOT_ERROR_CODE.SUCCESS:
			arg=(args[0],args[1],args[5],args[6],args[7])
			flag,result=self.unload(*arg)
			self.send_data(result)
			if flag==ROBOT_ERROR_CODE.SUCCESS:
				arg=(args[0],args[1],args[8],args[9],args[10])
				flag,result=self.load(*arg)
				self.send_data(result)
				if flag==ROBOT_ERROR_CODE.SUCCESS:
					arg=(args[0],args[1],args[11],args[12],args[13])
					flag,result=self.load(*arg)
					self.send_data(result)
				else:
					result=[args[0],self.load_command,args[11],args[12],args[13],"1",flag.name]
					self.send_data(result)
			else:
				flag=ROBOT_ERROR_CODE.TaskAbort
				result=[args[0],self.load_command,args[8],args[9],args[10],"1",flag.name]
				self.send_data(result)
				result=[args[0],self.load_command,args[11],args[12],args[13],"1",flag.name]
				self.send_data(result)
		else:
			flag=ROBOT_ERROR_CODE.TaskAbort
			result=[args[0],self.unload_command,args[5],args[6],args[7],"1",flag.name]
			self.send_data(result)
			result=[args[0],self.load_command,args[8],args[9],args[10],"1",flag.name]
			self.send_data(result)
			result=[args[0],self.load_command,args[11],args[12],args[13],"1",flag.name]
			self.send_data(result)
	
	def pl(self,*args):
		self.logger.info("start PL")
		flag=ROBOT_ERROR_CODE.UndefineError
		pick_args=args[0:5]
		flag,result=self.pick(*pick_args)
		self.send_data(result)
		data=[]
		arg=(args[0],args[1],args[5],args[6],args[7])
		if flag==ROBOT_ERROR_CODE.SUCCESS:
			flag,data=self.load(*arg)
		else:
			data=[args[0],self.load_command,args[5],args[6],args[7],"1",flag.name]
		self.send_data(data)
		return flag
	def ub(self,*args):
		flag=ROBOT_ERROR_CODE.UndefineError
		arg=args[0:5]
		flag,result=self.unload(*arg)
		self.send_data(result)
		data=[]
		arg=(args[0],args[1],args[5],args[6],args[7])
		if flag==ROBOT_ERROR_CODE.SUCCESS:
			flag,data=self.bin(*arg)
		else:
			data=[args[0],self.bin_command,args[5],args[6],args[7],"1",flag.name]
		self.send_data(data)
		return flag
		
	def ul(self,*args):
		flag=ROBOT_ERROR_CODE.UndefineError
		arg=args[0:5]
		flag,result=self.unload(*arg)
		self.send_data(result[1])
		data=[]
		arg=(args[0],args[1],args[5],args[6],args[7])
		if flag==ROBOT_ERROR_CODE.SUCCESS:
			flag,data=self.load(*arg)
		else:
			data=[args[0],self.load_command,args[5],args[6],args[7],"1",flag.name]
		self.send_data(data)
		return flag
	
	def acquire_uut_map(self,*args):
		data=[]
		self.logger.info("acquire_uut_map: ")
		def have_dut():
			self.clamp1_dut.state=True
			self.get_status()
			self.clamp1_open.state=True
			self.clamp1_opened.state=True
			self.get_status()
			self.clamp1_dut.state=False
			self.get_status()
		def no_dut():
			self.clamp1_dut.state=False
			self.get_status()
			self.clamp1_open.state=True
			self.clamp1_opened.state=True
			self.get_status()
			self.clamp1_dut.state=False
			self.get_status()
		if self.clamp1_open.state==True and self.clamp1_opened.state==True:
			self.clamp1_close.state=True
			self.clamp1_closed.state=True
			self.get_status()
		if args[2]==self.left_num:
			if self.lefts.buffers[int(args[3])-1].state==True:
				have_dut()
				data=[args[0],self.acquire_uut_map_command,args[2],args[3],"1,0,Success"]
			else:
				no_dut()
				data=args[0],self.acquire_uut_map_command,args[2],args[3],"0,0,Success"
		elif args[2]==self.right_num:
			if self.rights.buffers[int(args[3])-1].state==True:
				have_dut()
				data=[args[0],self.acquire_uut_map_command,args[2],args[3],"1,0,Success"]
			else:
				no_dut()
				data=[args[0],self.acquire_uut_map_command,args[2],args[3],"0,0,Success"]
		return data
	def clamp1_ready_take(self):
		self.clamp1_opened.state=False
		self.clamp1_open.state=False
		self.get_status()
		self.clamp1_close.state=True
		self.clamp1_closed.state=True
		self.get_status()
	
	def clamp2_ready_take(self):
		self.clamp2_opened.state=False
		self.clamp2_open.state=False
		self.get_status()
		self.clamp2_close.state=True
		self.clamp2_closed.state=True
		self.get_status()
	def take_over_clamp1(self):
		self.clamp1_close.state=False
		self.clamp1_closed.state=False
		self.clamp1_opened.state=True
		self.clamp1_open.state=True
		self.clamp1_dut.state=False
		self.get_status()
		time.sleep(0.8)
		
	def take_over_clamp2(self):
		self.clamp2_close.state=False
		self.clamp2_closed.state=False
		self.clamp2_opened.state=True
		self.clamp2_open.state=True
		self.clamp2_dut.state=False
		self.get_status()
		time.sleep(0.8)
	def take_uut_use_which_gripper(self,gripper):
		if gripper==1:
			self.clamp1_ready_take()
		elif gripper==2:
			self.clamp2_ready_take()
	def place_uut_use_which_gripper(self,gripper):
		if gripper==1:
			self.take_over_clamp1()
		elif gripper==2:
			self.take_over_clamp2()
	def delay_time_microseconds(self,times,speed):
		if speed!=100:
			end_time=datetime.now()
			total_time=0
			start_time=datetime.now()
			while True:
				time.sleep(0.002)
				end_time=datetime.now()
				total_time=(end_time-start_time).microseconds
				if total_time>=times:
					break
			self.current_position=self.target_position
			
	def from_inbuffer(self,gripper,slot):
		flag=ROBOT_ERROR_CODE.UndefineError
		end_time=datetime.now()
		total_time=0
		start_time=datetime.now()
		if gripper==1:
			if self.clamp1_opened.state==True and self.clamp1_open.state==True:
				self.take_uut_use_which_gripper(gripper)
				if self.inbufers.buffers[slot-1].state==True:
					if self.pick_flag:
						flag=ROBOT_ERROR_CODE.PickFailed
					else:
						self.inbufers.buffers[slot-1].state==False
						self.clamp1_dut.state=True
						self.get_status()
						flag=ROBOT_ERROR_CODE.SUCCESS
				else:
					flag=ROBOT_ERROR_CODE.NoFindUUT
			else:
				if self.clamp1_dut.state==True:
					flag=ROBOT_ERROR_CODE.UnitAlreadyExist
		else:
			if self.clamp2_opened.state==True and self.clamp2_open.state==True:
				self.take_uut_use_which_gripper(gripper)
				if self.inbufers.buffers[slot-1].state==True:
					if self.pick_flag:
						flag=ROBOT_ERROR_CODE.PickFailed
					else:
						self.inbufers.buffers[slot-1].state==False
						self.clamp2_dut.state=True
						self.get_status()
						flag=ROBOT_ERROR_CODE.SUCCESS
				else:
					flag=ROBOT_ERROR_CODE.NoFindUUT
			else:
				if self.clamp2_dut.state==True:
					flag=ROBOT_ERROR_CODE.UnitAlreadyExist
		end_time=datetime.now()	
		time=(end_time-start_time).seconds
		if time>=self.time_out:
			flag=ROBOT_ERROR_CODE.MoveActionTimeout
				
		return flag	
	

	def from_input(self,gripper,slot):
		self.logger.info("start take uut from input")
		self.logger.info("gripper: "+str(gripper))
		flag=ROBOT_ERROR_CODE.UndefineError
		end_time=datetime.now()
		total_time=0
		start_time=datetime.now()
		if gripper==1:
			if self.clamp1_opened.state==True and self.clamp1_open.state==True:
				self.logger.info("clamp1 state is "+str(self.clamp1_opened.state))
				self.take_uut_use_which_gripper(gripper)
				self.clamp1_dut.state=True
				self.get_status()
				if self.pick_flag:
					flag=ROBOT_ERROR_CODE.PickFailed
					self.clamp1_dut.state=False
					self.get_status()
				else:
					flag=ROBOT_ERROR_CODE.SUCCESS
			else:
				if self.clamp1_dut.state==True:
					flag=ROBOT_ERROR_CODE.UnitAlreadyExist
		else:
			if self.clamp2_opened==True and self.clamp2_open==True:
				self.take_uut_use_which_gripper(gripper)
				self.clamp2_dut.state=True
				self.get_status()
				if self.pick_flag:
					flag=ROBOT_ERROR_CODE.PickFailed
					self.clamp2_dut.state=False
					self.get_status()
				else:
					flag=ROBOT_ERROR_CODE.SUCCESS
			else:
				if self.clamp2_dut.state==True:
					flag=ROBOT_ERROR_CODE.UnitAlreadyExist
		end_time=datetime.now()
		time=(end_time-start_time).seconds
		if time>=self.time_out:
			flag=ROBOT_ERROR_CODE.MoveActionTimeout
		return flag
	def from_left(self,gripper,slot):
		flag=ROBOT_ERROR_CODE.UndefineError
		end_time=datetime.now()
		total_time=0
		start_time=datetime.now()
		if gripper==1:
			if self.clamp1_opened==True and self.clamp1_open==True:
				self.take_uut_use_which_gripper(gripper)
				if self.unload_flag:
					self.lefts.buffers[slot-1].state=True
					flag=ROBOT_ERROR_CODE.UnLoadFailed
				else:
					self.clamp1_dut.state=True
					self.get_status()
					self.lefts.buffers[slot-1].state=False
					flag=ROBOT_ERROR_CODE.SUCCESS
			else:
				if self.clamp1_dut.state==True:
					flag=ROBOT_ERROR_CODE.UnitAlreadyExist
		else:
			if self.clamp2_opened==True and self.clamp2_open==True:
				self.take_uut_use_which_gripper(gripper)
				if self.unload_flag:
					self.lefts.buffers[slot-1].state=True
					flag=ROBOT_ERROR_CODE.UnLoadFailed
				else:
					self.clamp2_dut.state=True
					self.get_status()
					self.lefts.buffers[slot-1].state=False
					flag=ROBOT_ERROR_CODE.SUCCESS
			else:
				if self.clamp2_dut.state==True:
					flag=ROBOT_ERROR_CODE.UnitAlreadyExist
		end_time=datetime.now()
		time=(end_time-start_time).seconds
		if time>=self.time_out:
			flag=ROBOT_ERROR_CODE.MoveActionTimeout
		return flag
	
	def from_right(self,gripper,slot):
		flag=ROBOT_ERROR_CODE.UndefineError
		end_time=datetime.now()
		total_time=0
		start_time=datetime.now()
		if gripper==1:
			if self.clamp1_opened==True and self.clamp1_open==True:
				self.take_uut_use_which_gripper(gripper)
				if self.unload_flag:
					self.rights.buffers[slot-1].state=True
					flag=ROBOT_ERROR_CODE.UnLoadFailed
				else:
					self.clamp1_dut.state=True
					self.get_status()
					self.rights.buffers[slot-1].state=False
					flag=ROBOT_ERROR_CODE.SUCCESS
			else:
				if self.clamp1_dut.state==True:
					flag=ROBOT_ERROR_CODE.UnitAlreadyExist
		else:
			if self.clamp2_opened==True and self.clamp2_open==True:
				self.take_uut_use_which_gripper(gripper)
				if self.unload_flag:
					self.rights.buffers[slot-1].state=True
					flag=ROBOT_ERROR_CODE.UnLoadFailed
				else:
					self.clamp2_dut.state=True
					self.get_status()
					self.rights.buffers[slot-1].state=False
					flag=ROBOT_ERROR_CODE.SUCCESS
			else:
				if self.clamp2_dut.state==True:
					flag=ROBOT_ERROR_CODE.UnitAlreadyExist
		end_time=datetime.now()
		time=(end_time-start_time).seconds
		if time>=self.time_out:
			flag=ROBOT_ERROR_CODE.MoveActionTimeout
		return flag
	def from_audit(self,gripper,slot):
		flag=ROBOT_ERROR_CODE.UndefineError
		end_time=datetime.now()
		total_time=0
		start_time=datetime.now()
		if gripper==1:
			if self.clamp1_opened==True and self.clamp1_open==True:
				self.take_uut_use_which_gripper(gripper)
				if self.audits.buffers[slot-1].state:
					if self.pick_flag:
						flag=ROBOT_ERROR_CODE.PickFailed
					else:
						self.audits.buffers[slot-1].state=False
						self.clamp1_dut.state=True
						self.get_status()
						flag=ROBOT_ERROR_CODE.SUCCESS
				else:
					flag=ROBOT_ERROR_CODE.NoFindUUT
			else:
				if self.clamp1_dut.state==True:
					flag=ROBOT_ERROR_CODE.UnitAlreadyExist
		else:
			if self.clamp2_opened==True and self.clamp2_open==True:
				self.take_uut_use_which_gripper(gripper)
				if self.audits.buffers[slot-1].state:
					if self.pick_flag:
						flag=ROBOT_ERROR_CODE.PickFailed
					else:
						self.audits.buffers[slot-1].state=False
						self.clamp2_dut.state=True
						self.get_status()
						flag=ROBOT_ERROR_CODE.SUCCESS
				else:
					flag=ROBOT_ERROR_CODE.NoFindUUT
			else:
				if self.clamp2_dut.state==True:
					flag=ROBOT_ERROR_CODE.UnitAlreadyExist
		end_time=datetime.now()
		time=(end_time-start_time).seconds
		if time>=self.time_out:
			flag=ROBOT_ERROR_CODE.MoveActionTimeout
		return flag
	def to_left(self,gripper,slot):
		flag=ROBOT_ERROR_CODE.UndefineError
		end_time=datetime.now()
		total_time=0
		start_time=datetime.now()
		if gripper==1:
			if self.clamp1_dut.state==True:
				self.place_uut_use_which_gripper(gripper)
				self.rights.buffers[slot-1].state=True
				flag=ROBOT_ERROR_CODE.SUCCESS
			else:
				flag=ROBOT_ERROR_CODE.NoUnitDetect
		else:
			if self.clamp2_dut.state==True:
				self.place_uut_use_which_gripper(gripper)
				self.rights.buffers[slot-1].state=True
				flag=ROBOT_ERROR_CODE.SUCCESS
			else:
				flag=ROBOT_ERROR_CODE.NoUnitDetect
		end_time=datetime.now()
		time=(end_time-start_time).seconds
		if time>=self.time_out:
			flag=ROBOT_ERROR_CODE.MoveActionTimeout
		return flag
	def to_right(self,gripper,slot):
		flag=ROBOT_ERROR_CODE.UndefineError
		end_time=datetime.now()
		total_time=0
		start_time=datetime.now()
		if gripper==1:
			if self.clamp1_dut.state==True:
				self.place_uut_use_which_gripper(gripper)
				self.lefts.buffers[slot-1].state=True
				flag=ROBOT_ERROR_CODE.SUCCESS
			else:
				flag=ROBOT_ERROR_CODE.NoUnitDetect
		else:
			if self.clamp2_dut.state==True:
				self.place_uut_use_which_gripper(gripper)
				self.lefts.buffers[slot-1].state=True
				flag=ROBOT_ERROR_CODE.SUCCESS
			else:
				flag=ROBOT_ERROR_CODE.NoUnitDetect
		end_time=datetime.now()
		time=(end_time-start_time).seconds
		if time>=self.time_out:
			flag=ROBOT_ERROR_CODE.MoveActionTimeout
		return flag
	def to_audit(self,gripper,slot):
		flag=ROBOT_ERROR_CODE.UndefineError
		end_time=datetime.now()
		total_time=0
		start_time=datetime.now()
		if gripper==1:
			if self.clamp1_dut.state==True:
				if self.audits.buffers[slot-1].state:
					flag=ROBOT_ERROR_CODE.PositionIsUsed
				else:
					self.place_uut_use_which_gripper(gripper)
					self.audits.buffers[slot-1].state=True
					self.get_status()
					if self.audits.buffers[slot-1].state==False:
						flag=ROBOT_ERROR_CODE.NoDetectSignal
					else:
						flag=ROBOT_ERROR_CODE.SUCCESS
		else:
			if self.clamp2_dut.state==True:
				if self.audits.buffers[slot-1].state:
					flag=ROBOT_ERROR_CODE.PositionIsUsed
				else:
					self.place_uut_use_which_gripper(gripper)
					self.audits.buffers[slot-1].state=True
					self.get_status()
					if self.audits.buffers[slot-1].state==False:
						flag=ROBOT_ERROR_CODE.NoDetectSignal
					else:
						flag=ROBOT_ERROR_CODE.SUCCESS
		end_time=datetime.now()
		time=(end_time-start_time).seconds
		if time>=self.time_out:
			flag=ROBOT_ERROR_CODE.MoveActionTimeout
		return flag
	
	def to_inbuffer(self,gripper,slot):
		flag=ROBOT_ERROR_CODE.UndefineError
		end_time=datetime.now()
		total_time=0
		start_time=datetime.now()
		if gripper==1:
			if self.clamp1_dut.state==True:
				if self.inbufers.buffers[slot-1].state:
					flag=ROBOT_ERROR_CODE.PositionIsUsed
				else:
					self.place_uut_use_which_gripper(gripper)
					self.inbufers.buffers[slot-1].state=True
					self.get_status()
					if self.inbufers.buffers[slot-1].state==False:
						flag=ROBOT_ERROR_CODE.NoDetectSignal
					else:
						flag=ROBOT_ERROR_CODE.SUCCESS
		else:
			if self.clamp2_dut.state==True:
				if self.inbufers.buffers[slot-1].state:
					flag=ROBOT_ERROR_CODE.PositionIsUsed
				else:
					self.place_uut_use_which_gripper(gripper)
					self.inbufers.buffers[slot-1].state=True
					self.get_status()
					if self.inbufers.buffers[slot-1].state==False:
						flag=ROBOT_ERROR_CODE.NoDetectSignal
					else:
						flag=ROBOT_ERROR_CODE.SUCCESS
		end_time=datetime.now()
		time=(end_time-start_time).seconds
		if time>=self.time_out:
			flag=ROBOT_ERROR_CODE.MoveActionTimeout
		return flag
	def to_process(self,gripper,slot):
		flag=ROBOT_ERROR_CODE.UndefineError
		end_time=datetime.now()
		total_time=0
		start_time=datetime.now()
		if gripper==1:
			if self.clamp1_dut.state==True:
				if self.process.buffers[slot-1].state:
					flag=ROBOT_ERROR_CODE.PositionIsUsed
				else:
					self.place_uut_use_which_gripper(gripper)
					self.process.buffers[slot-1].state=True
					self.get_status()
					if self.process.buffers[slot-1].state==False:
						flag=ROBOT_ERROR_CODE.NoDetectSignal
					else:
						flag=ROBOT_ERROR_CODE.SUCCESS
		else:
			if self.clamp2_dut.state==True:
				if self.process.buffers[slot-1].state:
					flag=ROBOT_ERROR_CODE.PositionIsUsed
				else:
					self.place_uut_use_which_gripper(gripper)
					self.process.buffers[slot-1].state=True
					self.get_status()
					if self.process.buffers[slot-1].state==False:
						flag=ROBOT_ERROR_CODE.NoDetectSignal
					else:
						flag=ROBOT_ERROR_CODE.SUCCESS
		end_time=datetime.now()
		time=(end_time-start_time).seconds
		if time>=self.time_out:
			flag=ROBOT_ERROR_CODE.MoveActionTimeout
		return flag
	def to_ng(self,gripper,slot):
		flag=ROBOT_ERROR_CODE.UndefineError
		end_time=datetime.now()
		total_time=0
		start_time=datetime.now()
		if gripper==1:
			if self.clamp1_dut.state==True:
				if self.ngBin.buffers[slot-1].state:
					flag=ROBOT_ERROR_CODE.PositionIsUsed
				else:
					self.place_uut_use_which_gripper(gripper)
					self.ngBin.buffers[slot-1].state=True
					self.get_status()
					if self.ngBin.buffers[slot-1].state==False:
						flag=ROBOT_ERROR_CODE.NoDetectSignal
					else:
						flag=ROBOT_ERROR_CODE.SUCCESS
		else:
			if self.clamp2_dut.state==True:
				if self.ngBin.buffers[slot-1].state:
					flag=ROBOT_ERROR_CODE.PositionIsUsed
				else:
					self.place_uut_use_which_gripper(gripper)
					self.ngBin.buffers[slot-1].state=True
					self.get_status()
					if self.ngBin.buffers[slot-1].state==False:
						flag=ROBOT_ERROR_CODE.NoDetectSignal
					else:
						flag=ROBOT_ERROR_CODE.SUCCESS
		end_time=datetime.now()
		time=(end_time-start_time).seconds
		if time>=self.time_out:
			flag=ROBOT_ERROR_CODE.MoveActionTimeout
		return flag
		
	
	def load_handling_time_file(self):
		path=os.getcwd()
		file=os.path.join(path,"testCSV.csv")
		f=open(file,'r')
		line=f.readline()
		while line!="" and line!=None:
			time.sleep(0.2)
			data=line.replace('\n','')
			args=data.split(',')
			if args[0]=='' or args[0]==None:
				break
			if len(args)>=3:
				self.csv_data[args[0]+args[1]+args[2]]=int(args[3])
			line=f.readline()
		f.close()
			
	
	def handleReceive_data_cam(self):
		while True:
			if self.clientCAM!=None:
				data=self.clientCAM.receive()
				print(data)
				time.sleep(0.5)
				if data!=None:
					if len(data)>0 and data!="00":
						self.logger.info("<-----:CAM"+data)
						data_re=data.replace('\r','')
						change_formate=data_re.split(',')
						self.logger.info("receive:"+data)
						self.logger.info("COMMAND:"+change_formate[1])
						continue_flag=False
						if change_formate[1] in self.action_commands.keys():
							self.motion_data_list.append(change_formate)
							self.logger.info("motion list:"+change_formate[1])
							if not self.motion_thread.is_alive():
								self.logger.info("motion_thread")
								self.motion_thread.start()
						elif change_formate[1] in self.no_action_commands.keys():
							self.logger.info("no motion list:"+change_formate[1])
							func=self.no_action_commands.get(change_formate[1])
							dataSend=func(*change_formate)
							self.send_data(dataSend)
							self.logger.info("send:"+dataSend)
						else:
							self.send_data("NO THIS COMMAND")
					elif data=="00":
						self.clientCAM=None
						break
				else:
					self.clientCAM=None
					break
			else:
				break
	def handleReceive_data_avc(self):
		while True:
			if self.clientAVC!=None:
				data=self.clientAVC.receive()
				time.sleep(0.5)
				if data!=None:
					if len(data)>0 and data!="00":
					
						self.logger.info("receive:"+data)
						change_formate=data.split(',')
						func=self.avc_commands.get(change_formate[1])
						#tup=(change_formate[0],change_formate[2])
						if func!=None:
							func(*change_formate)
							self.logger.info("send:"+dataSend)
						else:
							self.send_avc("no this command to AVC")
					elif data=="00":
						self.clientAVC=None
						timer.cancel()
						break
				else:
					self.clientAVC=None
					break
			else:
				break
	def listen_CAM(self):
		while True:
			self.serverCAM.listen()
			time.sleep(1)
			if self.serverCAM.client !=None and self.clientCAM==None:
				self.clientCAM=TcpSocketClient(self.serverCAM.client)
				self.logger.info("connect CAM success")
				timer=threading.Timer(1,self.get_status_timer)
				timer.start()
				self.handleReceive_data_cam()
				
	def listen_AVC(self):
		while True:
			time.sleep(1)
			self.serverAVC.listen()
			if self.serverAVC.client!=None and self.clientAVC==None:
				self.clientAVC=TcpSocketClient(self.serverAVC.client)
				self.logger.info("connect AVC success")
				timer=threading.Timer(1,self.get_status_timer)
				timer.start()
				self.handleReceive_data_avc()			
			
	def connect(self,mesId):
		flag=False
		data=[]
		if self.clientCAM==None and self.serverCAM==None:
			self.serverCAM=Server("127.0.0.1",49152)
			self.listen_CAM()
			time.sleep(3)
			start_time=datetime.now()
			end_time=datetime.now()
			times=0
			while self.clientCAM==None:
				time.sleep(0.002)
				end_time=datetime.now()
				times=(end_time-start_time).seconds
				if times>=600:
					flag=True
					data=[mesId,"CONNECT,1,connect timeout."]
					break
			if self.clientCAM!=None:
				flag=True
				timer=threading.Timer(1,self.get_status_timer)
				timer.start()
				data=[mesId,"CONNECT,0,SUCCESS."]
		else:
			data=[mesId,"CONNECT,1,please check connect status."]
		self.send_avc(data)
		return flag
	
	def disconnect(self,mesId):
		data=[]
		flag=False
		if self.clientCAM!=None:
			timer.clancel()
			self.stop_listen_CAM()
			if self.clientCAM!=None:
				self.clientCAM.close()
			self.clientCAM=None
			self.serverCAM=None
			flag=True
			data=[mesId,"DISCONNECT,0,SUCCESS."]
		else:
			data=[mesId,"DISCONNECT,1,please check connect status."]
		self.send_avc(data)
		return flag
	
	def remove_uut(self,*args):
		flag=False
		data=[]
		if args[2]==self.left_num:
			self.lefts.buffers(int(args[3])-1).state=False
			flag=True
		if args[2]==self.right_num:
			self.rights.buffers(int(args[3])-1).state=False
			flag=True
		if args[2]==self.inbuffer_num:
			self.inbufers.buffers(int(args[3])-1).state=False
			flag=True
		if self.auditbuffer_num:
			self.audits.buffers(int(args[3])-1).state=False
			flag=True
		if self.ngbuffer_num:
			self.ngBin.buffers(int(args[3])-1).state=False
			flag=True
		if self.processbuffer_num:
			self.process.buffers(int(args[3])-1).state=False
			flag=True
		self.get_status()
		data=[args[0],self.remove_uut_command,args[2],args[3],"0,Success"]
		self.send_avc(data)
		return flag
	
	def add_uut(self,*args):
		flag=False
		data=[]
		if args[2]==self.left_num:
			self.lefts.buffers(int(args[3])-1).state=True
			flag=True
		if args[2]==self.right_num:
			self.rights.buffers(int(args[3])-1).state=True
			flag=True
		if args[2]==self.inbuffer_num:
			self.inbufers.buffers(int(args[3])-1).state=True
			flag=True
		if self.auditbuffer_num:
			self.audits.buffers(int(args[3])-1).state=True
			flag=True
		if self.ngbuffer_num:
			self.ngBin.buffers(int(args[3])-1).state=True
			flag=True
		if self.processbuffer_num:
			self.process.buffers(int(args[3])-1).state=True
			flag=True
		self.get_status()
		data=[args[0],self.add_uut_command,args[2],args[3],"0,Success"]
		self.send_avc(data)
		return flag
	
	def make_error(self,*args):
		flag=False
		data=[]
		if args[2]=="PICK":
			self.pick_flag=True
			flag=True
		if args[2]=="UNLOAD":
			self.unload_flag=True
			flag=True
		data=[args[0],args[1],args[2],args[3],args[4],"0,Success"]
		self.send_avc(data)
		return flag
	def estop(self,*args):
		data=[]
		flag=False
		if args[1]=="1":
			self.estop.state=True
			self.unsolicited(self.estop,self.ESTOP_ERROR_CODE)
			flag=True
		else:
			self.estop.state=False
			self.unsolicited(self.estop,"0")
			flag=True
		self.get_status()
		data=[args[0],"ESTOP",args[1],"0,Success"]
		self.send_avc(data)
		return flag
	def air(self,*args):
		flag=False
		data=[]
		if args[1]=="1":
			self.air_pressure.state=False
			self.unsolicited(self.air_pressure,self.AIR_ERROR_CODE)
			flag=True
		else:
			self.air_pressure.state=True
			self.unsolicited(self.air_pressure,"0")
			flag=True
		self.get_status()
		data=[args[0],"AIR",args[1],"0,Success"
		self.send_avc(data)
		return flag
	
	def power(self,*args):
		flag=False
		data=[]
		if args[1]=="1":
			self.electric.state=False
			self.unsolicited(self.electric,self.ELECTRIC_ERROR_CODE)
			flag=True
		else:
			self.electric.state=True
			self.unsolicited(self.electric,"0")
			flag=True
		self.get_status()
		data=[args[0],"POWER",args[1],"0,Success"]
		self.send_avc(data)
		return flag
	
	def reset(self,mesId):
		data=[]
		self.pick_flag=False
		self.unload_flag=False
		if self.clientCAM==None:
			self.listen_CAM()
			timer=threading.Timer(1,self.get_status_timer)
			timer.start()
		self.estop.state=False
		self.air_pressure.state=True
		self.electric.state=True
		selg.get_status()
		
		data=[mesId,"RESET,0,Success"]
		self.send_avc(data)
	
	
	
	def send_data(self,data):
		if self.clientCAM!=None:
			if isinstance(data,list)==True:
				self.clientCAM.send(','.join(data))
				self.logger.info("-----> CAM:"+','.join(data))
			else:
				self.clientCAM.send(data)
				self.logger.info("-----> CAM:"+data)
	def send_avc(self,data):
		if self.clientAVC!=None:
			if isinstance(data,list)==True:
				self.clientAVC.send(','.join(data))
				self.logger.info("-----> AVC:"+','.join(data))
			else:
				self.clientAVC.send(data)
				self.logger.info("-----> AVC:"+data)
	def stop_listen_CAM(self):
		if self.motion_thread.isAlive():
			self.motion_thread_flag=True
		self.serverCAM.stop_listen()
		self.serverCAM=None
if __name__ == '__main__':
	robot=RobotLCOMP()
	
	thCAM=threading.Thread(target=robot.listen_CAM)
	thAVC=threading.Thread(target=robot.listen_AVC)
	thCAM.setDaemon(False)
	thCAM.start()
	thAVC.setDaemon(False)
	thAVC.start()
	
	
	
	


	































