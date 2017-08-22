#!/usr/bin/env python3
# -*- coding: utf-8 -*-



import logging
import os
from datetime import datetime

'''
LOG_DIR = os.path.join('log')  
if not os.path.exists(LOG_DIR):  
    os.makedirs(LOG_DIR) 
	
	file=open(datetime.now().date().isoformat()+'.log', 'a')
'''


fp=''
if not os.path.exists(datetime.now().date().isoformat()+'.log'):
	fp = open(datetime.now().date().isoformat()+'.log','a')
logging.basicConfig(level=logging.DEBUG,
        format='%(asctime)s %(filename)s[line:%(lineno)d] %(levelname)s %(message)s',
        datefmt='%a, %d %b %Y %H:%M:%S',
        filename=datetime.now().date().isoformat()+'.log',
        filemode='a')


console = logging.StreamHandler()
console.setLevel(logging.INFO)
formatter = logging.Formatter('%(name)-12s: %(levelname)-8s %(message)s')
console.setFormatter(formatter)
logger=logging.getLogger('testLog')
logger.addHandler(console)
	

#logger.info("1111")













		
		







