#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 16 09:25:28 2022

@author: student
"""
from inputs import get_key


    

        
while 1:
    
    events = get_key()
    for event in events:
        print(event.ev_type, event.code, event.state)