#!/usr/bin/env pybricks-micropython
import random, math
import pickle
import sys
import base64
from os import path

file_name = "saved_Q.pkl"

def send_pickle_as_text(file_path):
    with open(file_path, 'rb') as file:
        encoded = base64.b64encode(file.read()).decode('utf-8')
        sys.stdout.write(encoded)
        print(encoded)

send_pickle_as_text(file_name)