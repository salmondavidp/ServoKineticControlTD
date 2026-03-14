#!/usr/bin/env python3
"""Push the EtherCAT engine script to TouchDesigner's dna_controller"""
import urllib.request
import json
import base64
import os

API = "http://localhost:9981/api/exec"
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

with open(os.path.join(SCRIPT_DIR, "ethercat_engine.py"), "r") as f:
    script_content = f.read()

b64 = base64.b64encode(script_content.encode('utf-8')).decode('ascii')

push_script = "import base64\n"
push_script += "dc = me.parent()\n"
push_script += "ec_dat = dc.op('ethercat_engine')\n"
push_script += "if ec_dat is None:\n"
push_script += "    ec_dat = dc.create(textDAT, 'ethercat_engine')\n"
push_script += "code = base64.b64decode('" + b64 + "').decode('utf-8')\n"
push_script += "ec_dat.text = code\n"
push_script += "result = 'ethercat_engine created: ' + ec_dat.path"

data = json.dumps({"script": push_script}).encode('utf-8')
req = urllib.request.Request(API, data=data, headers={"Content-Type": "application/json"})
resp = urllib.request.urlopen(req)
print(json.loads(resp.read()))
