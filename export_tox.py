"""Export dna_controller as .tox file"""
import json, urllib.request, os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
tox_path = os.path.join(SCRIPT_DIR, 'dna_controller.tox').replace('\\', '/')

# Verify enabled check is present
verify_script = 'result = "enabled_check" if "enabled" in op("/project1/dna_controller/script1_callbacks").text else "NO_CHECK"'
data = json.dumps({'script': verify_script}).encode()
req = urllib.request.Request('http://localhost:9981/api/exec', data=data, headers={'Content-Type':'application/json'})
resp = urllib.request.urlopen(req)
print('Verify:', json.loads(resp.read().decode()))

# Export the .tox
export_script = f"op('/project1/dna_controller').save('{tox_path}')\nresult='Exported to {tox_path}'"
data = json.dumps({'script': export_script}).encode()
req = urllib.request.Request('http://localhost:9981/api/exec', data=data, headers={'Content-Type':'application/json'})
resp = urllib.request.urlopen(req)
print('Export:', json.loads(resp.read().decode()))
