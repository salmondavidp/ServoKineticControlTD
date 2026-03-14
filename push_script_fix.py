import json, urllib.request, os

# Read the modified script (relative to this script's location)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(SCRIPT_DIR, 'temp_script_fix.py'), 'r') as f:
    new_script = f.read()

# Escape for Python string embedding
escaped = new_script.replace('\\', '\\\\').replace("'", "\\'").replace('\n', '\\n').replace('\r', '')

push_script = "me.parent().op('script1_callbacks').text = '" + escaped + "'\nresult='Script updated'"

data = json.dumps({'script': push_script}).encode()
req = urllib.request.Request('http://localhost:9981/api/exec', data=data, headers={'Content-Type':'application/json'})
resp = urllib.request.urlopen(req)
print(resp.read().decode())
