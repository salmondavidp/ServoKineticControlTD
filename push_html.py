import json, urllib.request, os

# Read the HTML file (relative to this script's location)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(SCRIPT_DIR, 'ui_timeline.html'), 'r', encoding='utf-8') as f:
    html = f.read()

# Escape for Python string
html_escaped = html.replace('\\', '\\\\').replace("'", "\\'").replace('\n', '\\n').replace('\r', '')

script = "me.parent().op('ui_html').text = '" + html_escaped + "'\nresult='HTML updated'"

data = json.dumps({'script': script}).encode()
req = urllib.request.Request('http://localhost:9981/api/exec', data=data, headers={'Content-Type':'application/json'})
resp = urllib.request.urlopen(req)
print(resp.read().decode())
