# MCP Server for TouchDesigner — Complete Setup Guide

A REST API server that runs **inside TouchDesigner**, letting AI tools (Claude Code, Cursor, etc.) and external scripts read, create, modify, and debug your TouchDesigner project in real-time.

---

## Table of Contents
- [What You Need](#what-you-need)
- [Setup Steps](#setup-steps)
- [Connecting Claude Code](#connecting-claude-code)
- [Connecting Cursor](#connecting-cursor)
- [API Reference](#api-reference)
- [Example Usage](#example-usage)
- [Architecture](#architecture)
- [Troubleshooting](#troubleshooting)

---

## What You Need

| Requirement | Details |
|---|---|
| TouchDesigner | 2023.11000+ (Non-Commercial or higher) |
| Python | Comes bundled with TD (3.11+) |
| PyYAML | `pip install pyyaml` (for OpenAPI schema loading) |
| Claude Code / Cursor | Any version — free tier works for MCP connection |
| Network | Localhost only (no internet needed for the MCP connection) |

> **Note:** The MCP server itself is free — it's just a WebServer DAT inside TouchDesigner. You do NOT need a paid AI subscription for the server to work. The AI tool you connect to it is separate.

---

## Setup Steps

### Step 1: Get the MCP WebServer Base

The MCP server is packaged as a `.tox` component file: `mcp_webserver_base.tox`

**Option A — From this repo:**
Download `mcp_webserver_base.tox` from the root of this repository.

**Option B — Build from source:**
The full source is in the `modules/` folder. You can import the modules manually (see Architecture section below).

### Step 2: Import into TouchDesigner

1. Open your TouchDesigner project
2. Drag and drop `mcp_webserver_base.tox` into your project, or right-click in the Network Editor > **Import Component (.tox)** > select `mcp_webserver_base.tox`
3. The base will appear with a WebServer DAT inside it

### Step 3: Set the External TOX Path

1. Select the imported base component
2. In the Parameters panel, set `External Tox` to point to where `mcp_webserver_base.tox` lives on disk
3. This is needed so the `import_modules.py` script can find the `modules/` folder next to it

### Step 4: Folder Structure

Make sure the folder structure next to your `.tox` file looks like this:

```
your_project_folder/
├── mcp_webserver_base.tox
├── import_modules.py
└── modules/
    ├── mcp_webserver_script.py      <- WebServer DAT callback script
    ├── mcp/
    │   ├── __init__.py              <- Loads OpenAPI schema
    │   ├── controllers/
    │   │   ├── __init__.py
    │   │   ├── api_controller.py    <- Main controller (routes requests)
    │   │   ├── generated_handlers.py <- Auto-generated endpoint handlers
    │   │   └── openapi_router.py    <- Matches URLs to handlers
    │   └── services/
    │       ├── __init__.py
    │       └── api_service.py       <- Talks to TouchDesigner (td.op, etc.)
    ├── td_server/
    │   └── openapi_server/
    │       └── openapi/
    │           ├── openapi.json      <- API schema (endpoints definition)
    │           └── openapi.yaml      <- Same schema in YAML format
    └── utils/
        ├── config.py
        ├── error_handling.py
        ├── logging.py
        ├── result.py
        ├── serialization.py
        ├── types.py
        └── version.py
```

### Step 5: Verify the Server is Running

Open a terminal and run:

```bash
curl http://localhost:9981/api/info
```

You should get back something like:

```json
{
  "success": true,
  "data": {
    "server": "TouchDesigner 2023.11000.1234",
    "version": "2023.11000.1234",
    "osName": "Windows",
    "mcpApiVersion": "1.0.0"
  }
}
```

If you get a connection error, check that the WebServer DAT inside the base is **Active** and set to port **9981**.

---

## Connecting Claude Code

Claude Code can talk to TouchDesigner through the MCP server using Python scripts that send HTTP requests.

### How It Works

Claude Code runs shell commands and Python scripts on your machine. These scripts use `urllib` or `requests` to send HTTP calls to `http://localhost:9981/api/...`, which the WebServer DAT inside TouchDesigner receives and processes.

### Example: Push a Script from Claude Code to TD

```python
import json, urllib.request

API = 'http://localhost:9981/api/exec'
script = 'result = op("/project1").name'

data = json.dumps({'script': script}).encode('utf-8')
req = urllib.request.Request(API, data=data, headers={'Content-Type': 'application/json'})
resp = urllib.request.urlopen(req)
print(json.loads(resp.read()))
```

### What Claude Code Can Do via MCP

- **Read any node** — get all parameters, types, paths
- **Create nodes** — add new CHOPs, TOPs, DATs, SOPs, COMPs
- **Modify parameters** — change any parameter on any operator
- **Execute Python** — run arbitrary Python code inside TouchDesigner
- **Debug** — get error messages from nodes
- **Inspect the API** — browse TD's Python classes and methods
- **Push UI/HTML** — update WebRender DATs with new HTML content
- **Control hardware** — send commands to connected devices through TD

---

## Connecting Cursor

Cursor (or any AI code editor) can connect using the same HTTP API.

### Option 1: Direct HTTP Calls in Terminal

Use the integrated terminal in Cursor to make `curl` calls:

```bash
# List all nodes in your project
curl "http://localhost:9981/api/nodes?parentPath=/project1"

# Execute Python inside TD
curl -X POST http://localhost:9981/api/exec \
  -H "Content-Type: application/json" \
  -d '{"script": "result = str([c.name for c in op(\"/project1\").children])"}'
```

### Option 2: Write a Helper Script

Create a small Python helper to simplify repeated calls:

```python
# td_api.py — drop this in your project
import json, urllib.request

def td_exec(script):
    """Execute Python inside TouchDesigner and return the result"""
    data = json.dumps({'script': script}).encode('utf-8')
    req = urllib.request.Request(
        'http://localhost:9981/api/exec',
        data=data,
        headers={'Content-Type': 'application/json'}
    )
    resp = urllib.request.urlopen(req)
    return json.loads(resp.read())

def td_node(path):
    """Get full details of a node"""
    resp = urllib.request.urlopen(f'http://localhost:9981/api/node/{path}')
    return json.loads(resp.read())

def td_nodes(parent_path='/'):
    """List children of a node"""
    resp = urllib.request.urlopen(f'http://localhost:9981/api/nodes?parentPath={parent_path}')
    return json.loads(resp.read())

def td_create(parent_path, node_type, name=None, params=None):
    """Create a new node"""
    body = {'parentPath': parent_path, 'nodeType': node_type}
    if name: body['nodeName'] = name
    if params: body['parameters'] = params
    data = json.dumps(body).encode('utf-8')
    req = urllib.request.Request(
        'http://localhost:9981/api/node',
        data=data,
        headers={'Content-Type': 'application/json'}
    )
    resp = urllib.request.urlopen(req)
    return json.loads(resp.read())

def td_update(node_path, properties):
    """Update node parameters"""
    data = json.dumps({'properties': properties}).encode('utf-8')
    req = urllib.request.Request(
        f'http://localhost:9981/api/node/{node_path}',
        data=data,
        headers={'Content-Type': 'application/json'},
        method='PUT'
    )
    resp = urllib.request.urlopen(req)
    return json.loads(resp.read())
```

Then in your AI tool's chat, just say: "use td_api.py to create a noise CHOP in /project1"

---

## API Reference

Base URL: `http://localhost:9981`

### Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/api/info` | TouchDesigner version, OS, MCP API version |
| `GET` | `/api/nodes?parentPath=/` | List child nodes under a path |
| `GET` | `/api/node/{path}` | Get full details of a node (all parameters) |
| `POST` | `/api/node` | Create a new node |
| `PUT` | `/api/node/{path}` | Update node parameters |
| `DELETE` | `/api/node/{path}` | Delete a node |
| `GET` | `/api/node/{path}/errors` | Get error messages for a node and children |
| `POST` | `/api/node/{path}/method` | Call a method on a node (cook, pulse, etc.) |
| `POST` | `/api/exec` | Execute arbitrary Python script inside TD |
| `GET` | `/api/python/classes` | List all TD Python classes |
| `GET` | `/api/python/class/{name}` | Get methods/properties of a TD class |
| `GET` | `/api/python/help/{module}` | Get Python help() for a module |

### Request/Response Format

All requests and responses use JSON.

**Request body for `/api/exec`:**
```json
{
  "script": "result = op('/project1').name"
}
```

**Response format (success):**
```json
{
  "success": true,
  "data": {
    "result": "project1",
    "stdout": "",
    "stderr": ""
  }
}
```

**Response format (error):**
```json
{
  "success": false,
  "error": "Node not found at path: /nonexistent",
  "errorCategory": "NOT_FOUND"
}
```

### Request Body for Creating Nodes

```json
{
  "parentPath": "/project1",
  "nodeType": "noiseCHOP",
  "nodeName": "my_noise",
  "parameters": {
    "roughness": 0.5,
    "seed": 42
  }
}
```

### Request Body for Updating Nodes

```json
{
  "properties": {
    "roughness": 0.8,
    "amp": 2.0
  }
}
```

### Request Body for Calling Node Methods

```json
{
  "method": "cook",
  "args": [],
  "kwargs": {}
}
```

---

## Example Usage

### 1. Check if TD is running
```bash
curl http://localhost:9981/api/info
```

### 2. List all nodes in your project
```bash
curl "http://localhost:9981/api/nodes?parentPath=/project1"
```

### 3. Get details of a specific node
```bash
curl http://localhost:9981/api/node//project1/constant1
```

### 4. Create a Text TOP
```bash
curl -X POST http://localhost:9981/api/node \
  -H "Content-Type: application/json" \
  -d '{"parentPath": "/project1", "nodeType": "textTOP", "nodeName": "myText", "parameters": {"text": "Hello World"}}'
```

### 5. Update a parameter
```bash
curl -X PUT http://localhost:9981/api/node//project1/myText \
  -H "Content-Type: application/json" \
  -d '{"properties": {"text": "Updated!", "fontsize": 32}}'
```

### 6. Run Python inside TouchDesigner
```bash
curl -X POST http://localhost:9981/api/exec \
  -H "Content-Type: application/json" \
  -d '{"script": "result = [c.name for c in op(\"/project1\").children]"}'
```

### 7. Delete a node
```bash
curl -X DELETE http://localhost:9981/api/node//project1/myText
```

### 8. Get errors from a node
```bash
curl http://localhost:9981/api/node//project1/errors
```

### 9. Browse TD Python API
```bash
# List all classes
curl http://localhost:9981/api/python/classes

# Get details of a specific class
curl http://localhost:9981/api/python/class/noiseCHOP

# Get full help text
curl http://localhost:9981/api/python/help/td
```

---

## Architecture

```
┌─────────────────────────────────────────┐
│  External Tool                          │
│  (Claude Code / Cursor / Python Script) │
└──────────────┬──────────────────────────┘
               │ HTTP (localhost:9981)
               v
┌──────────────────────────────────────┐
│  TouchDesigner                       │
│  ┌────────────────────────────────┐  │
│  │  WebServer DAT (port 9981)     │  │
│  │  callback: mcp_webserver_script│  │
│  └──────────┬─────────────────────┘  │
│             v                        │
│  ┌────────────────────────────────┐  │
│  │  import_modules.py             │  │
│  │  - Adds modules/ to sys.path  │  │
│  │  - Loads OpenAPI schema        │  │
│  └──────────┬─────────────────────┘  │
│             v                        │
│  ┌────────────────────────────────┐  │
│  │  api_controller.py             │  │
│  │  - Receives HTTP request       │  │
│  │  - CORS headers                │  │
│  │  - Routes via OpenAPI schema   │  │
│  └──────────┬─────────────────────┘  │
│             v                        │
│  ┌────────────────────────────────┐  │
│  │  openapi_router.py             │  │
│  │  - Matches URL to operationId  │  │
│  │  - Extracts path/query params  │  │
│  └──────────┬─────────────────────┘  │
│             v                        │
│  ┌────────────────────────────────┐  │
│  │  generated_handlers.py         │  │
│  │  - One handler per endpoint    │  │
│  │  - Parses body, maps params    │  │
│  │  - Calls api_service methods   │  │
│  └──────────┬─────────────────────┘  │
│             v                        │
│  ┌────────────────────────────────┐  │
│  │  api_service.py                │  │
│  │  - td.op() / td.ops()         │  │
│  │  - Node CRUD operations        │  │
│  │  - Python exec in TD context   │  │
│  │  - Error collection            │  │
│  │  - API introspection           │  │
│  └────────────────────────────────┘  │
└──────────────────────────────────────┘
```

### How a Request Flows

1. External tool sends `GET http://localhost:9981/api/nodes?parentPath=/project1`
2. **WebServer DAT** receives it, calls `onHTTPRequest()` in `mcp_webserver_script.py`
3. **ControllerManager** passes it to `api_controller.py`
4. **RequestProcessor** normalizes the request (extracts method, path, query params, body)
5. **OpenAPIRouter** matches `GET /api/nodes` to operation ID `get_nodes`
6. **generated_handlers.py** `get_nodes()` handler is called with `parent_path="/project1"`
7. **api_service.py** `get_nodes()` calls `td.op("/project1").findChildren(depth=1)`
8. Result is serialized to JSON and sent back as the HTTP response

---

## Troubleshooting

### "Connection refused" when calling the API
- Check that TouchDesigner is running
- Check the WebServer DAT is **Active** (toggle it on)
- Verify the port is `9981` (or whatever you set it to)
- Try `http://127.0.0.1:9981/api/info` instead of `localhost`

### "OpenAPI schema not available"
- Make sure the `modules/` folder is next to your `.tox` file
- Check that `modules/td_server/openapi_server/openapi/openapi.yaml` exists
- Verify `PyYAML` is installed: `pip install pyyaml`

### "Module not found" errors in TD console
- Check that `import_modules.py` is running and adding paths to `sys.path`
- Open TD's Textport and run: `import sys; print(sys.path)` to verify paths

### API returns success but result is empty
- Use the `result` variable in your exec scripts: `result = op('/project1').name`
- Without setting `result`, the response will be `null`

### Changes not reflected in TD
- TD caches Python modules. After editing `.py` files, restart TD or run in Textport:
  ```python
  import importlib
  import mcp.services.api_service
  importlib.reload(mcp.services.api_service)
  ```

### CORS errors from browser-based tools
- The server already sends `Access-Control-Allow-Origin: *` headers
- If you still get CORS errors, make sure you're hitting the right port

---

## What Can You Build With This?

- **AI-powered TD development** — Let Claude/Cursor read your network, create nodes, fix errors
- **External control panels** — Build web UIs that control TD parameters in real-time
- **Automated project setup** — Script entire network creation from templates
- **CI/CD for TD** — Run tests and validation on TD projects
- **Hardware integration** — Bridge external hardware controllers through the API
- **Live performance tools** — External apps that drive TD during a show
- **Debugging dashboards** — Monitor TD node errors and performance from outside

---

*Built for the ServoKineticControl project — TouchDesigner + EtherCAT motor control with AI assistance.*
