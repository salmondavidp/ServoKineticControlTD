# MCP Server for TouchDesigner — How It Works

## What is it?
An HTTP REST API server that runs **inside TouchDesigner**, letting any external tool (Claude, Cursor, scripts, apps) control and inspect your TouchDesigner project remotely.

## How to Connect
TouchDesigner runs a **WebServer DAT** on port `9981`. Any HTTP client can talk to it:

```
Base URL: http://localhost:9981
```

No authentication needed — it's local-only by default.

## Available API Endpoints

| Method | Endpoint | What it does |
|--------|----------|-------------|
| `GET` | `/api/info` | Get TD version, OS, MCP API version |
| `GET` | `/api/nodes?parentPath=/` | List all nodes under a path |
| `GET` | `/api/node/{path}` | Get full details of a specific node (params, type, etc.) |
| `POST` | `/api/node` | Create a new node (type, name, parent, parameters) |
| `PUT` | `/api/node/{path}` | Update node parameters |
| `DELETE` | `/api/node/{path}` | Delete a node |
| `GET` | `/api/node/{path}/errors` | Get error messages for a node and children |
| `POST` | `/api/node/{path}/method` | Call a method on a node (e.g. `cook()`, `pulse()`) |
| `POST` | `/api/exec` | Execute arbitrary Python script inside TD |
| `GET` | `/api/python/classes` | List all TD Python classes |
| `GET` | `/api/python/class/{name}` | Get methods/properties of a TD class |
| `GET` | `/api/python/help/{module}` | Get Python help() for a module |

## Example Usage

**Get TD info:**
```bash
curl http://localhost:9981/api/info
```

**List root nodes:**
```bash
curl "http://localhost:9981/api/nodes?parentPath=/"
```

**Execute a Python script inside TD:**
```bash
curl -X POST http://localhost:9981/api/exec \
  -H "Content-Type: application/json" \
  -d '{"script": "result = op(\"/project1\").name"}'
```

**Create a node:**
```bash
curl -X POST http://localhost:9981/api/node \
  -H "Content-Type: application/json" \
  -d '{"parentPath": "/project1", "nodeType": "textTOP", "nodeName": "myText"}'
```

**Update a node's parameters:**
```bash
curl -X PUT http://localhost:9981/api/node//project1/myText \
  -H "Content-Type: application/json" \
  -d '{"properties": {"text": "Hello World", "fontsize": 24}}'
```

## What Can You Do With It?
- **AI-assisted development** — Connect Claude/Cursor to read, create, modify, and debug TD nodes
- **Remote control** — Build external UIs or scripts that control TD
- **Automation** — Script node creation, parameter updates, project setup
- **Debugging** — Inspect node errors, run diagnostic scripts
- **Introspection** — Browse TD's Python API, get help on any class/module

## Architecture
```
External Tool (Claude, Cursor, Script)
        |
        | HTTP REST API
        v
  WebServer DAT (port 9981)
        |
  mcp_webserver_script.py (entry point)
        |
  OpenAPI Router (matches routes)
        |
  Generated Handlers (per endpoint)
        |
  API Service (talks to TouchDesigner)
        |
  TouchDesigner (td.op, td.ops, etc.)
```
