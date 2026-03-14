"""
MCP package initialization
Loads OpenAPI schema for routing
"""

import os
import json

openapi_schema = None

_base_dir = os.path.dirname(__file__)
_json_path = os.path.normpath(os.path.join(_base_dir, '..', 'td_server', 'openapi_server', 'openapi', 'openapi.json'))
_yaml_path = os.path.normpath(os.path.join(_base_dir, '..', 'td_server', 'openapi_server', 'openapi', 'openapi.yaml'))

# Try JSON first (no dependencies)
if os.path.exists(_json_path):
    try:
        with open(_json_path, 'r', encoding='utf-8') as f:
            openapi_schema = json.load(f)
        print(f"MCP: Loaded OpenAPI schema from {_json_path}")
    except Exception as e:
        print(f"MCP: Failed to load JSON schema: {e}")

# Fallback to YAML
if openapi_schema is None and os.path.exists(_yaml_path):
    try:
        import yaml
        with open(_yaml_path, 'r', encoding='utf-8') as f:
            openapi_schema = yaml.safe_load(f)
        print(f"MCP: Loaded OpenAPI schema from {_yaml_path}")
    except Exception as e:
        print(f"MCP: Failed to load YAML schema: {e}")

if openapi_schema is None:
    openapi_schema = {"paths": {}}
    print("MCP: WARNING - Using empty OpenAPI schema")
