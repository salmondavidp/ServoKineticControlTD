#!/usr/bin/env python3
"""Push updated script1_callbacks to TouchDesigner"""
import urllib.request
import json
import base64

API = "http://localhost:9981/api/exec"

callback_code = r'''# DNA Multi-Pattern Cue Timeline Sequencer (with Servo support)

import json as _json

def onSetupParameters(scriptOp):
	pass

# Track which servo patterns have been triggered to avoid re-triggering
_servo_triggered = {}

def onCook(scriptOp):
	global _servo_triggered
	scriptOp.clear()
	ctrl = scriptOp.parent()
	total_dur = ctrl.par.Totaldur.eval()
	if total_dur <= 0:
		total_dur = 1.0
	frac = scriptOp.inputs[0]["timer_fraction"].eval()
	elapsed = frac * total_dur
	first_ch = int(ctrl.par.Firstchan.eval())
	total_ch = int(ctrl.par.Totalchans.eval())
	# Read disabled channels
	dis_tbl = ctrl.op("disabled_chans")
	disabled = set()
	if dis_tbl and dis_tbl.numRows >= 2:
		for r in range(1, dis_tbl.numRows):
			disabled.add(int(dis_tbl[r, "chan"].val))
	# Init all channels to 0
	chan_vals = {}
	for c in range(total_ch):
		chan_vals["chan" + str(first_ch + c)] = 0.0
	# Read cue timeline
	cue_tl = ctrl.op("cue_timeline")
	cues_tbl = ctrl.op("cues")
	patterns_tbl = ctrl.op("patterns")
	all_patterns = []
	servo_patterns = []
	# From cue_timeline: each row is a cue placed at a time
	if cue_tl and cue_tl.numRows >= 2 and cues_tbl and cues_tbl.numRows >= 2:
		for row in range(1, cue_tl.numRows):
			cue_name = cue_tl[row, "cue_name"].val
			cue_start = float(cue_tl[row, "start_s"].val)
			for cr in range(1, cues_tbl.numRows):
				if cues_tbl[cr, "name"].val == cue_name:
					try:
						pats = _json.loads(cues_tbl[cr, "patterns_json"].val)
						for p in pats:
							pat_type = p.get("type", "dmx")
							if pat_type == "servo":
								servo_patterns.append({
									"start": p["start"] + cue_start,
									"steps": p.get("steps", []),
									"loop_mode": p.get("loop_mode", "once"),
									"loop_count": p.get("loop_count", 1),
									"seq_mode": p.get("seq_mode", "PV"),
									"cue_name": cue_name,
								})
							else:
								all_patterns.append({
									"start": p["start"] + cue_start,
									"ramp_up": p["ramp_up"],
									"hold_high": p["hold_high"],
									"ramp_down": p["ramp_down"],
									"hold_low": p["hold_low"],
									"start_ch": p["start_chan"],
									"num_ch": p["num_chans"],
									"delay_s": p["delay_ms"] / 1000.0,
									"channels": p.get("channels", None),
								})
					except:
						pass
					break
	# Fallback: read direct patterns table
	if len(all_patterns) == 0 and patterns_tbl and patterns_tbl.numRows >= 2:
		for row in range(1, patterns_tbl.numRows):
			all_patterns.append({
				"start": float(patterns_tbl[row, "start_s"].val),
				"ramp_up": float(patterns_tbl[row, "ramp_up"].val),
				"hold_high": float(patterns_tbl[row, "hold_high"].val),
				"ramp_down": float(patterns_tbl[row, "ramp_down"].val),
				"hold_low": float(patterns_tbl[row, "hold_low"].val),
				"start_ch": int(patterns_tbl[row, "start_chan"].val),
				"num_ch": int(patterns_tbl[row, "num_chans"].val),
				"delay_s": float(patterns_tbl[row, "delay_ms"].val) / 1000.0,
			})
	# Handle servo patterns - trigger EtherCAT sequence at the right time
	# Reset triggers if timeline restarted (elapsed near 0)
	if elapsed < 0.1:
		_servo_triggered = {}
	for sp in servo_patterns:
		sp_key = sp["cue_name"] + "_" + str(sp["start"])
		if elapsed >= sp["start"] and sp_key not in _servo_triggered:
			_servo_triggered[sp_key] = True
			# Trigger EtherCAT sequence via external ec_server (HTTP on port 9982)
			try:
				import threading, json
				def _do_servo_sequence(steps_data, loop_mode, loop_count, seq_mode):
					try:
						import urllib.request
						ec_steps = []
						for s in steps_data:
							if seq_mode == "PV":
								ec_steps.append({
									"action1": s.get("action1", "forward"),
									"speed1": s.get("speed1", 80000),
									"dur1": s.get("dur1", 2),
									"hold": s.get("hold", 0),
									"action2": s.get("action2", "backward"),
									"speed2": s.get("speed2", 80000),
									"dur2": s.get("dur2", 2),
								})
							elif seq_mode == "CSP":
								pos_str = s.get("positions", "0.0")
								pos_list = [float(x.strip()) for x in pos_str.split(",")]
								pos_dict = {str(i): v for i, v in enumerate(pos_list)}
								ec_steps.append({
									"positions": pos_dict,
									"csp_velocity": s.get("csp_velocity", 100),
									"hold": s.get("hold", 0),
								})
							else:
								pos_str = s.get("positions", "0.0")
								pos_list = [float(x.strip()) for x in pos_str.split(",")]
								pos_dict = {str(i): v for i, v in enumerate(pos_list)}
								ec_steps.append({
									"positions": pos_dict,
									"velocity": s.get("velocity", 80000),
									"accel": s.get("accel", 6000),
									"decel": s.get("decel", 6000),
									"hold": s.get("hold", 0),
								})
						body = {
							"steps": ec_steps,
							"loop_count": loop_count if loop_mode == "count" else 1,
							"loop_forever": loop_mode == "forever",
						"mode": seq_mode,
						}
						data = json.dumps(body).encode()
						req = urllib.request.Request(
							"http://localhost:9982/run_sequence",
							data=data,
							headers={"Content-Type": "application/json"}
						)
						urllib.request.urlopen(req, timeout=5)
					except Exception as e:
						print("Servo sequence error:", e)
				t = threading.Thread(target=_do_servo_sequence, args=(sp["steps"], sp["loop_mode"], sp["loop_count"], sp.get("seq_mode", "PV")), daemon=True)
				t.start()
			except:
				pass
	# Compute DMX pattern values
	for p in all_patterns:
		wave_dur = p["ramp_up"] + p["hold_high"] + p["ramp_down"] + p["hold_low"]
		ch_list = p.get("channels", None) or [p["start_ch"] + i for i in range(p["num_ch"])]
		for idx, ch_num in enumerate(ch_list):
			ch_name = "chan" + str(ch_num)
			# Skip disabled channels
			if ch_num in disabled:
				if ch_name not in chan_vals:
					chan_vals[ch_name] = 0.0
				continue
			ch_offset = p["start"] + idx * p["delay_s"]
			ch_elapsed = elapsed - ch_offset
			if ch_elapsed < 0 or wave_dur <= 0:
				v = 0.0
			elif ch_elapsed >= wave_dur:
				v = 0.0
			elif ch_elapsed <= p["ramp_up"]:
				v = ch_elapsed / p["ramp_up"] if p["ramp_up"] > 0 else 1.0
			elif ch_elapsed <= p["ramp_up"] + p["hold_high"]:
				v = 1.0
			elif ch_elapsed <= p["ramp_up"] + p["hold_high"] + p["ramp_down"]:
				v = 1.0 - (ch_elapsed - p["ramp_up"] - p["hold_high"]) / p["ramp_down"] if p["ramp_down"] > 0 else 0.0
			else:
				v = 0.0
			v = max(0.0, min(1.0, v))
			if ch_name in chan_vals:
				chan_vals[ch_name] = max(chan_vals[ch_name], v)
			else:
				chan_vals[ch_name] = v
	# Force disabled channels to 0
	for ch_num in disabled:
		ch_name = "chan" + str(ch_num)
		if ch_name in chan_vals:
			chan_vals[ch_name] = 0.0
	# Output all channels
	for ch_name in sorted(chan_vals.keys(), key=lambda x: int(x.replace("chan",""))):
		chan = scriptOp.appendChan(ch_name)
		chan[0] = chan_vals[ch_name]
'''

b64 = base64.b64encode(callback_code.encode('utf-8')).decode('ascii')

push_script = "import base64\n"
push_script += "dc = me.parent()\n"
push_script += "cb = dc.op('script1_callbacks')\n"
push_script += "code = base64.b64decode('" + b64 + "').decode('utf-8')\n"
push_script += "cb.text = code\n"
push_script += "result = 'script1_callbacks updated'"

data = json.dumps({"script": push_script}).encode('utf-8')
req = urllib.request.Request(API, data=data, headers={"Content-Type": "application/json"})
resp = urllib.request.urlopen(req)
print(json.loads(resp.read()))
