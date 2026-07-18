#!/usr/bin/env python3
"""BlueGill startup-melody editor — compose a tune, HEAR it on the PC exactly as the ESC will
play it, and export the `Eep_Pgm_Beep_Melody: DB ...` bytes for src/Bluejay.asm.

The preview uses the REAL Bluejay beep timing model (derived from Modules/Fx.asm `beep` + the
code's own "150 djnz = 25us" calibration), so what you hear ~= what the ESC makes, including the
achievable frequency range (~111..2271 Hz) and duration quantization. No GUI/audio deps (browser
Web Audio); stdlib only. Save/load melodies (they appear in the preset list); insert anywhere.

    python3 melody_editor.py                      # opens the editor in your browser
    python3 melody_editor.py --decode "2,58,4,32,94,51,..."   # print a byte-melody as real notes

Model:  period_us = FIXED_US + SLOPE_US * pitch   (pitch = item2, the pulse half-period byte)
        freq_hz   = 1e6 / period_us               (LOWER pitch byte => HIGHER note)
        dur_ms    = pulses * period_us / 1000      (pulses = item1)
Defaults FIXED_US=406.7, SLOPE_US=33.6 at beep_strength=40 (both tweakable in the UI to match your
ESC by ear). Bluejay melody bytes: 4 header, then (item1,item2) pairs, item1==0 ends; item2==0 =>
item1 ms of silence.
"""
import sys, json, webbrowser, argparse, os, re
from http.server import BaseHTTPRequestHandler, HTTPServer

FIXED_US = 406.7
SLOPE_US = 33.6
HEADER   = [2, 58, 4, 32]
MELO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "melodies")

def period_us(pitch):  return FIXED_US + SLOPE_US * max(1, pitch)
def pitch_of_freq(f):  return max(1, min(255, int(round((1e6 / max(1.0, f) - FIXED_US) / SLOPE_US))))
def freq_of_pitch(p):  return 1e6 / period_us(p)
def pulses_of(ms, p):  return max(1, min(255, int(round(ms * 1000.0 / period_us(p)))))
def dur_of(pulses, p): return pulses * period_us(p) / 1000.0

def notes_to_bytes(events):
    out = list(HEADER)
    for e in events:
        if e['type'] == 'rest':
            out += [max(1, min(255, int(round(e['dur'])))), 0]
        else:
            p = pitch_of_freq(e['freq']); out += [pulses_of(e['dur'], p), p]
    out += [0]
    return out

def bytes_to_notes(bs):
    ev = []; body = bs[4:]; i = 0
    while i + 1 < len(body):
        it1, it2 = body[i], body[i + 1]
        if it1 == 0: break
        if it2 == 0: ev.append({'type': 'rest', 'dur': it1})
        else: ev.append({'type': 'note', 'freq': round(freq_of_pitch(it2), 1), 'dur': round(dur_of(it1, it2), 1)})
        i += 2
    return ev

def db_line(bs): return "Eep_Pgm_Beep_Melody: DB " + ",".join(str(b) for b in bs)
def safe(name): return re.sub(r'[^A-Za-z0-9._-]', '_', name)[:32] or "melody"

PAGE = r"""<!doctype html><html><head><meta charset=utf-8><title>BlueGill melody editor</title>
<style>
 body{font-family:system-ui,sans-serif;margin:1.1em;max-width:860px;color:#222}
 h1{font-size:1.25em} table{border-collapse:collapse;margin:.5em 0;width:100%} td,th{border:1px solid #ccc;padding:.2em .5em}
 button{margin:.1em;padding:.3em .6em} input,select{padding:.2em}
 #db{width:100%;height:4.5em;font-family:monospace;font-size:.82em}
 .muted{color:#888;font-size:.85em} .warn{color:#c60}
 .bar{display:flex;gap:.4em;flex-wrap:wrap;align-items:center;margin:.35em 0}
 tr.cur{background:#eef6ff} tr.cur td:first-child::before{content:"▶ "}
 td{text-align:center}
</style></head><body>
<h1>🌊 BlueGill startup-melody editor</h1>
<p class=muted>Preview = the REAL ESC beep timing (what you hear ≈ what the ESC makes). Click a row to
set the insertion point (▶); <b>+note/+rest</b> insert after it. Export the <b>DB</b> line into
<code>Eep_Pgm_Beep_Melody:</code> in <code>src/Bluejay.asm</code>, rebuild, reflash.</p>
<div class=bar>
 Note <select id=note></select> Octave <select id=oct></select>
 Dur(ms) <input id=dur type=number value=160 min=10 max=540 style=width:5em>
 <button onclick=insNote()>+ note</button> <button onclick=insRest()>+ rest</button>
 <button onclick=play()>▶ Play</button> <button onclick=stopAll()>■ Stop</button>
 <button onclick=clearAll()>clear</button>
</div>
<div class=bar>
 Melody <select id=preset></select> <button onclick=loadPreset()>load</button>
 <button onclick=saveAs()>save as…</button>
 &nbsp;<span class=muted>range <span id=range></span></span>
</div>
<div class=bar class=muted>calibrate: FIXED_US <input id=fx type=number value=406.7 step=0.1 style=width:5.5em oninput=refresh()>
 SLOPE_US <input id=sl type=number value=33.6 step=0.1 style=width:5em oninput=refresh()>
 <span class=muted>(match your ESC by ear if needed)</span></div>
<table id=tbl><thead><tr><th>#</th><th>type</th><th>note→actual</th><th>dur ms</th><th>edit</th></tr></thead><tbody></tbody></table>
<p>DB bytes (paste over <code>Eep_Pgm_Beep_Melody:</code>):</p>
<textarea id=db readonly></textarea>
<div class=bar><button onclick=copyDb()>copy DB line</button><span id=msg class=muted></span></div>
<script>
const NOTES=["C","C#","D","D#","E","F","F#","G","G#","A","A#","B"];
let FIXED=406.7, SLOPE=33.6, ev=[], cur=-1, ac=null, playing=false;
function midi(n,o){return 12*(o+1)+NOTES.indexOf(n);}
function freqNote(n,o){return 440*Math.pow(2,(midi(n,o)-69)/12);}
function periodUs(p){return FIXED+SLOPE*Math.max(1,p);}
function pitchOf(f){return Math.max(1,Math.min(255,Math.round((1e6/Math.max(1,f)-FIXED)/SLOPE)));}
function freqOf(p){return 1e6/periodUs(p);}
function pulsesOf(ms,p){return Math.max(1,Math.min(255,Math.round(ms*1000/periodUs(p))));}
function durOf(pulses,p){return pulses*periodUs(p)/1000;}
function bytes(){let o=[2,58,4,32]; for(const e of ev){ if(e.type=='rest'){o.push(Math.max(1,Math.min(255,Math.round(e.dur))),0);} else {let p=pitchOf(e.freq); o.push(pulsesOf(e.dur,p),p);} } o.push(0); return o;}
function noteName(f){ // nearest note name for a frequency
 let m=Math.round(69+12*Math.log2(f/440)); return NOTES[(m%12+12)%12]+(Math.floor(m/12)-1);}
function refresh(){
 FIXED=+document.getElementById('fx').value||406.7; SLOPE=+document.getElementById('sl').value||33.6;
 const fmin=freqOf(255),fmax=freqOf(1);
 document.getElementById('range').textContent=Math.round(fmin)+"–"+Math.round(fmax)+" Hz";
 const tb=document.querySelector('#tbl tbody'); tb.innerHTML='';
 ev.forEach((e,i)=>{let tr=document.createElement('tr'); if(i==cur)tr.className='cur';
  tr.onclick=()=>{cur=i;refresh();};
  let desc='—', warn='';
  if(e.type=='note'){ let p=pitchOf(e.freq), af=freqOf(p);
   let want=(e.name?e.name+e.oct:noteName(e.freq));
   desc=want+' → '+Math.round(af)+'Hz('+noteName(af)+') p'+p;
   if(e.freq>freqOf(1)+1) warn=' too HIGH (max '+Math.round(fmax)+'Hz)';
   else if(e.freq<freqOf(255)-1) warn=' too LOW (min '+Math.round(fmin)+'Hz)';
   else if(Math.abs(af-e.freq)/e.freq>0.03) warn=' (quantized)';
  }
  let ap=e.type=='note'?pitchOf(e.freq):0, adur=e.type=='note'?durOf(pulsesOf(e.dur,ap),ap):e.dur;
  tr.innerHTML=`<td>${i+1}</td><td>${e.type}</td><td>${desc}<span class=warn>${warn}</span></td>`+
   `<td>${Math.round(adur)}</td><td><button onclick="event.stopPropagation();del(${i})">x</button>`+
   `<button onclick="event.stopPropagation();mv(${i},-1)">↑</button>`+
   `<button onclick="event.stopPropagation();mv(${i},1)">↓</button></td>`;
  tb.appendChild(tr);});
 const bs=bytes();
 document.getElementById('db').value='Eep_Pgm_Beep_Melody: DB '+bs.join(',')+'\n; '+ev.length+' events, '+bs.length+' bytes';
}
function insAt(e){ let at=(cur<0?ev.length:cur+1); ev.splice(at,0,e); cur=at; refresh(); }
function insNote(){let n=document.getElementById('note').value,o=+document.getElementById('oct').value,
  d=+document.getElementById('dur').value; insAt({type:'note',name:n,oct:o,freq:freqNote(n,o),dur:d});}
function insRest(){insAt({type:'rest',dur:+document.getElementById('dur').value});}
function del(i){ev.splice(i,1); if(cur>=ev.length)cur=ev.length-1; refresh();}
function mv(i,d){let j=i+d; if(j<0||j>=ev.length)return;[ev[i],ev[j]]=[ev[j],ev[i]];cur=j;refresh();}
function clearAll(){ev=[];cur=-1;refresh();}
async function play(){
 if(playing)return; playing=true; ac=ac||new (window.AudioContext||window.webkitAudioContext)();
 let t=ac.currentTime+0.05;
 for(const e of ev){
  let dur, f=0;
  if(e.type=='note'){ let p=pitchOf(e.freq); f=freqOf(p); dur=durOf(pulsesOf(e.dur,p),p)/1000; }
  else dur=e.dur/1000;
  if(f>0){ const o=ac.createOscillator(),g=ac.createGain(); o.type='square'; o.frequency.value=f;
   g.gain.setValueAtTime(0.0001,t); g.gain.exponentialRampToValueAtTime(0.22,t+0.004);
   g.gain.setValueAtTime(0.22,t+Math.max(0.01,dur-0.008)); g.gain.exponentialRampToValueAtTime(0.0001,t+dur);
   o.connect(g).connect(ac.destination); o.start(t); o.stop(t+dur+0.01); }
  t+=dur+0.008;
 }
 setTimeout(()=>playing=false,(t-ac.currentTime)*1000+60);
}
function stopAll(){if(ac){ac.close();ac=null;} playing=false;}
function copyDb(){navigator.clipboard.writeText(document.getElementById('db').value.split('\n')[0])
  .then(()=>document.getElementById('msg').textContent='copied ✓');}
// ---- presets: built-in + saved (server) ----
const BUILTIN={
 "chime (major arpeggio)":[["C",5,150],["E",5,150],["G",5,150],["C",6,320]],
 "power-on ding":[["G",5,120],["C",6,300]],
 "two-tone boot":[["C",6,110],["rest",0,40],["G",5,110],["rest",0,40],["C",6,240]],
 "little fanfare":[["C",5,120],["E",5,120],["G",5,120],["C",6,180],["G",5,120],["C",6,360]],
 "current 'under the sea'":[["G",5,150],["G",5,150],["G",5,150],["E",5,300],["rest",0,120],["G",5,150],["G",5,150],["G",5,150],["C",6,380]]
};
let SAVED={};
function toEvents(a){return a.map(x=>x[0]=='rest'?{type:'rest',dur:x[2]}:{type:'note',name:x[0],oct:x[1],freq:freqNote(x[0],x[1]),dur:x[2]});}
function fillPresets(){let s=document.getElementById('preset'); s.innerHTML='';
 for(const k in BUILTIN){let o=document.createElement('option');o.value='b:'+k;o.textContent=k;s.appendChild(o);}
 for(const k in SAVED){let o=document.createElement('option');o.value='s:'+k;o.textContent='★ '+k;s.appendChild(o);}}
function loadPreset(){let v=document.getElementById('preset').value;
 if(v.startsWith('b:')) ev=toEvents(BUILTIN[v.slice(2)]);
 else if(v.startsWith('s:')) ev=JSON.parse(JSON.stringify(SAVED[v.slice(2)]));
 cur=ev.length-1; refresh();}
async function refreshSaved(){try{SAVED=await (await fetch('/api/list')).json();}catch(e){SAVED={};} fillPresets();}
async function saveAs(){let name=prompt('save melody as:'); if(!name)return;
 let r=await (await fetch('/api/save',{method:'POST',body:JSON.stringify({name,events:ev})})).json();
 document.getElementById('msg').textContent=r.ok?('saved ★'+r.name):('error: '+(r.err||'?'));
 await refreshSaved(); document.getElementById('preset').value='s:'+r.name;}
// init
NOTES.forEach(n=>{let o=document.createElement('option');o.textContent=n;document.getElementById('note').appendChild(o);});
for(let i=3;i<=7;i++){let o=document.createElement('option');o.textContent=i;if(i==5)o.selected=1;document.getElementById('oct').appendChild(o);}
refreshSaved().then(()=>{document.getElementById('preset').value='b:chime (major arpeggio)';loadPreset();});
</script></body></html>"""

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def _json(self, obj):
        b = json.dumps(obj).encode(); self.send_response(200)
        self.send_header("Content-Type", "application/json"); self.end_headers(); self.wfile.write(b)
    def do_GET(self):
        if self.path == "/" or self.path.startswith("/?"):
            self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers(); self.wfile.write(PAGE.encode()); return
        if self.path == "/api/list":
            out = {}
            if os.path.isdir(MELO_DIR):
                for fn in sorted(os.listdir(MELO_DIR)):
                    if fn.endswith(".json"):
                        try: out[fn[:-5]] = json.load(open(os.path.join(MELO_DIR, fn)))
                        except Exception: pass
            self._json(out); return
        self.send_response(404); self.end_headers()
    def do_POST(self):
        if self.path == "/api/save":
            n = int(self.headers.get("Content-Length", 0)); body = json.loads(self.rfile.read(n) or b"{}")
            name = safe(str(body.get("name", ""))); events = body.get("events", [])
            os.makedirs(MELO_DIR, exist_ok=True)
            json.dump(events, open(os.path.join(MELO_DIR, name + ".json"), "w"))
            self._json({"ok": True, "name": name}); return
        self.send_response(404); self.end_headers()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--decode", help="comma-separated melody bytes -> real notes")
    args = ap.parse_args()
    if args.decode:
        bs = [int(x) for x in args.decode.replace(" ", "").split(",") if x != ""]
        print("header:", bs[:4], f"  (model FIXED={FIXED_US}us SLOPE={SLOPE_US}us)")
        for e in bytes_to_notes(bs):
            print("  rest %d ms" % e['dur'] if e['type'] == 'rest' else "  note %6.1f Hz  %5.1f ms" % (e['freq'], e['dur']))
        return
    url = "http://127.0.0.1:%d/" % args.port
    print("BlueGill melody editor at", url, "(Ctrl-C to stop)  saved melodies:", MELO_DIR)
    try: webbrowser.open(url)
    except Exception: pass
    HTTPServer(("127.0.0.1", args.port), H).serve_forever()

if __name__ == "__main__":
    main()
