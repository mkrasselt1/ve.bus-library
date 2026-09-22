#pragma once
// Web UI for the mqtt_ha example — plain HTML/CSS/JS, no external CDNs, so it
// also works on networks without internet access.
#include <pgmspace.h>

static const char STYLE_CSS[] PROGMEM = R"rawliteral(
:root{color-scheme:light;--bg:#f4f4f2;--surface:#fcfcfb;--border:#e2e1dc;--text:#0b0b0b;
--text2:#52514e;--muted:#8a8984;--accent:#2a78d6;--s1:#2a78d6;--s2:#eb6834;--s3:#1baf7a;
--good:#1a7f37;--bad:#c62828;--warnbg:#fff4d6;--warnfg:#6b4a00;--grid:#ecebe6}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){color-scheme:dark;--bg:#111110;
--surface:#1a1a19;--border:#2e2e2c;--text:#fff;--text2:#c3c2b7;--muted:#8d8c85;--accent:#3987e5;
--s1:#3987e5;--s2:#d95926;--s3:#199e70;--good:#4caf50;--bad:#ef5350;--warnbg:#3a2f10;--warnfg:#f3d488;--grid:#262624}}
:root[data-theme="dark"]{color-scheme:dark;--bg:#111110;--surface:#1a1a19;--border:#2e2e2c;--text:#fff;
--text2:#c3c2b7;--muted:#8d8c85;--accent:#3987e5;--s1:#3987e5;--s2:#d95926;--s3:#199e70;--good:#4caf50;
--bad:#ef5350;--warnbg:#3a2f10;--warnfg:#f3d488;--grid:#262624}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--text);font:15px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
main{max-width:1100px;margin:0 auto;padding:16px}
header{display:flex;flex-wrap:wrap;align-items:center;gap:8px 16px;margin-bottom:16px}
header h1{font-size:20px;margin:0;font-weight:650}
header .sub{color:var(--text2);font-size:13px}
header .sp{flex:1}
a{color:var(--accent)}
.pill{display:inline-flex;align-items:center;gap:6px;padding:3px 10px;border:1px solid var(--border);
border-radius:999px;font-size:13px;background:var(--surface);color:var(--text2)}
.dot{width:8px;height:8px;border-radius:50%;background:var(--muted)}
.dot.ok{background:var(--good)}.dot.bad{background:var(--bad)}
.banner{background:var(--warnbg);color:var(--warnfg);border-radius:10px;padding:10px 14px;margin-bottom:16px}
.banner a{color:inherit;font-weight:600}
.tiles{display:grid;grid-template-columns:repeat(auto-fill,minmax(160px,1fr));gap:12px;margin-bottom:16px}
.tile,.card{background:var(--surface);border:1px solid var(--border);border-radius:12px}
.tile{padding:12px 14px}
.tile .l{color:var(--text2);font-size:13px}
.tile .v{font-size:26px;font-weight:650;font-variant-numeric:tabular-nums;margin-top:2px}
.tile .v small{font-size:14px;font-weight:500;color:var(--text2);margin-left:3px}
.tile .h{color:var(--muted);font-size:12px;min-height:1em}
.card{padding:14px 16px;margin-bottom:16px}
.card h2{font-size:15px;margin:0 0 10px;font-weight:650}
.row{display:flex;flex-wrap:wrap;align-items:center;gap:8px}
.seg{display:inline-flex;border:1px solid var(--border);border-radius:8px;overflow:hidden}
.seg button{border:0;border-radius:0;background:transparent;padding:5px 12px}
.seg button.on{background:var(--accent);color:#fff}
button,.btn{font:inherit;border:1px solid var(--border);background:var(--surface);color:var(--text);
padding:7px 14px;border-radius:8px;cursor:pointer}
button.primary{background:var(--accent);border-color:var(--accent);color:#fff}
button.danger{color:var(--bad)}
button:disabled{opacity:.5;cursor:default}
input,select{font:inherit;color:var(--text);background:var(--bg);border:1px solid var(--border);
border-radius:8px;padding:7px 10px;min-width:0}
label.f{display:grid;gap:4px;font-size:13px;color:var(--text2)}
.grid2{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:12px}
.chart{position:relative;margin-top:6px}
.chart canvas{width:100%;height:200px;display:block;touch-action:pan-y}
.chart h3{font-size:13px;font-weight:600;margin:14px 0 2px;color:var(--text2)}
.legend{display:flex;flex-wrap:wrap;gap:12px;font-size:13px;color:var(--text2)}
.legend i{display:inline-block;width:12px;height:3px;border-radius:2px;margin-right:5px;vertical-align:middle}
.tip{position:absolute;pointer-events:none;background:var(--surface);border:1px solid var(--border);
border-radius:8px;padding:6px 9px;font-size:12px;box-shadow:0 2px 8px rgba(0,0,0,.15);display:none;white-space:nowrap;z-index:2}
.tip b{font-variant-numeric:tabular-nums}
table{width:100%;border-collapse:collapse;font-size:14px}
td{padding:5px 4px;border-bottom:1px solid var(--grid)}
td:last-child{text-align:right;font-variant-numeric:tabular-nums;color:var(--text)}
td:first-child{color:var(--text2)}
td[colspan]{text-align:left;color:var(--text);padding-top:10px}
.cols{columns:2 300px;column-gap:24px}
.cols table{break-inside:avoid}
footer{color:var(--muted);font-size:12px;margin-top:8px}
.toast{position:fixed;left:50%;bottom:20px;transform:translateX(-50%);background:var(--text);color:var(--bg);
padding:8px 16px;border-radius:8px;display:none;z-index:9}
progress{width:100%;height:8px}
.note{color:var(--muted);font-size:12px}
)rawliteral";

static const char INDEX_HTML[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>VE.Bus Dashboard</title><link rel="stylesheet" href="/style.css"></head><body><main>
<header><div><h1>Victron Multiplus</h1><div class="sub" id="sub">&nbsp;</div></div><span class="sp"></span>
<span class="pill"><span class="dot" id="dSync"></span>VE.Bus</span>
<span class="pill"><span class="dot" id="dMqtt"></span>MQTT</span>
<span class="pill"><span class="dot" id="dWifi"></span><span id="wifi">WiFi</span></span>
<a class="btn" href="/admin/">Settings &amp; control</a></header>
<div class="banner" id="pwBanner" style="display:none">The admin password is still the default. <a href="/admin/">Change it now</a>.</div>
<div class="banner" id="offBanner" style="display:none">Device not reachable — retrying…</div>
<section class="tiles" id="tiles"></section>
<section class="card"><div class="row"><h2 style="margin:0">History</h2><span class="sp" style="flex:1"></span>
<div class="seg" id="range"><button data-n="60">1 h</button><button data-n="360">6 h</button><button data-n="1440" class="on">24 h</button></div></div>
<div class="chart"><h3>Power (W)</h3><div class="legend" id="lgPower"></div><canvas id="cPower"></canvas><div class="tip"></div></div>
<div class="chart"><h3>Battery voltage (V)</h3><canvas id="cBatV"></canvas><div class="tip"></div></div>
<div class="chart"><h3>State of charge</h3><canvas id="cSoc"></canvas><div class="tip"></div></div>
<div class="note" id="histNote"></div></section>
<section class="card"><h2>All values</h2><div class="cols" id="details"></div></section>
<footer id="foot"></footer></main>
<script>
const $=id=>document.getElementById(id);
const TILES=[['output_power','Output power','W',0],['mains_power','Mains power','W',0],['bat_volt','Battery voltage','V',2],
['bat_current','Battery current','A',0],['soc','State of charge','%',0],['ess_power','ESS setpoint','W',0],
['device_state','Device state','',null],['temp','Temperature','°C',1]];
const DETAILS=[['AC','ac_power','AC power','W'],['AC','mains_voltage','Mains voltage','V'],['AC','mains_current','Mains current','A'],
['AC','mains_freq','Mains frequency','Hz'],['AC','inv_voltage','Inverter voltage','V'],['AC','inv_current','Inverter current','A'],
['AC','inv_freq','Inverter frequency','Hz'],['AC','ac_in_min','AC input min','A'],['AC','ac_in_max','AC input max','A'],
['AC','ac_in_actual','AC input actual','A'],['AC','ac_in_config','AC input config',''],
['DC','dc_current','DC current','A'],['DC','dc_allows_inv','DC allows inverting',''],['DC','charger_status','Charger status',''],
['DC','charge_sub_state','Charge sub-state',''],['ESS','ess_power_eff','Effective ESS power','W'],['ESS','virtual_mode','Battery-neutral mode',''],
['ESS','switch_state','Switch state',''],['Status','led_on','LED on',''],['Status','led_blink','LED blink',''],
['Status','ups_status','UPS status (NUT)',''],['Status','nut_clients','NUT clients',''],['Status','checksum_faults','Checksum faults',''],['Status','firmware_version','VE.Bus firmware','']];
const SERIES={cPower:[{i:0,n:'Output',c:'--s1'},{i:1,n:'Mains',c:'--s2'},{i:2,n:'ESS effective',c:'--s3'}],
cBatV:[{i:3,n:'Battery',c:'--s1',k:.01,d:2}],cSoc:[{i:4,n:'SoC',c:'--s1'}]};
let hist=null,range=1440,failCount=0;
$('tiles').innerHTML=TILES.map(t=>`<div class="tile"><div class="l">${t[1]}</div><div class="v" id="t_${t[0]}">–</div><div class="h" id="h_${t[0]}"></div></div>`).join('');
$('lgPower').innerHTML=SERIES.cPower.map(s=>`<span><i style="background:var(${s.c})"></i>${s.n}</span>`).join('');
function dur(s){const d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60);return (d?d+'d ':'')+h+'h '+m+'m'}
function fmtV(v,dec){return typeof v==='number'?v.toFixed(dec):v}
function setDot(id,ok){$(id).className='dot '+(ok?'ok':'bad')}
async function poll(){
  try{const r=await fetch('/api/state',{cache:'no-store'});const s=await r.json();failCount=0;$('offBanner').style.display='none';
    for(const t of TILES){$('t_'+t[0]).innerHTML=t[3]===null?s[t[0]]:`${fmtV(s[t[0]],t[3])}<small>${t[2]}</small>`}
    $('h_device_state').textContent=s.charge_sub_state!=='N/A'?s.charge_sub_state:'';
    $('h_ess_power').textContent=`effective ${s.ess_power_eff} W`+(s.virtual_mode==='ON'?' · battery-neutral':'');
    $('h_bat_volt').textContent=`DC ${fmtV(s.dc_current,1)} A`;
    setDot('dSync',s.sync==='ON');setDot('dMqtt',s.mqtt);setDot('dWifi',s.rssi>-80);$('wifi').textContent=s.rssi+' dBm';
    $('sub').textContent=`${s.device_id} · ${s.ip} · ${s.host}.local`;
    $('pwBanner').style.display=s.default_pw?'block':'none';
    const groups={};for(const d of DETAILS){(groups[d[0]]=groups[d[0]]||[]).push(`<tr><td>${d[2]}</td><td>${s[d[1]]}${d[3]?' '+d[3]:''}</td></tr>`)}
    $('details').innerHTML=Object.entries(groups).map(([g,r])=>`<table><tr><td colspan="2"><b>${g}</b></td></tr>${r.join('')}</table>`).join('');
    $('foot').textContent=`Uptime ${dur(s.uptime)} · free heap ${(s.free_heap/1024).toFixed(0)} kB · last reset: ${s.reset_reason} · MQTT connects ${s.mqtt_reconnects} · WiFi reconnects ${s.wifi_reconnects}`;
  }catch(e){if(++failCount>1)$('offBanner').style.display='block'}
}
async function loadHist(){try{const r=await fetch('/api/history',{cache:'no-store'});hist=await r.json();hist.t=Date.now();drawAll()}catch(e){}}
function nice(x){const p=Math.pow(10,Math.floor(Math.log10(x))),f=x/p;return (f<=1?1:f<=2?2:f<=5?5:10)*p}
function css(v){return getComputedStyle(document.documentElement).getPropertyValue(v).trim()}
function hhmm(t){const d=new Date(t);return d.getHours().toString().padStart(2,'0')+':'+d.getMinutes().toString().padStart(2,'0')}
function drawChart(cv,hover){
  const ser=SERIES[cv.id],dpr=window.devicePixelRatio||1,W=cv.clientWidth,H=cv.clientHeight;
  cv.width=W*dpr;cv.height=H*dpr;const g=cv.getContext('2d');g.scale(dpr,dpr);g.clearRect(0,0,W,H);
  if(!hist)return;const rows=hist.data.slice(-range),n=rows.length,step=hist.interval*1000;
  const tEnd=hist.t-hist.age*1000,tStart=tEnd-(range-1)*step;
  const tOf=i=>tEnd-(n-1-i)*step,val=(r,s)=>r?r[s.i]*(s.k||1):null;
  let lo=Infinity,hi=-Infinity;for(const r of rows)if(r)for(const s of ser){const v=val(r,s);lo=Math.min(lo,v);hi=Math.max(hi,v)}
  const L=52,R=10,T=8,B=22,pw=W-L-R,ph=H-T-B;
  g.font='12px system-ui,sans-serif';g.fillStyle=css('--muted');
  if(lo===Infinity){g.fillText('No data yet — samples are recorded once per minute.',L,T+ph/2);return}
  if(cv.id==='cPower'){lo=Math.min(lo,0);hi=Math.max(hi,0)}
  if(hi-lo<1e-9){hi+=1;lo-=1}
  const st=nice((hi-lo)/4);lo=Math.floor(lo/st)*st;hi=Math.ceil(hi/st)*st;
  const X=t=>L+(t-tStart)/(tEnd-tStart||1)*pw,Y=v=>T+(hi-v)/(hi-lo)*ph,dec=st<1?(st<.1?2:1):0;
  g.strokeStyle=css('--grid');g.lineWidth=1;g.textAlign='right';g.textBaseline='middle';
  for(let v=lo;v<=hi+st/2;v+=st){const y=Math.round(Y(v))+.5;g.beginPath();g.moveTo(L,y);g.lineTo(W-R,y);g.stroke();g.fillText(v.toFixed(dec),L-6,y)}
  g.textAlign='center';g.textBaseline='top';const xt=range<=60?15:range<=360?60:240;
  const first=Math.ceil(tStart/(xt*60000))*xt*60000;
  for(let t=first;t<=tEnd;t+=xt*60000)g.fillText(hhmm(t),X(t),H-B+6);
  g.lineWidth=2;g.lineJoin='round';
  for(const s of ser){g.strokeStyle=css(s.c);g.beginPath();let pen=false;
    rows.forEach((r,i)=>{const v=val(r,s);if(v===null){pen=false;return}const x=X(tOf(i)),y=Y(v);pen?g.lineTo(x,y):g.moveTo(x,y);pen=true});g.stroke()}
  const tip=cv.parentNode.querySelector('.tip');
  if(hover==null||hover<L||hover>W-R){tip.style.display='none';return}
  const i=Math.max(0,Math.min(n-1,Math.round(n-1-(tEnd-(tStart+(hover-L)/pw*(tEnd-tStart)))/step))),r=rows[i],x=X(tOf(i));
  g.strokeStyle=css('--muted');g.lineWidth=1;g.beginPath();g.moveTo(x,T);g.lineTo(x,T+ph);g.stroke();
  if(r)for(const s of ser){g.fillStyle=css(s.c);g.strokeStyle=css('--surface');g.lineWidth=2;g.beginPath();g.arc(x,Y(val(r,s)),4,0,7);g.fill();g.stroke()}
  tip.innerHTML=`<div>${hhmm(tOf(i))}</div>`+(r?ser.map(s=>`<div><i style="display:inline-block;width:8px;height:8px;border-radius:50%;background:var(${s.c});margin-right:5px"></i>${s.n} <b>${val(r,s).toFixed(s.d||0)}</b></div>`).join(''):'<div>no data</div>');
  tip.style.display='block';const tw=tip.offsetWidth;tip.style.left=(x+12+tw>W?x-12-tw:x+12)+'px';tip.style.top=(cv.offsetTop+T)+'px';
}
function drawAll(){for(const id in SERIES)drawChart($(id));const n=hist?hist.data.length:0;
  $('histNote').textContent=n?`1-minute averages · ${Math.round(n/6)/10} h recorded since boot`:''}
for(const id in SERIES){const cv=$(id);const mv=e=>{const b=cv.getBoundingClientRect();drawChart(cv,e.clientX-b.left)};
  cv.addEventListener('pointermove',mv);cv.addEventListener('pointerdown',mv);cv.addEventListener('pointerleave',()=>drawChart(cv))}
$('range').onclick=e=>{const n=e.target.dataset.n;if(!n)return;range=+n;for(const b of $('range').children)b.classList.toggle('on',b===e.target);drawAll()};
addEventListener('resize',drawAll);
matchMedia('(prefers-color-scheme: dark)').addEventListener('change',drawAll);
poll();loadHist();setInterval(poll,5000);setInterval(loadHist,60000);
</script></body></html>)rawliteral";

static const char ADMIN_HTML[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>VE.Bus Settings</title><link rel="stylesheet" href="/style.css"></head><body><main>
<header><div><h1>Settings &amp; control</h1><div class="sub" id="sub">&nbsp;</div></div><span class="sp"></span>
<a class="btn" href="/">← Dashboard</a></header>
<div class="banner" id="pwBanner" style="display:none">You are using the default admin password <b>vebus</b>. Set a new one below.</div>

<section class="card"><h2>Control</h2>
<div class="grid2">
<label class="f">ESS setpoint (W, −1875…1875, negative = feed-in)<span class="row"><input type="number" id="ess" min="-1875" max="1875" step="1" style="width:120px"><button class="primary" onclick="cmd('ess_power',$('ess').value)">Set</button></span></label>
<label class="f">Switch state<span class="row"><select id="sw"><option value="on">On</option><option value="off">Off</option><option value="charger_only">Charger only</option><option value="inverter_only">Inverter only</option></select><button onclick="cmd('switch_state',$('sw').value)">Apply</button></span></label>
<label class="f">Battery-neutral UPS mode<span class="row"><select id="vm"><option value="OFF">Off</option><option value="ON">On</option></select><button onclick="cmd('virtual_mode',$('vm').value)">Apply</button></span></label>
</div>
<p class="note" id="ctlState"></p>
<div class="row"><button onclick="cmd('wakeup')">Wake up</button><button onclick="cmd('sleep')">Sleep</button>
<button onclick="cmd('force_absorption')">Force absorption</button><button onclick="cmd('force_float')">Force float</button>
<button onclick="confirm('Start equalisation charge? Only do this for batteries that support it.')&&cmd('force_equalise')">Force equalise</button></div>
</section>

<form class="card" id="cfg"><h2>MQTT &amp; device</h2><div class="grid2">
<label class="f">MQTT host<input name="host" required></label>
<label class="f">MQTT port<input name="port" type="number" min="1" max="65535"></label>
<label class="f">MQTT user<input name="user" autocomplete="off"></label>
<label class="f">MQTT password<input name="pass" type="password" autocomplete="new-password" id="mpass"></label>
<label class="f">HA device id<input name="device" required></label>
<label class="f">MQTT topic prefix<input name="prefix" required></label>
<label class="f">ESS fail-safe timeout (s, 0 = off)<input name="ess_timeout" type="number" min="0" max="65535"></label>
</div>
<p class="note">Fail-safe: if no new ESS setpoint arrives (MQTT, web or serial) within this time, the setpoint returns to 0 W — protects the battery if Home Assistant or the network goes down.</p>
<label class="note"><input type="checkbox" name="clear_pass" value="1"> remove stored MQTT password</label>
<h2 style="margin-top:16px">Network UPS (NUT)</h2>
<label class="note"><input type="checkbox" name="nut" value="1"> NUT server enabled (TCP port 3493)</label>
<div class="grid2" style="margin-top:8px">
<label class="f">UPS name<input name="nut_ups" maxlength="15" pattern="[A-Za-z0-9_.-]+"></label>
<label class="f">NUT user (empty = no login required)<input name="nut_user" autocomplete="off" maxlength="31"></label>
<label class="f">NUT password<input name="nut_pass" type="password" autocomplete="new-password" id="npass" maxlength="31"></label>
<label class="f">Low battery at SoC (%, 0 = LED only)<input name="low_soc" type="number" min="0" max="100"></label>
<label class="f">Nominal power (W, for load %, 0 = off)<input name="nominal_w" type="number" min="0" max="65535"></label>
</div>
<p class="note" id="nutHint">Clients (Synology, TrueNAS, Proxmox, upsmon, Home Assistant NUT) connect to <code>&lt;ups&gt;@&lt;device-ip&gt;</code>. Status is OL (mains) / OB (on battery); LB (low battery) is raised by the Multiplus low-battery LED or the SoC threshold — that is what triggers the shutdown on your clients.</p>
<h2 style="margin-top:16px">Admin password</h2><div class="grid2">
<label class="f">New password (user: admin)<input type="password" id="ap1" autocomplete="new-password" minlength="4" maxlength="31"></label>
<label class="f">Repeat<input type="password" id="ap2" autocomplete="new-password"></label></div>
<p class="note">Leave empty to keep the current password. Your browser will ask for the new one after saving.</p>
<div class="row" style="margin-top:8px"><button class="primary" type="submit">Save</button></div></form>

<section class="card"><h2>System</h2>
<label class="f">Firmware update (.bin from <code>.pio/build/mqtt_ha/firmware.bin</code>)<span class="row"><input type="file" id="fw" accept=".bin"><button id="fwBtn" onclick="upload()">Upload</button></span></label>
<progress id="prog" value="0" max="100" style="display:none;margin-top:8px"></progress>
<div class="row" style="margin-top:14px"><button onclick="confirm('Reboot now?')&&post('/admin/api/reboot').then(()=>toast('Rebooting…'))">Reboot</button>
<button class="danger" onclick="confirm('Forget WiFi credentials? The device will open the VEBus-Setup access point.')&&post('/admin/api/wifireset').then(()=>toast('WiFi reset — connect to VEBus-Setup'))">Reset WiFi</button></div>
<p class="note" id="sys"></p></section>
<div class="toast" id="toast"></div></main>
<script>
const $=id=>document.getElementById(id);
function toast(t){const e=$('toast');e.textContent=t;e.style.display='block';clearTimeout(e._t);e._t=setTimeout(()=>e.style.display='none',3000)}
async function post(url,body){const r=await fetch(url,{method:'POST',body:new URLSearchParams(body||{})});
  const j=await r.json().catch(()=>({ok:r.ok}));if(!r.ok||j.ok===false)throw new Error(j.error||('HTTP '+r.status));return j}
async function cmd(c,v){try{await post('/admin/api/control',{cmd:c,value:v??''});toast('Sent: '+c.replace('_',' '));setTimeout(state,1500)}catch(e){toast('Error: '+e.message)}}
async function state(){try{const s=await (await fetch('/api/state',{cache:'no-store'})).json();
  $('sub').textContent=`${s.device_id} · ${s.ip}`;$('pwBanner').style.display=s.default_pw?'block':'none';
  $('ctlState').textContent=`Current: setpoint ${s.ess_power} W (effective ${s.ess_power_eff} W) · switch ${s.switch_state} · battery-neutral ${s.virtual_mode} · state ${s.device_state}`;
  if(document.activeElement!==$('ess')&&$('ess').value==='')$('ess').value=s.ess_power;
  $('sw').value=s.switch_state;$('vm').value=s.virtual_mode;
  $('sys').textContent=`Uptime ${Math.floor(s.uptime/3600)} h · free heap ${(s.free_heap/1024).toFixed(0)} kB · last reset: ${s.reset_reason} · WiFi ${s.rssi} dBm`}catch(e){}}
async function loadCfg(){const c=await (await fetch('/admin/api/config',{cache:'no-store'})).json();const f=$('cfg');
  for(const k of ['host','port','user','device','prefix','ess_timeout','nut_ups','nut_user','low_soc','nominal_w'])f[k].value=c[k];
  f.nut.checked=c.nut;
  $('mpass').placeholder=c.pass_set?'•••••• (unchanged)':'(none)';
  $('npass').placeholder=c.nut_pass_set?'•••••• (unchanged)':'(none)';
  $('nutHint').querySelector('code').textContent=c.nut_ups+'@'+location.hostname}
$('cfg').onsubmit=async e=>{e.preventDefault();const f=new FormData(e.target);
  if($('ap1').value!==$('ap2').value){toast('Passwords do not match');return}
  if($('ap1').value)f.append('admin_pass',$('ap1').value);
  try{await post('/admin/api/config',f);toast('Saved — MQTT reconnecting');$('ap1').value=$('ap2').value='';f.get('clear_pass')&&(e.target.clear_pass.checked=false);loadCfg();state()}catch(err){toast('Error: '+err.message)}};
function upload(){const file=$('fw').files[0];if(!file){toast('Choose a .bin file first');return}
  const x=new XMLHttpRequest(),fd=new FormData(),p=$('prog');fd.append('firmware',file);
  x.upload.onprogress=e=>{p.style.display='block';p.value=e.loaded/e.total*100};
  x.onload=()=>{toast(x.status==200?'Update OK — rebooting…':'Update failed: '+x.responseText);$('fwBtn').disabled=false;if(x.status==200)setTimeout(()=>location.href='/',15000)};
  x.onerror=()=>{toast('Upload failed');$('fwBtn').disabled=false};
  $('fwBtn').disabled=true;x.open('POST','/admin/update');x.send(fd)}
loadCfg();state();setInterval(state,5000);
</script></body></html>)rawliteral";
