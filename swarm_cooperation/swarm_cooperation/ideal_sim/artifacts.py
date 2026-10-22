"""可复现JSON/CSV与无外部依赖的交互回放报告。"""
import csv
import json
from pathlib import Path


def write_artifacts(sim, directory):
    out = Path(directory)
    out.mkdir(parents=True, exist_ok=True)
    report = sim.report()
    (out / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    for name, records in (("events", sim.events), ("decisions", sim.decisions)):
        (out / f"{name}.jsonl").write_text("".join(json.dumps(row, ensure_ascii=False) + "\n" for row in records), encoding="utf-8")
    if sim.monitor.frames:
        with (out / "telemetry_simulation.jsonl").open("w", encoding="utf-8") as stream:
            for frame in sim.monitor.frames:
                stream.write(json.dumps(dict(frame, episode_id=f"sim-{sim.cfg.seed}",
                                             split_group=f"sim-{sim.cfg.seed}"), ensure_ascii=False) + "\n")
    with (out / "trace.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        if sim.samples:
            writer = csv.DictWriter(stream, fieldnames=list(sim.samples[0]))
            writer.writeheader()
            writer.writerows(sim.samples)
    data = json.dumps({"report": report, "samples": sim.samples, "events": sim.events}, ensure_ascii=False).replace("<", "\\u003c")
    html = HTML.replace("__DATA__", data)
    (out / "replay.html").write_text(html, encoding="utf-8")
    return out.resolve()


HTML = r'''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>巡检与中继调度回放</title><style>
body{margin:0;background:#0e1827;color:#e4edf9;font:15px system-ui,sans-serif}main{max-width:1100px;margin:auto;padding:28px}
h1{font-size:27px;margin:0 0 12px}p{color:#acbfd4;line-height:1.7}.bar,.cards{display:flex;gap:14px;flex-wrap:wrap;align-items:center}
button,select{padding:9px 14px;border:1px solid #43617c;border-radius:7px;background:#1d3148;color:white;cursor:pointer}
input{flex:1;min-width:200px}.maps{display:grid;grid-template-columns:1fr 1fr;gap:18px}canvas{width:100%;background:#15253a;border-radius:14px}
.card{padding:18px;background:#192b40;border-radius:12px;flex:1;margin:14px 0}.card b{font-size:22px;display:block;color:#6ed6be}
table{border-collapse:collapse;width:100%;font-size:14px}td,th{padding:9px;border-bottom:1px solid #30475e;text-align:left}
#events{max-height:220px;overflow:auto;font:13px monospace;line-height:1.8;background:#15253a;padding:14px}
@media(max-width:650px){.maps{grid-template-columns:1fr}main{padding:14px}}
</style><main><h1 id="heading">固定航线 · 巡检与中继调度</h1>
<p id="assumptions">算法仿真：左侧显示环境真值，右侧显示调度器已知地图。橙色为货箱，蓝色为主机，绿色为中继。</p>
<div class="cards" id="cards"></div><div class="bar"><button id="play">播放</button><select id="speed"><option value="1">1×</option><option value="5" selected>5×</option><option value="20">20×</option></select><input id="slider" type="range" min="0" value="0"><span id="time"></span></div>
<p id="state"></p><div class="maps"><section><h3>环境真值</h3><canvas id="truth" width="480" height="510"></canvas></section><section><h3>已观测地图</h3><canvas id="known" width="480" height="510"></canvas></section></div>
<h3>第二轮正向：同场景固定站位 / 自适应调度对照</h3><p>first_pass_fixed 为历史固定候选，不一定是实际首轮站位；frozen_map 为旧地图枚举；updated_map 为更新地图枚举。全部在同一变更地图、同一路径采样上评估，假设中继已到站。固定站位对照衡量调度收益，旧地图枚举对照单独衡量地图更新收益；完整五轮运动过程另见每段指标。指标依次为断链、弱链路、中继转发占用、移动距离、切换。</p><div id="comparison"></div>
<h3>实际执行的各单程</h3><div id="legs"></div><h3>当前时刻之前的事件</h3><div id="events"></div>
<p id="model-status"></p></main>
<script>const D=__DATA__, R=D.report, S=D.samples, E=D.events;
const el=id=>document.getElementById(id), slider=el('slider');slider.max=Math.max(0,S.length-1);
const cars=R.config.vehicle_kind==='ground_car';
el('heading').textContent=`固定航线 · ${R.config.rounds} 轮往返 · ${cars?'两台地面中继小车':'中继调度'}`;
el('assumptions').textContent=`算法仿真；地图模式：${R.config.map_observation}。${cars?'小车按有限速度沿地面路线移动，到位后确认路由；没有起飞或悬停稳定阶段。':'中继按有限速度移动，包含起飞、稳定就位及路由确认。'}static 模式不感知货箱移除；ideal 模式使用理想局部观测。`;
el('model-status').textContent=`策略：${R.config.strategy}；预测模型数据来源：${R.predictor_source}；推理调用：${R.inference_calls}，回退：${R.inference_fallbacks}。${R.success?'任务完成':'任务失败：'+R.failure_reason}。未验证 NPU 或真实飞行动力学。`;
el('cards').innerHTML=`<div class="card"><b>${R.completed_rounds} / ${R.config.rounds}</b>完整往返</div><div class="card"><b>${R.main_distance_m} m</b>主机航程</div><div class="card"><b>${R.removal?R.removal.cells.length:0} 块</b>${R.change_executed?'已执行移除':'未提交移除'}</div><div class="card"><b>${R.all_phase_outage_s} s</b>全阶段断链</div>`;
function table(headers,rows){return '<table><tr>'+headers.map(x=>'<th>'+x+'</th>').join('')+'</tr>'+rows.map(row=>'<tr>'+row.map(x=>'<td>'+x+'</td>').join('')+'</tr>').join('')+'</table>'}
const cmp=R.second_outbound;
el('comparison').innerHTML=cmp?table(['方案','站位','断链采样','弱链路采样','中继跳采样','移动/m','切换'],Object.entries(cmp.records).map(([k,v])=>[k,JSON.stringify(v.stations),v.outage_samples,v.weak_samples,v.relay_hop_samples,v.movement_m,v.switches]))+`<p>相对历史固定候选：${cmp.lexicographic_improvement?'改善':'持平或未改善'}；单独地图更新贡献：${cmp.map_only_improvement?'改善':'持平或未改善'}。采样间距 ${cmp.sample_spacing_cells} 格。</p>`:'尚未执行第二轮';
el('legs').innerHTML=table(['轮次','方向','航程/m','飞行/s','断链/s','弱链路/s','中继跳秒','切换','地图版本'],R.legs.map(v=>[v.round,v.direction,v.distance_m,v.flight_s,v.outage_s,v.weak_s,v.relay_hop_s,v.switches,v.map_version]));
function draw(id,removed,s){const c=el(id).getContext('2d');c.clearRect(0,0,480,510);const px=x=>65+x*110,py=y=>460-y*100;
for(let x=1;x<=3;x++)for(let y=1;y<=4;y++){const gone=removed.has(`${x},${y}`);c.fillStyle=gone?'#213950':'#b88141';c.fillRect(px(x-1)+7,py(y)+7,96,86);c.fillStyle=gone?'#7e9cb5':'#fff1df';c.font='14px sans-serif';c.fillText(`${x},${y}${gone?' 空':''}`,px(x-1)+22,py(y)+52)}
c.strokeStyle='#426380';c.lineWidth=1;for(let x=0;x<=3;x++){c.beginPath();c.moveTo(px(x),py(0));c.lineTo(px(x),py(4));c.stroke()}for(let y=0;y<=4;y++){c.beginPath();c.moveTo(px(0),py(y));c.lineTo(px(3),py(y));c.stroke()}
const route=R.contract.outward_route;c.strokeStyle='#4f8bad66';c.lineWidth=4;c.beginPath();route.forEach((p,i)=>i?c.lineTo(px(p[0]),py(p[1])):c.moveTo(px(p[0]),py(p[1])));c.stroke();
const points=[[0,0],[s.r1_x,s.r1_y],[s.r2_x,s.r2_y],[s.x,s.y]], active=[0,...s.route,3];c.strokeStyle=s.link_degraded?'#ef7373':'#6ed6be';c.lineWidth=2;c.setLineDash([6,5]);c.beginPath();active.forEach((k,i)=>i?c.lineTo(px(points[k][0]),py(points[k][1])):c.moveTo(px(points[k][0]),py(points[k][1])));c.stroke();c.setLineDash([]);
points.forEach((p,k)=>{c.fillStyle=['#e1eafa','#6ed6be','#c2e97d','#67b5ff'][k];c.beginPath();c.arc(px(p[0]),py(p[1]),k===3?9:7,0,Math.PI*2);c.fill();c.fillText(['GS','R1','R2','主机'][k],px(p[0])+10,py(p[1])-10)});}
function render(){const s=S[+slider.value];if(!s)return;const events=E.filter(e=>e.time_s<=s.time_s);const truth=new Set(),known=new Set();for(const e of events){if(e.kind==='boxes_removed')e.cells.forEach(c=>truth.add(c.join(',')));if(e.kind==='map_updated'){known.clear();for(let x=1;x<=3;x++)for(let y=1;y<=4;y++)if(!e.known_occupied.some(c=>c[0]===x&&c[1]===y))known.add(`${x},${y}`)}}draw('truth',truth,s);draw('known',known,s);el('time').textContent=s.time_s.toFixed(1)+' s';el('state').textContent=`第 ${s.round} 轮 · ${s.direction==='outbound'?'正向巡线':'反向巡线'} · ${s.phase} · 地图 v${s.map_version} · 物理 RSSI ${s.effective_rssi_dbm.toFixed(1)} dBm · ${s.link_degraded?'无有效观测路径（降级）':'观测路径可用'} · 中继 ${s.relay_status.join('/')} · ${s.window_open?'手动窗口开放':'窗口关闭'}`;el('events').textContent=events.slice(-14).map(e=>`${e.time_s.toFixed(1)}s ${e.kind} ${JSON.stringify(e)}`).join('\n')}
let playing=false,last=0,acc=0;el('play').onclick=()=>{playing=!playing;el('play').textContent=playing?'暂停':'播放';if(+slider.value>=S.length-1)slider.value=0};slider.oninput=render;
function frame(t){if(playing&&last){acc+=(t-last)/1000*(+el('speed').value);while(acc>=R.config.dt_s){slider.value=Math.min(S.length-1,+slider.value+1);acc-=R.config.dt_s;if(+slider.value>=S.length-1){playing=false;el('play').textContent='播放';break}}render()}last=t;requestAnimationFrame(frame)}render();requestAnimationFrame(frame);
</script></html>'''
