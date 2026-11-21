"use strict";
const $ = id => document.getElementById(id);
const integer = n => n.toLocaleString("ko-KR");
const kg = n => (n / 10).toLocaleString("ko-KR", { maximumFractionDigits: 1 });
const seconds = n => n ? `${(n / 1000).toFixed(2)} s` : "—";
let port = null, reader = null, readTask = null, stopping = false;
let report = null, signature = "", receivedAt = null, lastTraffic = 0;
const errorNames = [[1,"무게"],[2,"IMU"],[4,"보조 장치"],[8,"통신"],[16,"FSR"],
  [32,"최종 데이터 누락/기록 한도"],[64,"거리 센서"],[128,"보조 개입"],[256,"기준 높이"],[512,"보정"]];

function notice(text, error = false) {
  $("notice").textContent = text;
  $("notice").classList.toggle("error", error);
}
function advice(s) {
  if (!s.rec) return s.reps < 3 ? "3회 미만 · 다음 세트 추천 보류" : "측정 상태 확인 필요 · 추천 보류";
  return `${["", "무게 유지", "무게 감소", "무게 증가"][s.rec]} · 다음 세트 ${kg(s.nextWeight)} kg × ${s.nextReps}회`;
}
function render(r) {
  const data = { session: r.session, dropped: r.dropped, sets: r.sets.map(({cycle, ...s}) => s) };
  const next = JSON.stringify(data);
  notice("운동 결과 수신 완료. 마지막으로 완료한 운동을 표시합니다. 다음 운동도 거치 후 D를 누르면 갱신됩니다.");
  if (next === signature) return;
  signature = next; report = data; receivedAt = new Date();
  $("waiting").hidden = true; $("report").hidden = false;
  const sets = data.sets;
  const bad = sets.filter(s => s.invalid).length;
  $("report-meta").textContent = `운동 ${r.session} · ${receivedAt.toLocaleTimeString("ko-KR")} 수신${bad ? ` · 확인이 필요한 세트 ${bad}개 포함` : ""}`;
  $("total-sets").innerHTML = `${sets.length}<small>세트</small>`;
  $("total-reps").innerHTML = `${integer(sets.reduce((v,s) => v + s.reps, 0))}<small>회</small>`;
  $("total-volume").innerHTML = `${kg(sets.reduce((v,s) => v + s.weight * s.reps, 0))}<small>kg·회</small>`;
  $("assist-sets").innerHTML = `${sets.filter(s => s.assisted).length}<small>세트</small>`;
  $("overflow").hidden = !r.dropped;
  $("overflow").textContent = `STM2 저장 한도 32세트를 초과했습니다. 이후 ${r.dropped}세트는 이 리포트와 합계에 포함되지 않았습니다.`;
  const peak = Math.max(1, ...sets.map(s => s.reps));
  $("rep-chart").innerHTML = sets.map((s,i) => `<div class="bar-column" aria-label="${i+1}세트 ${s.reps}회${s.invalid ? ', 측정 상태 확인 필요' : ''}"><strong>${s.reps}</strong><div class="bar ${s.invalid ? 'bad' : ''}" style="--height:${Math.max(3, s.reps / peak * 105)}px"></div><span class="bar-label">${i+1}세트</span></div>`).join("");
  $("sets").innerHTML = sets.map((s,i) => {
    const status = s.assisted ? "보조 개입" : s.invalid ? "측정 상태 확인" : !s.reps ? "반복 없음" : s.reps < 3 ? "분석 자료 부족" : "측정 완료";
    const issues = errorNames.filter(([flag]) => s.invalid & flag).map(([,name]) => name);
    if (s.invalid & ~1023) issues.push("알 수 없는 상태");
    const cause = s.cause === 1 ? "페달" : s.cause >= 2 && s.cause <= 4 ? `FSR ${s.cause-1}` : "장치 상태";
    return `<article class="set"><div class="set-head"><h3>${i+1}세트</h3><span class="badge ${s.invalid || s.reps < 3 ? 'bad' : ''}">${status}</span></div>
      <div class="set-main">${kg(s.weight)} kg <span>×</span> ${s.reps}회</div>
      <dl class="details">
        <div><dt>평균 상승 시간</dt><dd>${seconds(s.avg)}</dd></div>
        <div><dt>마지막 상승 시간</dt><dd>${seconds(s.last)}</dd></div>
        <div><dt>최대 기울기</dt><dd>${s.invalid & (2|512) ? '—' : (s.tilt/10).toFixed(1)+'°'}</dd></div>
        <div><dt>상승 시간 증가율</dt><dd>${s.rec && s.reps >= 3 && s.avg ? s.timeIncrease+'%' : '—'}</dd></div>
        <div><dt>불안정 점수</dt><dd>${s.rec ? s.score+' / 100' : '분석 보류'}</dd></div>
        <div><dt>보조 개입</dt><dd>${s.assisted ? cause : '없음'}</dd></div>
      </dl><p class="advice">${advice(s)}</p>${issues.length ? `<p class="issues">확인 항목: ${issues.join(' · ')}</p>` : ''}</article>`;
  }).join("");
}
const receiver = new BenchReceiver(render);

async function readLoop(selected) {
  const decoder = new TextDecoder();
  try {
    reader = selected.readable.getReader();
    while (!stopping) {
      const { value, done } = await reader.read();
      if (done) break;
      if (value?.length) {
        lastTraffic = Date.now();
        $("connection-state").textContent = "USB 데이터 수신 중";
        receiver.feed(decoder.decode(value, {stream: true}));
      }
    }
    if (!stopping) notice("보드 연결이 종료되었습니다. 케이블을 확인하고 다시 연결하세요.", true);
  } catch {
    if (!stopping) notice("USB 수신이 중단되었습니다. 케이블과 전원을 확인한 뒤 다시 연결하세요. 표시된 결과는 유지됩니다.", true);
  } finally {
    if (reader) { reader.releaseLock(); reader = null; }
    await selected.close().catch(() => {});
    port = null; receiver.reset();
    $("connect").textContent = "STM2 연결"; $("connect").disabled = false;
    $("connection-state").textContent = "보드 연결 안 됨";
  }
}
$("connect").addEventListener("click", async () => {
  if (!navigator.serial || !window.isSecureContext) {
    notice("start.cmd로 실행한 localhost 화면을 Chrome 또는 Edge에서 열어주세요.", true); return;
  }
  $("connect").disabled = true;
  if (port) {
    stopping = true;
    if (reader) await reader.cancel().catch(() => {});
    await readTask;
    notice("연결을 해제했습니다. 표시된 결과는 유지됩니다.");
    return;
  }
  let selected;
  try {
    selected = await navigator.serial.requestPort();
    await selected.open({baudRate:115200, dataBits:8, stopBits:1, parity:"none", flowControl:"none"});
    port = selected; stopping = false; lastTraffic = Date.now(); receiver.reset();
    $("connection-state").textContent = "USB 연결됨 · 결과 대기";
    $("connect").textContent = "연결 해제"; $("connect").disabled = false;
    notice("연결되었습니다. 운동을 마치고 바를 거치한 뒤 키패드 D를 누르세요.");
    readTask = readLoop(selected);
  } catch (e) {
    if (selected) await selected.close().catch(() => {});
    notice(e.name === "NotFoundError" ? "포트 선택을 취소했습니다." : "포트를 열 수 없습니다. STM2의 ST-LINK 포트를 선택하고 다른 시리얼 모니터를 닫아주세요.", e.name !== "NotFoundError");
    $("connect").disabled = false;
  }
});
setInterval(() => {
  if (port && Date.now() - lastTraffic > 6000) $("connection-state").textContent = "수신 없음 · STM2 확인";
}, 1000);
$("download").addEventListener("click", () => {
  if (!report) return;
  const blob = new Blob([JSON.stringify({receivedAt:receivedAt.toISOString(), weightUnit:"0.1 kg", timeUnit:"ms", tiltUnit:"0.1 degree", ...report},null,2)], {type:"application/json"});
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a"); a.href=url; a.download=`smart-bench-${receivedAt.toISOString().replace(/[:.]/g,'-')}.json`; a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});
if (!navigator.serial || !window.isSecureContext) notice("이 화면의 USB 연결은 localhost에서 실행한 Chrome 또는 Edge를 사용해주세요.", true);
