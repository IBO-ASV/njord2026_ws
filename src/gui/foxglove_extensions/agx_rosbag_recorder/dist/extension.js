const SERVICE = "/agx/rosbag/set_recording";
const STATUS = "/agx/rosbag/recording_status";

function initPanel(context) {
  const root = context.panelElement;
  root.replaceChildren();
  root.style.cssText = "height:100%;box-sizing:border-box;padding:18px;background:#101820;color:#e5edf5;font:14px system-ui,sans-serif";
  const title = document.createElement("div");
  title.textContent = "AGX ROSBAG RECORDING";
  title.style.cssText = "font-weight:700;letter-spacing:.06em;color:#a9c7e8;margin-bottom:14px";
  const state = document.createElement("div");
  state.textContent = "AGX status waiting…";
  state.style.cssText = "font-size:16px;margin-bottom:8px";
  const path = document.createElement("div");
  path.style.cssText = "color:#b6c2d0;word-break:break-all;margin-bottom:8px";
  const details = document.createElement("div");
  details.style.cssText = "color:#94a3b8;margin-bottom:14px";
  const button = document.createElement("button");
  button.textContent = "録画開始";
  button.disabled = true;
  button.style.cssText = "border:0;border-radius:5px;padding:10px 18px;background:#7c3aed;color:white;font-size:15px;font-weight:700;cursor:pointer";
  const feedback = document.createElement("div");
  feedback.style.cssText = "margin-top:10px;color:#fbbf24;min-height:1.4em";
  root.append(title,state,path,details,button,feedback);
  let latest = null;
  let busy = false;
  function render() {
    const current = latest?.state || "WAITING";
    state.textContent = current === "RECORDING" ? "● 録画中" : current === "STARTING" ? "開始処理中…" : current === "STOPPING" ? "保存処理中…" : current === "ERROR" ? "エラー" : "待機中";
    state.style.color = current === "RECORDING" ? "#4ade80" : current === "ERROR" ? "#f87171" : "#e5edf5";
    path.textContent = latest?.path ? `保存先: ${latest.path}` : "保存先: /home/ibo/rosbags/agx_日時";
    const elapsed = Number(latest?.elapsed_sec || 0);
    details.textContent = latest ? `${latest.message || ""} · 空き ${latest.free_gb ?? "--"} GiB${elapsed ? ` · ${Math.floor(elapsed/60)}分${elapsed%60}秒` : ""}` : "AGXとのライブ接続を待っています";
    const recording = current === "RECORDING" || current === "STARTING" || current === "STOPPING";
    button.textContent = recording ? "録画停止" : "録画開始";
    button.style.background = recording ? "#dc2626" : "#7c3aed";
    button.disabled = busy || !latest || current === "STARTING" || current === "STOPPING" || (current === "ERROR" && recording);
    button.style.opacity = button.disabled ? ".55" : "1";
  }
  button.onclick = async () => {
    if (busy || !latest) return;
    const start = !["RECORDING", "STARTING", "STOPPING"].includes(latest.state);
    busy = true; feedback.textContent = start ? "AGXへ録画開始を要求しています…" : "録画を安全に終了しています…"; render();
    try {
      const result = await context.callService(SERVICE, {data: start});
      feedback.textContent = result?.message || (result?.success === false ? "操作に失敗しました" : "要求を受け付けました");
    } catch (error) {
      feedback.textContent = `サービス呼び出し失敗: ${error?.message || error}`;
    } finally { busy = false; render(); }
  };
  context.subscribe([{topic: STATUS}]);
  context.watch("currentFrame");
  context.onRender = (renderState, done) => {
    try {
      for (const event of renderState.currentFrame || []) {
        if (event.topic !== STATUS) continue;
        const raw = event.message?.data;
        try { latest = typeof raw === "string" ? JSON.parse(raw) : raw; } catch { latest = null; }
        render();
      }
    } finally { done(); }
  };
  render();
  return () => root.replaceChildren();
}

function activate(extensionContext) {
  extensionContext.registerPanel({name: "agx-rosbag-recorder", initPanel});
}

module.exports = {activate};
