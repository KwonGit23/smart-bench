/* Newline framed STM2 reports. Commit only a complete, matching transaction. */
"use strict";
class BenchReceiver {
  constructor(onReport) { this.onReport = onReport; this.reset(); }
  reset() { this.buffer = ""; this.pending = null; this.discard = false; }
  feed(text) {
    for (const part of text.split(/(?<=\n)/)) {
      if (!part) continue;
      const ended = part.endsWith("\n");
      if (!this.discard) {
        this.buffer += part;
        if (this.buffer.length > 1024) { this.buffer = ""; this.discard = true; }
      }
      if (ended) {
        if (!this.discard) this.line(this.buffer.trim());
        this.buffer = ""; this.discard = false;
      }
    }
  }
  line(line) {
    if (!line.startsWith("@BENCH ")) return;
    let p;
    try { p = JSON.parse(line.slice(7)); } catch { this.pending = null; return; }
    const uint = (n, max = 0xffffffff) => Number.isInteger(n) && n >= 0 && n <= max;
    if (!p || p.v !== 1 || !uint(p.session) || !p.session || !uint(p.cycle) || !p.cycle) return;
    if (p.type === "begin") {
      this.pending = uint(p.count, 32) && uint(p.dropped)
        ? { session: p.session, cycle: p.cycle, count: p.count, dropped: p.dropped, sets: [] } : null;
      return;
    }
    const r = this.pending;
    if (!r || p.session !== r.session || p.cycle !== r.cycle) return;
    if (p.type === "set") {
      const fields = ["id", "weight", "reps", "avg", "last", "tilt", "invalid", "cause", "assisted", "score", "timeIncrease", "rec", "nextWeight", "nextReps"];
      if (p.index !== r.sets.length + 1 || p.index > r.count || fields.some(k => !uint(p[k])) ||
          !p.id || p.weight > 9999 || p.reps > 65535 || p.assisted > 1 || p.score > 100 || p.rec > 3 ||
          (r.sets.length && p.id <= r.sets.at(-1).id)) { this.pending = null; return; }
      r.sets.push(p);
    } else if (p.type === "end") {
      this.pending = null;
      if (p.count === r.count && r.sets.length === r.count) this.onReport(r);
    }
  }
}
globalThis.BenchReceiver = BenchReceiver;
