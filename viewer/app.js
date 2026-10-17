"use strict";

const D = window.REFSORS;
const $ = (id) => document.getElementById(id);

function esc(s) {
  return String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[c]);
}

function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

// ------------------------------------------------------------ markdown (README subset)

function inline(text) {
  return esc(text)
    .replace(/`([^`]+)`/g, "<code>$1</code>")
    .replace(/\[([^\]]+)\]\(([^)]+)\)/g, (m, label, href) =>
      /^https?:/.test(href) ? `<a href="${href}">${label}</a>` : `<code>${label}</code>`);
}

function markdown(src) {
  const out = [];
  const blocks = src.split(/\n\s*\n/);
  for (const block of blocks) {
    const lines = block.split("\n");
    if (/^\|/.test(lines[0])) {
      const cells = (l) => l.replace(/^\||\|$/g, "").split("|").map((c) => c.trim());
      const head = cells(lines[0]).map((c) => `<th>${inline(c)}</th>`).join("");
      const rows = lines.slice(2).map((l) => `<tr>${cells(l).map((c) => `<td>${inline(c)}</td>`).join("")}</tr>`).join("");
      out.push(`<div class="scroll"><table><thead><tr>${head}</tr></thead><tbody>${rows}</tbody></table></div>`);
    } else if (/^\d+\. /.test(lines[0])) {
      const items = block.split(/\n(?=\d+\. )/).map((it) => `<li>${inline(it.replace(/^\d+\. /, "").replace(/\s*\n\s*/g, " "))}</li>`);
      out.push(`<ol>${items.join("")}</ol>`);
    } else {
      out.push(`<p>${inline(lines.join(" "))}</p>`);
    }
  }
  return out.join("");
}

// ------------------------------------------------------------ overview

function renderOverview() {
  const r = D.readme;
  $("title").textContent = r.title;
  $("overview-intro").innerHTML = markdown(r.intro);
  $("overview-mission").innerHTML = markdown(r.Mission);
  $("overview-services").innerHTML = markdown(r.Services);

  const rows = D.setup.robots.map((b) => `
    <tr>
      <td><code>${esc(b.ns)}</code></td>
      <td><code>${esc(b.model)}</code></td>
      <td>${b.includes.map((i) => `<code>${esc(i.replace("$(find ", "").replace(")", ""))}</code>`).join("<br>")}</td>
      <td><code>${esc(b.scan_topic)}</code></td>
      <td><code>${esc(b.camera_topic)}</code></td>
      <td>${b.spawn ? `x ${b.spawn.x}, y ${b.spawn.y} <small>(${esc(b.launch)})</small>` : '<span class="none">package default, not set in this repository</span>'}</td>
    </tr>`).join("");
  $("robots").innerHTML = `<thead><tr><th>Namespace</th><th>Gazebo model</th><th>Spawned by</th><th>Laser topic</th><th>Camera topic</th><th>Start pose</th></tr></thead><tbody>${rows}</tbody>`;
}

// ------------------------------------------------------------ architecture

let selectedNode = null;

function renderNodes() {
  const groups = [["larger_robot", "larger_robot (Husky)"], ["", "Shared"], ["smaller_robot", "smaller_robot (Jackal)"]];
  $("nodes").innerHTML = groups.map(([ns, label]) => `
    <div><h4>${esc(label)}</h4>
      ${D.architecture.nodes.filter((n) => n.ns === ns).map((n) => `
        <button class="node" data-id="${esc(n.id)}" aria-pressed="false">
          ${esc(n.name)}<small>${esc(n.id)} · ${n.sources.map(esc).join(", ")}</small>
        </button>`).join("")}
    </div>`).join("");
  $("nodes").addEventListener("click", (e) => {
    const btn = e.target.closest(".node");
    if (!btn) return;
    selectedNode = selectedNode === btn.dataset.id ? null : btn.dataset.id;
    document.querySelectorAll(".node").forEach((b) => b.setAttribute("aria-pressed", String(b.dataset.id === selectedNode)));
    renderEndpoints();
  });
}

function refList(refs, empty) {
  if (!refs.length) return `<span class="none">${empty}</span>`;
  return refs.map((r) => `<code>${esc(r.node)}</code>`).join("<br>");
}

function renderEndpoints() {
  const eps = D.architecture.endpoints.filter((e) =>
    !selectedNode || e.providers.concat(e.users).some((r) => r.node === selectedNode));
  const noProvider = { topic: "published outside this package", service: "none in this package", action: "none in this package" };
  const rows = eps.map((e) => {
    const all = e.providers.concat(e.users);
    const role = selectedNode
      ? (e.providers.some((r) => r.node === selectedNode) ? '<span class="role provides">provides</span>' : '<span class="role">uses</span>')
      : "";
    const type = D.architecture.interfaces[e.type] ? `<a href="#if-${esc(e.type)}">${esc(e.type)}</a>` : esc(e.type || "?");
    const refs = [...new Set(all.map((r) => `${r.file}:${r.line}`))].map(esc).join(", ");
    return `<tr>
      <td>${esc(e.kind)} ${role}</td>
      <td><code>${esc(e.name)}</code><br><small>${type}</small><br><small class="refs">${refs}</small></td>
      <td>${refList(e.providers, noProvider[e.kind])}</td>
      <td>${refList(e.users, e.kind === "topic" ? "no subscriber in this package" : "no caller in this package")}</td>
    </tr>`;
  }).join("");
  $("endpoints").innerHTML = `<thead><tr><th>Kind</th><th>Name and type</th><th>Provided by</th><th>Used by</th></tr></thead><tbody>${rows}</tbody>`;
}

function renderInterfaces() {
  const items = Object.entries(D.architecture.interfaces).map(([name, it]) => `
    <details id="if-${esc(name)}">
      <summary>${esc(name)} <small>.${esc(it.kind)}</small></summary>
      ${it.sections.map((s) => `<div class="sec">${esc(s.label)}</div>
        ${s.fields.length ? `<ul>${s.fields.map((f) => `<li>${esc(f.type)} ${esc(f.name)}</li>`).join("")}</ul>` : '<div class="none">empty</div>'}`).join("")}
      <div class="sec"><code>${esc(it.file)}</code></div>
    </details>`).join("");
  $("interfaces").innerHTML = items;
  window.addEventListener("hashchange", openTargetInterface);
  openTargetInterface();
}

function openTargetInterface() {
  const el = location.hash.startsWith("#if-") && document.querySelector(location.hash);
  if (el) el.open = true;
}

// ------------------------------------------------------------ maps

const CHAR_COLORS = { " ": "--free", "0": "--free", ":": "--unknown", "#": "--occupied", "1": "--occupied", "x": "--path", "X": "--kept" };
let currentMap = 0;

function renderMapTabs() {
  $("map-tabs").innerHTML = D.maps.map((m, i) =>
    `<button role="tab" data-i="${i}" aria-selected="${i === currentMap}">${esc(m.file.replace("maps/", ""))}</button>`).join("");
  $("map-tabs").addEventListener("click", (e) => {
    const b = e.target.closest("button");
    if (!b) return;
    currentMap = Number(b.dataset.i);
    document.querySelectorAll("#map-tabs button").forEach((x) => x.setAttribute("aria-selected", String(x === b)));
    drawMap();
  });
}

function mix(a, b, t) {
  const pa = a.match(/\w\w/g).map((h) => parseInt(h, 16));
  const pb = b.match(/\w\w/g).map((h) => parseInt(h, 16));
  return `rgb(${pa.map((v, i) => Math.round(v + (pb[i] - v) * t)).join(",")})`;
}

function drawMap() {
  const m = D.maps[currentMap];
  const [rows, cols] = m.size;
  const cell = Math.max(1, Math.floor(480 / Math.max(rows, cols)));
  const canvas = $("map-canvas");
  canvas.width = cols * cell;
  canvas.height = rows * cell;
  const ctx = canvas.getContext("2d");
  const free = cssVar("--free");
  const occ = cssVar("--occupied");
  const max = m.kind === "numeric" ? Math.max(1, ...m.rows.flat()) : 1;
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      if (m.kind === "numeric") {
        const v = m.rows[r][c] || 0;
        ctx.fillStyle = v ? mix(free, occ, 0.15 + 0.85 * v / max) : free;
      } else {
        const ch = m.rows[r][c] === undefined ? " " : m.rows[r][c];
        ctx.fillStyle = cssVar(CHAR_COLORS[ch] || "--accent");
      }
      ctx.fillRect(c * cell, r * cell, cell, cell);
    }
  }

  let legend;
  if (m.kind === "numeric") {
    legend = `<ul class="legend"><li><span class="ramp" style="background:linear-gradient(90deg, ${free}, ${occ})"></span></li>
      <li>empty cell = 0, darkest = ${max} (largest value in the file)</li></ul>`;
  } else {
    legend = `<ul class="legend">${Object.entries(m.legend).map(([ch, label]) =>
      `<li><span class="swatch" style="background:var(${CHAR_COLORS[ch]})"></span><code>${esc(JSON.stringify(ch))}</code> ${esc(label)}</li>`).join("")}</ul>`;
  }
  $("map-info").innerHTML = `
    <h3 style="margin-top:0">${esc(m.title)}</h3>
    <dl>
      <dt>File</dt><dd><code>${esc(m.file)}</code></dd>
      <dt>Written by</dt><dd>${esc(m.writer)}</dd>
      <dt>Last commit</dt><dd><code>${esc(m.commit.sha)}</code> ${esc(m.commit.date)}</dd>
      <dt>Grid</dt><dd>${rows} × ${cols} cells</dd>
      <dt>Cell</dt><dd>${m.cell_m} m</dd>
    </dl>${legend}`;
}

// ------------------------------------------------------------ world

function renderWorld() {
  const w = D.world;
  const robots = D.setup.robots;
  $("world-note").innerHTML = `Top-down view of <code>${esc(w.file)}</code>, the world <code>launch/master.launch</code> loads.
    Boxes and cylinders are drawn from their visual geometry${w.uses_state ? ", at the poses saved in the file's <code>&lt;state&gt;</code>, which Gazebo applies on load" : ""}.
    x points right and y up, in meters.`;
  const half = 21;
  const parts = [];
  for (let v = -20; v <= 20; v += 5) {
    parts.push(`<line class="grid" x1="${v}" y1="${-half}" x2="${v}" y2="${half}"/><line class="grid" x1="${-half}" y1="${v}" x2="${half}" y2="${v}"/>`);
    parts.push(`<text class="axis" x="${v}" y="${half + 1.2}" text-anchor="middle">${v}</text>`);
    parts.push(`<text class="axis" x="${-half - 0.4}" y="${-v + 0.35}" text-anchor="end">${v}</text>`);
  }
  for (const s of w.shapes) {
    // SVG y grows downward, so world y is negated and rotations flip sign.
    const deg = -s.yaw * 180 / Math.PI;
    const title = `<title>${esc(s.model)} (${s.x}, ${s.y})</title>`;
    if (s.shape_kind === "box") {
      parts.push(`<rect class="box" x="${-s.sx / 2}" y="${-s.sy / 2}" width="${s.sx}" height="${s.sy}" transform="translate(${s.x} ${-s.y}) rotate(${deg})">${title}</rect>`);
    } else {
      const cls = s.model === "bomb" ? "target" : "box";
      parts.push(`<circle class="${cls}" cx="${s.x}" cy="${-s.y}" r="${Math.max(s.r, 0.35)}">${title}</circle>`);
      if (s.model === "bomb") parts.push(`<text class="label" x="${s.x + 0.8}" y="${-s.y + 0.4}">target (bomb)</text>`);
    }
  }
  for (const b of robots.filter((r) => r.spawn)) {
    parts.push(`<circle class="spawn" cx="${b.spawn.x}" cy="${-b.spawn.y}" r="0.6"><title>${esc(b.model)} spawn, ${esc(b.launch)}</title></circle>`);
    parts.push(`<text class="label" x="${b.spawn.x + 0.9}" y="${-b.spawn.y + 0.4}">${esc(b.model)} start</text>`);
  }
  const svg = $("world-svg");
  svg.setAttribute("viewBox", `${-half - 2.5} ${-half - 0.5} ${2 * half + 3} ${2 * half + 2.5}`);
  svg.innerHTML = parts.join("");
}

// ------------------------------------------------------------ history

function renderHistory() {
  $("commits").innerHTML = D.history.map((c) => `
    <li><time datetime="${esc(c.date)}">${esc(c.date.slice(0, 10))}</time><code>${esc(c.sha)}</code>${esc(c.subject)}</li>`).join("");
}

renderOverview();
renderNodes();
renderEndpoints();
renderInterfaces();
renderMapTabs();
drawMap();
renderWorld();
renderHistory();
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", drawMap);
