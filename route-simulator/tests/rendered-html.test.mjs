import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import test from "node:test";

async function render() {
  const workerUrl = new URL("../dist/server/index.js", import.meta.url);
  workerUrl.searchParams.set("test", `${process.pid}-${Date.now()}`);
  const { default: worker } = await import(workerUrl.href);

  return worker.fetch(
    new Request("http://localhost/", {
      headers: { accept: "text/html" },
    }),
    {
      ASSETS: {
        fetch: async () => new Response("Not found", { status: 404 }),
      },
    },
    {
      waitUntil() {},
      passThroughOnException() {},
    },
  );
}

test("server-renders the tmcode1 route simulator", async () => {
  const response = await render();
  assert.equal(response.status, 200);
  assert.match(response.headers.get("content-type") ?? "", /^text\/html\b/i);

  const html = await response.text();
  assert.match(html, /<html lang="zh-CN">/);
  assert.match(html, /<title>麦轮小车两轮路线动态仿真<\/title>/);
  assert.match(html, /场地实时俯视图/);
  assert.match(html, /车体长方体/);
  assert.match(html, /机械臂中心/);
  assert.match(html, /最终入库不转圈/);
  assert.match(html, /value="1050"/);
  assert.doesNotMatch(html, /Your site is taking shape|Building your site/);
});

test("keeps route geometry and controls tied to current tmcode1 values", async () => {
  const [page, css, layout] = await Promise.all([
    readFile(new URL("../app/page.tsx", import.meta.url), "utf8"),
    readFile(new URL("../app/globals.css", import.meta.url), "utf8"),
    readFile(new URL("../app/layout.tsx", import.meta.url), "utf8"),
  ]);

  assert.match(page, /const FIELD = 2400/);
  assert.match(page, /const START: Pose = \{ x: 2215, y: 2250, heading: 0 \}/);
  assert.match(page, /const ARM_OFFSET = 225/);
  assert.match(page, /addMove\(\s*1200,\s*2100/);
  assert.match(page, /addMove\(\s*1200,\s*300/);
  assert.match(page, /addMove\(\s*300,\s*1200/);
  assert.match(page, /addMove\(2150, 2250/);
  assert.match(page, /addMove\(2215, 2250/);
  assert.match(page, /直接平移入库/);
  assert.match(page, /type="range"/);
  assert.match(page, /setPlaying/);
  assert.match(css, /\.vehicle\s*\{/);
  assert.match(css, /width:\s*9\.5833%/);
  assert.match(css, /height:\s*12\.5%/);
  assert.match(layout, /route-preview\.png/);
});
