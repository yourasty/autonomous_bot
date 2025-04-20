const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
  <head>
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <style>
      body {
        font-family: sans-serif;
        text-align: center;
        margin: 20px;
      }
      .mode-toggle button {
        padding: 10px 20px;
        margin: 0 10px;
        font-size: 16px;
        border: none;
        border-radius: 8px;
        background-color: #ccc;
        cursor: pointer;
        transition: background-color 0.2s;
      }
      .mode-toggle button.active {
        background-color: #4caf50;
        color: white;
      }
      .sensors {
        display: grid;
        grid-template-columns: 1fr 1fr 1fr;
        grid-template-rows: 1fr 1fr;
      }
      #logOutput {
        height: 250px;
        overflow-y: scroll;
      }
      #radarCanvas {
        display: block;
        margin: 0 auto 20px;
      }
    </style>
  </head>
  <body>
    <h2>RC Web Controller</h2>
    <div class="mode-toggle">
      <button id="btnManual" class="active" onclick='setMode("manual")'>
        Manual
      </button>
      <button id="btnAuto" onclick='setMode("auto")'>Autonomous</button>
    </div>
    <h3>Throttle: <span id="tVal">0</span></h3>
    <input id="throttle" type="range" min="-100" max="100" value="0" disabled />
    <h3>Steering: <span id="sVal">0</span></h3>
    <input id="steer" type="range" min="-55" max="55" value="0" disabled />
    <div class="sensors">
      <h3>Radar Sweep</h3>
      <canvas
        id="radarCanvas"
        width="300"
        height="300"
        style="border: 1px solid #ccc"
      ></canvas>

      <h3>Left</h3>
      <h3>Front</h3>
      <h3>Right</h3>
      <pre id="left"></pre>
      <pre id="front"></pre>
      <pre id="right"></pre>
    </div>
    <h3>Logs</h3>
    <pre id="logOutput"></pre>

    <script>
      const logMessages = {
        AUTO_FORWARD: "[AUTO] Clear path. Moving forward.",
        AUTO_REVERSE_LEFT: "[AUTO] Obstacle too close. Reversing left",
        AUTO_REVERSE_RIGHT: "[AUTO] Obstacle too close. Reversing right",
        AUTO_AVOID_LEFT: "[AUTO] Avoiding: Steer left",
        AUTO_AVOID_RIGHT: "[AUTO] Avoiding: Steer right",
        SENSOR_FAIL: "[FAIL] All sensors unresponsive. Rebooting.",
        MODE_SWITCH: "[INFO] Mode changed",
      };
      let ws;
      let lastPongTime = Date.now();
      let reconnectDelay = 500; // Delay before attempting reconnect
      let pingInterval = 5000; // Interval between pings

      function connectWebSocket() {
        ws = new WebSocket("ws://" + location.hostname + ":81");

        ws.onopen = () => {
          console.log("[WS] Connected");
          lastPongTime = Date.now();
        };

        ws.onmessage = (event) => {
          try {
            const obj = JSON.parse(event.data);

            if (obj.type === "pong") {
              lastPongTime = Date.now();
              return;
            } else if (obj.type === "telemetry") {
              if (obj.left !== undefined)
                document.getElementById("left").innerText = obj.left;
              if (obj.front !== undefined)
                document.getElementById("front").innerText = obj.front;
              if (obj.right !== undefined)
                document.getElementById("right").innerText = obj.right;

              if (obj.logs && Array.isArray(obj.logs)) {
                const logBox = document.getElementById("logOutput");
                obj.logs.forEach(({ code, msg }) => {
                  const logText = logMessages[code] || msg || code;
                  logBox.innerText += "\n" + logText;
                });
                logBox.scrollTop = logBox.scrollHeight;
              }
              if (Array.isArray(obj.radar)) {
                drawRadarSweep(obj.radar);
              }
            }
          } catch (e) {
            console.warn("[WS] Invalid JSON received", e);
            // Optionally handle non-JSON fallback if needed
          }
        };

        ws.onclose = () => {
          console.warn("[WS] Connection closed. Attempting reconnect...");
          setTimeout(connectWebSocket, reconnectDelay);
        };

        ws.onerror = (err) => {
          console.error("[WS] Error:", err);
          ws.close();
        };
      }

      connectWebSocket();

      // Heartbeat / Ping system
      setInterval(() => {
        if (ws && ws.readyState === WebSocket.OPEN) {
          ws.send(JSON.stringify({ type: "ping" }));

          // If no response in 2s, force reconnect
          if (Date.now() - lastPongTime > pingInterval) {
            console.warn("[WS] No pong. Reconnecting...");
            ws.close();
          }
        }
      }, pingInterval);

      function setMode(mode) {
        ws.send(JSON.stringify({ type: "mode", value: mode }));
        updateModeUI(mode);
      }

      function updateModeUI(mode) {
        document.getElementById("btnManual").classList.remove("active");
        document.getElementById("btnAuto").classList.remove("active");
        document
          .getElementById("btn" + (mode === "manual" ? "Manual" : "Auto"))
          .classList.add("active");
      }

      let lastThrottle = 0;
      let lastSteering = 0;

      setInterval(() => {
        const gp = navigator.getGamepads()[0];
        if (!gp) return;
        const throttle = Math.round(-gp.axes[3] * 100);
        const steering = Math.round(gp.axes[0] * 55);

        document.getElementById("tVal").innerText = throttle;
        document.getElementById("sVal").innerText = steering;

        if (
          Math.abs(throttle - lastThrottle) > 1 ||
          Math.abs(steering - lastSteering) > 1
        ) {
          ws.send(
            JSON.stringify({
              type: "input",
              throttle: throttle,
              steering: steering,
            })
          );

          lastThrottle = throttle;
          lastSteering = steering;
        }
      }, 50);

      function drawRadarSweep(radarPoints) {
        const canvas = document.getElementById("radarCanvas");
        if (!canvas || !radarPoints) return;
        const ctx = canvas.getContext("2d");

        const width = canvas.width;
        const height = canvas.height;
        const cx = width / 2;
        const cy = height;
        const radius = Math.min(cx, cy) - 10;

        ctx.clearRect(0, 0, width, height);

        // Background arc
        ctx.beginPath();
        ctx.arc(cx, cy, radius, Math.PI, 2 * Math.PI);
        ctx.strokeStyle = "#ccc";
        ctx.stroke();

        // Draw radar sweep area
        ctx.beginPath();
        ctx.moveTo(cx, cy);

        radarPoints.forEach((pt, i) => {
          const angleDeg = pt.angle;
          const distance = Math.min(pt.distance, 1000);
          const angleRad = (angleDeg * Math.PI) / 180;
          const relAngle = angleRad - Math.PI / 2 - Math.PI / 2; // shift so 90° is up
          const r = (distance / 1000) * radius;
          const x = cx + r * Math.cos(relAngle);
          const y = cy + r * Math.sin(relAngle);
          ctx.lineTo(x, y);
        });

        ctx.closePath();
        ctx.fillStyle = "rgba(0,255,0,0.2)";
        ctx.fill();
        ctx.strokeStyle = "#0f0";
        ctx.stroke();
      }
    </script>
  </body>
</html>

)rawliteral";