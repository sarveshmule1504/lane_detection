/*
 * ============================================================
 *   ESP32-CAM LANE DETECTION  —  Standalone (No Pico UART)
 * ============================================================
 *
 * Captures grayscale frames, detects two lane lines,
 * computes steering direction, and serves a live web
 * dashboard with annotated video stream.
 *
 * HARDWARE:  AI-Thinker ESP32-CAM (OV2640)
 *
 * ARDUINO IDE SETUP:
 *   1. File → Preferences → Additional Boards Manager URLs:
 *      https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 *   2. Tools → Board → ESP32 Arduino → "AI Thinker ESP32-CAM"
 *   3. Tools → PSRAM   → "Enabled"
 *   4. Tools → Partition Scheme → "Huge APP (3MB No OTA/1MB SPIFFS)"
 *
 * WIRING (for programming via FTDI/USB-Serial):
 *   FTDI TX  →  ESP32-CAM U0R (GPIO 3)
 *   FTDI RX  →  ESP32-CAM U0T (GPIO 1)
 *   FTDI GND →  ESP32-CAM GND
 *   GPIO 0   →  GND    ← pull LOW before upload, release after
 *   5V       →  5V
 *
 * USAGE:
 *   After upload, open Serial Monitor (115200 baud).
 *   Connect to the printed IP address in a browser.
 *   Adjust threshold/settings on the web dashboard.
 */

#include <Arduino.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "img_converters.h"
#include <WiFi.h>

// ======================== USER CONFIG ========================
// *** CHANGE THESE to your WiFi credentials! ***
const char* WIFI_SSID     = "Agastyas PG_201";
const char* WIFI_PASSWORD = "Agastyas.pg@123";

// If WiFi fails, ESP32-CAM creates its own Access Point:
const char* AP_SSID     = "ESP32-CAM-Lane";
const char* AP_PASSWORD = "12345678";

// ======================== CAMERA PINS (AI-Thinker) ========================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
#define FLASH_GPIO_NUM     4

// ======================== LANE DETECTION DEFAULTS ========================
// All adjustable at runtime via the web dashboard
int g_threshold   = 120;    // Brightness threshold (0–255)
int g_roiTopPct   = 50;     // ROI starts at this % from top (bottom half = road)
int g_scanRow1Pct = 70;     // First scan row (% within ROI)
int g_scanRow2Pct = 90;     // Second scan row (% within ROI)
int g_deadZone    = 20;     // Pixel offset tolerance for "CENTER"
int g_minPixels   = 8;      // Minimum bright pixels to count as a lane line
bool g_invert     = false;  // true = detect DARK lines on LIGHT background

// ======================== GLOBAL STATE ========================
struct LaneResult {
    int leftPos;        // Left lane line X position (averaged)
    int rightPos;       // Right lane line X position (averaged)
    int center;         // Computed lane center X
    int offset;         // Pixel offset from frame center (neg=left, pos=right)
    char dir[8];        // "LEFT", "RIGHT", "CENTER", or "NONE"
    bool leftOK;        // Left line detected?
    bool rightOK;       // Right line detected?
};

LaneResult g_lane = { -1, -1, -1, 0, "NONE", false, false };

httpd_handle_t stream_httpd  = NULL;
httpd_handle_t control_httpd = NULL;
volatile bool  g_streaming   = false;
bool  g_flashOn    = false;
float g_fps        = 0;
unsigned long g_lastFpsCalc = 0;
int   g_frameCount = 0;

// ======================== CAMERA INIT ========================
bool initCamera() {
    camera_config_t cfg;
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.pin_d0       = Y2_GPIO_NUM;
    cfg.pin_d1       = Y3_GPIO_NUM;
    cfg.pin_d2       = Y4_GPIO_NUM;
    cfg.pin_d3       = Y5_GPIO_NUM;
    cfg.pin_d4       = Y6_GPIO_NUM;
    cfg.pin_d5       = Y7_GPIO_NUM;
    cfg.pin_d6       = Y8_GPIO_NUM;
    cfg.pin_d7       = Y9_GPIO_NUM;
    cfg.pin_xclk     = XCLK_GPIO_NUM;
    cfg.pin_pclk     = PCLK_GPIO_NUM;
    cfg.pin_vsync    = VSYNC_GPIO_NUM;
    cfg.pin_href     = HREF_GPIO_NUM;
    cfg.pin_sccb_sda = SIOD_GPIO_NUM;
    cfg.pin_sccb_scl = SIOC_GPIO_NUM;
    cfg.pin_pwdn     = PWDN_GPIO_NUM;
    cfg.pin_reset    = RESET_GPIO_NUM;
    cfg.xclk_freq_hz = 20000000;
    cfg.pixel_format = PIXFORMAT_GRAYSCALE;   // 1 byte/pixel – fastest for processing
    cfg.frame_size   = FRAMESIZE_QVGA;        // 320 × 240
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 2;                     // Double buffer for lower latency
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;    // Always get the newest frame

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("Camera init FAILED: 0x%x\n", err);
        return false;
    }

    // Tweak sensor for lane detection (higher contrast helps)
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 0);
        s->set_contrast(s, 1);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_aec2(s, 1);
        s->set_gain_ctrl(s, 1);
    }

    Serial.println("[CAM] Initialized: GRAYSCALE 320x240");
    return true;
}

// ======================== WIFI INIT ========================
void initWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[WiFi] Connecting to '%s'", WIFI_SSID);

    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 20) {
        delay(500);
        Serial.print(".");
        tries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        WiFi.setSleep(false); // Disable WiFi power save to fix massive stream lag
        Serial.printf("\n[WiFi] Connected!  IP: http://%s\n", WiFi.localIP().toString().c_str());
        return;
    }

    // Fallback → create Access Point
    Serial.printf("\n[WiFi] Failed! Creating AP: '%s' / '%s'\n", AP_SSID, AP_PASSWORD);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.printf("[WiFi] AP IP: http://%s\n", WiFi.softAPIP().toString().c_str());
}

// ======================== DRAWING HELPERS ========================
inline void setPixel(uint8_t* buf, int w, int h, int x, int y, uint8_t v) {
    if (x >= 0 && x < w && y >= 0 && y < h)
        buf[y * w + x] = v;
}

void drawHLine(uint8_t* buf, int w, int h, int x1, int x2, int y, uint8_t v) {
    if (x1 < 0) x1 = 0;
    if (x2 >= w) x2 = w - 1;
    for (int x = x1; x <= x2; x++) setPixel(buf, w, h, x, y, v);
}

void drawVLine(uint8_t* buf, int w, int h, int x, int y1, int y2, uint8_t v) {
    if (y1 < 0) y1 = 0;
    if (y2 >= h) y2 = h - 1;
    for (int y = y1; y <= y2; y++) setPixel(buf, w, h, x, y, v);
}

// Draw a thick cross marker at (cx, cy)
void drawCross(uint8_t* buf, int w, int h, int cx, int cy, int size, uint8_t v) {
    for (int t = -1; t <= 1; t++) {
        drawHLine(buf, w, h, cx - size, cx + size, cy + t, v);
        drawVLine(buf, w, h, cx + t, cy - size, cy + size, v);
    }
}

// ======================== LANE DETECTION ========================

/*
 * Scans a single horizontal row of the grayscale image.
 * Divides the row into left half and right half.
 * Finds the centroid (average X) of all "lane-line" pixels in each half.
 *
 * This centroid approach is robust to noise because the lane line
 * contributes many more pixels than random bright spots.
 */
void scanRow(uint8_t* buf, int w, int y,
             int threshold, bool invert, int minPx,
             int* leftPos, int* rightPos)
{
    int half = w / 2;

    // ---- Left half: centroid of lane-line pixels ----
    long sumX = 0;
    int count = 0;
    for (int x = 0; x < half; x++) {
        uint8_t px = buf[y * w + x];
        bool isLine = invert ? (px < threshold) : (px > threshold);
        if (isLine) { sumX += x; count++; }
    }
    *leftPos = (count >= minPx) ? (int)(sumX / count) : -1;

    // ---- Right half: centroid of lane-line pixels ----
    sumX = 0; count = 0;
    for (int x = half; x < w; x++) {
        uint8_t px = buf[y * w + x];
        bool isLine = invert ? (px < threshold) : (px > threshold);
        if (isLine) { sumX += x; count++; }
    }
    *rightPos = (count >= minPx) ? (int)(sumX / count) : -1;
}

/*
 * Full lane detection pipeline:
 *   1. Compute scan row Y positions from ROI settings
 *   2. Scan two rows for left/right lane edges
 *   3. Average results for stability
 *   4. Compute lane center, offset, and steering direction
 */
void detectLanes(uint8_t* buf, int w, int h) {
    int roiTop = (h * g_roiTopPct) / 100;
    int roiH   = h - roiTop;
    int scanY1 = roiTop + (roiH * g_scanRow1Pct) / 100;
    int scanY2 = roiTop + (roiH * g_scanRow2Pct) / 100;

    // Clamp to image bounds
    if (scanY1 >= h) scanY1 = h - 2;
    if (scanY2 >= h) scanY2 = h - 1;

    // Scan both rows
    int l1, r1, l2, r2;
    scanRow(buf, w, scanY1, g_threshold, g_invert, g_minPixels, &l1, &r1);
    scanRow(buf, w, scanY2, g_threshold, g_invert, g_minPixels, &l2, &r2);

    // Average the two rows for each side (use whichever are valid)
    int leftFinal  = -1, rightFinal = -1;

    if (l1 >= 0 && l2 >= 0)       leftFinal = (l1 + l2) / 2;
    else if (l1 >= 0)             leftFinal = l1;
    else if (l2 >= 0)             leftFinal = l2;

    if (r1 >= 0 && r2 >= 0)       rightFinal = (r1 + r2) / 2;
    else if (r1 >= 0)             rightFinal = r1;
    else if (r2 >= 0)             rightFinal = r2;

    // Compute direction
    int frameCenter = w / 2;
    int laneCenter  = -1;
    int offset      = 0;
    const char* dir = "NONE";

    if (leftFinal >= 0 && rightFinal >= 0) {
        // Both lines detected → compute actual lane center
        laneCenter = (leftFinal + rightFinal) / 2;
        offset = laneCenter - frameCenter;
        if (abs(offset) <= g_deadZone) dir = "CENTER";
        else if (offset < 0)           dir = "LEFT";
        else                            dir = "RIGHT";
    }
    else if (leftFinal >= 0) {
        // Only left line → drifting right, steer left
        dir = "LEFT";
        offset = -(frameCenter / 2);
    }
    else if (rightFinal >= 0) {
        // Only right line → drifting left, steer right
        dir = "RIGHT";
        offset = frameCenter / 2;
    }

    // Update global result
    g_lane.leftPos  = leftFinal;
    g_lane.rightPos = rightFinal;
    g_lane.center   = laneCenter;
    g_lane.offset   = offset;
    g_lane.leftOK   = (leftFinal >= 0);
    g_lane.rightOK  = (rightFinal >= 0);
    strncpy(g_lane.dir, dir, sizeof(g_lane.dir) - 1);
    g_lane.dir[sizeof(g_lane.dir) - 1] = '\0';
}

/*
 * Draw visual overlay on the grayscale frame:
 *   - Dashed ROI boundary
 *   - Dotted scan rows
 *   - Frame center line (dim)
 *   - Detected lane positions (bright crosses + vertical lines)
 *   - Lane center line (brightest)
 *   - Direction indicator bar at top
 */
void drawOverlay(uint8_t* buf, int w, int h) {
    int roiTop = (h * g_roiTopPct) / 100;
    int roiH   = h - roiTop;
    int scanY1 = roiTop + (roiH * g_scanRow1Pct) / 100;
    int scanY2 = roiTop + (roiH * g_scanRow2Pct) / 100;
    if (scanY1 >= h) scanY1 = h - 2;
    if (scanY2 >= h) scanY2 = h - 1;

    // ROI boundary — dashed line
    for (int x = 0; x < w; x += 4) {
        setPixel(buf, w, h, x,     roiTop, 180);
        setPixel(buf, w, h, x + 1, roiTop, 180);
    }

    // Scan rows — dotted
    for (int x = 0; x < w; x += 6) {
        setPixel(buf, w, h, x, scanY1, 200);
        setPixel(buf, w, h, x, scanY2, 200);
    }

    // Frame center — dim dashed vertical
    int cx = w / 2;
    for (int y = roiTop; y < h; y += 4) {
        setPixel(buf, w, h, cx, y,     100);
        setPixel(buf, w, h, cx, y + 1, 100);
    }

    // Left lane line
    if (g_lane.leftOK) {
        drawCross(buf, w, h, g_lane.leftPos, scanY1, 6, 255);
        drawCross(buf, w, h, g_lane.leftPos, scanY2, 4, 240);
        for (int dy = -1; dy <= 0; dy++)
            drawVLine(buf, w, h, g_lane.leftPos + dy, roiTop, h - 1, 220);
    }

    // Right lane line
    if (g_lane.rightOK) {
        drawCross(buf, w, h, g_lane.rightPos, scanY1, 6, 255);
        drawCross(buf, w, h, g_lane.rightPos, scanY2, 4, 240);
        for (int dy = 0; dy <= 1; dy++)
            drawVLine(buf, w, h, g_lane.rightPos + dy, roiTop, h - 1, 220);
    }

    // Lane center — bright thick vertical
    if (g_lane.center >= 0) {
        for (int dx = -1; dx <= 1; dx++)
            drawVLine(buf, w, h, g_lane.center + dx, roiTop, h - 1, 255);
    }

    // Direction bar at top of frame
    int barY = 4, barH = 8;
    if (strcmp(g_lane.dir, "LEFT") == 0) {
        for (int y = barY; y < barY + barH; y++)
            drawHLine(buf, w, h, cx - 50, cx, y, 255);
    } else if (strcmp(g_lane.dir, "RIGHT") == 0) {
        for (int y = barY; y < barY + barH; y++)
            drawHLine(buf, w, h, cx, cx + 50, y, 255);
    } else if (strcmp(g_lane.dir, "CENTER") == 0) {
        for (int y = barY; y < barY + barH; y++) {
            int half = barH / 2 - abs(y - barY - barH / 2);
            drawHLine(buf, w, h, cx - half * 3, cx + half * 3, y, 255);
        }
    }
}

// ======================== WEB DASHBOARD HTML ========================
static const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width,initial-scale=1">
    <title>ESP32-CAM Lane Detection</title>
    <style>
        *{margin:0;padding:0;box-sizing:border-box}
        body{font-family:'Segoe UI',system-ui,sans-serif;background:#0f0f1a;color:#e0e0e0;min-height:100vh}
        .hdr{background:linear-gradient(135deg,#1a1a3e,#0d0d2b);padding:14px 20px;border-bottom:1px solid #2a2a5a;display:flex;justify-content:space-between;align-items:center}
        .hdr h1{font-size:1.15em;background:linear-gradient(90deg,#00d4ff,#7b2ff7);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
        .fps{color:#888;font-size:.9em;font-family:monospace}
        .wrap{display:flex;flex-wrap:wrap;gap:14px;padding:14px;max-width:920px;margin:0 auto}
        .vid{flex:1 1 320px;background:#1a1a2e;border-radius:12px;overflow:hidden;border:1px solid #2a2a5a}
        .vid img{width:100%;display:block;background:#000;min-height:200px}
        .side{flex:0 0 255px;display:flex;flex-direction:column;gap:12px}
        .card{background:#1a1a2e;border-radius:12px;padding:15px;border:1px solid #2a2a5a}
        .card h3{font-size:.8em;color:#666;text-transform:uppercase;letter-spacing:1px;margin-bottom:10px}
        .dir{font-size:2.1em;font-weight:700;text-align:center;padding:10px;border-radius:8px;transition:all .3s}
        .d-left{color:#ff9f43;background:rgba(255,159,67,.12)}
        .d-right{color:#ff6b6b;background:rgba(255,107,107,.12)}
        .d-center{color:#2ed573;background:rgba(46,213,115,.12)}
        .d-none{color:#555;background:rgba(85,85,85,.08)}
        .sr{display:flex;justify-content:space-between;padding:5px 0;border-bottom:1px solid #222240}
        .sl{color:#888;font-size:.88em}.sv{font-weight:600;font-family:monospace;font-size:.92em}
        .sg{margin-bottom:10px}
        .sg label{display:flex;justify-content:space-between;font-size:.82em;color:#aaa;margin-bottom:3px}
        input[type=range]{width:100%;accent-color:#7b2ff7;height:6px}
        .btn{width:100%;padding:9px;border:none;border-radius:8px;cursor:pointer;font-weight:600;font-size:.88em;transition:all .2s;margin-top:6px}
        .btn:hover{transform:scale(1.02);opacity:.9}
        .bf{background:linear-gradient(135deg,#f39c12,#e67e22);color:#fff}
        .bf.on{background:linear-gradient(135deg,#2ecc71,#27ae60)}
        .bi{background:linear-gradient(135deg,#3498db,#2980b9);color:#fff}
        .conn{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:6px}
        .conn.ok{background:#2ecc71}.conn.err{background:#e74c3c}
    </style>
</head>
<body>
<div class="hdr">
    <h1>&#x1F6E3; ESP32-CAM Lane Detection</h1>
    <span class="fps"><span class="conn err" id="dot"></span><span id="fps">-- FPS</span></span>
</div>
<div class="wrap">
    <div class="vid"><img id="stream" alt="Stream"></div>
    <div class="side">
        <div class="card">
            <h3>Steering Direction</h3>
            <div class="dir d-none" id="dir">NONE</div>
            <div style="margin-top:10px">
                <div class="sr"><span class="sl">Offset</span><span class="sv" id="off">0 px</span></div>
                <div class="sr"><span class="sl">Left Line</span><span class="sv" id="lp">—</span></div>
                <div class="sr"><span class="sl">Right Line</span><span class="sv" id="rp">—</span></div>
                <div class="sr"><span class="sl">Lane Center</span><span class="sv" id="lc">—</span></div>
            </div>
        </div>
        <div class="card">
            <h3>Detection Settings</h3>
            <div class="sg">
                <label>Threshold <span id="tv">120</span></label>
                <input type="range" id="th" min="30" max="230" value="120"
                    oninput="upd('threshold',this.value);document.getElementById('tv').textContent=this.value">
            </div>
            <div class="sg">
                <label>Dead Zone <span id="dv">20</span></label>
                <input type="range" id="dz" min="5" max="60" value="20"
                    oninput="upd('deadzone',this.value);document.getElementById('dv').textContent=this.value">
            </div>
            <div class="sg">
                <label>ROI Top % <span id="rv">50</span></label>
                <input type="range" id="ri" min="20" max="80" value="50"
                    oninput="upd('roi',this.value);document.getElementById('rv').textContent=this.value">
            </div>
            <button class="btn bi" id="ib" onclick="inv()">Mode: Bright Lines &#x2600;</button>
            <button class="btn bf" id="fb" onclick="fl()">&#x1F4A1; Flash: OFF</button>
        </div>
    </div>
</div>
<script>
    // Build stream URL on port 81
    var host = window.location.hostname;
    document.getElementById('stream').src = 'http://'+host+':81/stream';

    function poll(){
        fetch('/status').then(r=>r.json()).then(d=>{
            var e=document.getElementById('dir');
            e.textContent=d.dir;
            e.className='dir d-'+d.dir.toLowerCase();
            document.getElementById('off').textContent=d.offset+' px';
            document.getElementById('lp').textContent=d.leftOK?d.leftPos+' px':'\u2014';
            document.getElementById('rp').textContent=d.rightOK?d.rightPos+' px':'\u2014';
            document.getElementById('lc').textContent=d.center>=0?d.center+' px':'\u2014';
            document.getElementById('fps').textContent=d.fps.toFixed(1)+' FPS';
            var dot=document.getElementById('dot');
            dot.className='conn ok';
            // Sync slider values from server on first load
        }).catch(()=>{
            document.getElementById('dot').className='conn err';
        });
    }
    function upd(k,v){fetch('/set?'+k+'='+v).catch(()=>{});}
    function fl(){
        fetch('/flash').then(r=>r.json()).then(d=>{
            var b=document.getElementById('fb');
            b.textContent='\u{1F4A1} Flash: '+(d.flash?'ON':'OFF');
            if(d.flash)b.classList.add('on');else b.classList.remove('on');
        });
    }
    function inv(){
        fetch('/set?invert=1').then(()=>fetch('/status')).then(r=>r.json()).then(d=>{
            document.getElementById('ib').textContent='Mode: '+(d.invert?'Dark Lines \u{1F319}':'Bright Lines \u{2600}');
        });
    }
    setInterval(poll,250);
    poll();
</script>
</body>
</html>
)rawliteral";

// ======================== HTTP HANDLERS ========================

// GET / → Dashboard page
static esp_err_t index_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

// GET /status → JSON with current lane detection data
static esp_err_t status_handler(httpd_req_t* req) {
    char json[256];
    snprintf(json, sizeof(json),
        "{\"dir\":\"%s\",\"offset\":%d,\"leftPos\":%d,\"rightPos\":%d,"
        "\"center\":%d,\"leftOK\":%s,\"rightOK\":%s,\"fps\":%.1f,\"invert\":%s}",
        g_lane.dir, g_lane.offset, g_lane.leftPos, g_lane.rightPos,
        g_lane.center,
        g_lane.leftOK  ? "true" : "false",
        g_lane.rightOK ? "true" : "false",
        g_fps,
        g_invert ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, strlen(json));
}

// GET /set?threshold=120&deadzone=20&roi=50&invert=1
static esp_err_t settings_handler(httpd_req_t* req) {
    char buf[128];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(buf, "threshold", val, sizeof(val)) == ESP_OK)
            g_threshold = constrain(atoi(val), 10, 250);
        if (httpd_query_key_value(buf, "deadzone", val, sizeof(val)) == ESP_OK)
            g_deadZone = constrain(atoi(val), 1, 100);
        if (httpd_query_key_value(buf, "roi", val, sizeof(val)) == ESP_OK)
            g_roiTopPct = constrain(atoi(val), 10, 90);
        if (httpd_query_key_value(buf, "invert", val, sizeof(val)) == ESP_OK)
            g_invert = !g_invert;  // Toggle
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "OK", 2);
}

// GET /flash → Toggle flash LED
static esp_err_t flash_handler(httpd_req_t* req) {
    g_flashOn = !g_flashOn;
    digitalWrite(FLASH_GPIO_NUM, g_flashOn ? HIGH : LOW);

    char json[32];
    snprintf(json, sizeof(json), "{\"flash\":%s}", g_flashOn ? "true" : "false");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, strlen(json));
}

// GET /stream → MJPEG stream with lane overlay
#define BOUNDARY "frameboundary"
static const char* STREAM_CT   = "multipart/x-mixed-replace;boundary=" BOUNDARY;
static const char* STREAM_SEP  = "\r\n--" BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t* req) {
    esp_err_t res;
    char partHdr[64];

    res = httpd_resp_set_type(req, STREAM_CT);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "15");

    g_streaming = true;
    Serial.println("[Stream] Client connected");

    while (true) {
        // 1. Capture grayscale frame
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("[Stream] Capture failed, retrying...");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            continue;
        }

        // 2. Run lane detection on raw pixels
        detectLanes(fb->buf, fb->width, fb->height);

        // 3. Draw visual overlay
        drawOverlay(fb->buf, fb->width, fb->height);

        // 4. Convert grayscale buffer → JPEG
        uint8_t* jpgBuf = NULL;
        size_t   jpgLen = 0;
        bool ok = fmt2jpg(fb->buf, fb->len, fb->width, fb->height,
                          PIXFORMAT_GRAYSCALE, 40, &jpgBuf, &jpgLen);
        esp_camera_fb_return(fb);

        if (!ok || !jpgBuf) {
            Serial.println("[Stream] JPEG conversion failed");
            if (jpgBuf) free(jpgBuf);
            continue;
        }

        // 5. Send MJPEG frame
        res = httpd_resp_send_chunk(req, STREAM_SEP, strlen(STREAM_SEP));
        if (res == ESP_OK) {
            size_t hlen = snprintf(partHdr, sizeof(partHdr), STREAM_PART, (unsigned)jpgLen);
            res = httpd_resp_send_chunk(req, partHdr, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char*)jpgBuf, jpgLen);
        }

        free(jpgBuf);

        // 6. FPS counter
        g_frameCount++;
        unsigned long now = millis();
        if (now - g_lastFpsCalc >= 1000) {
            g_fps = g_frameCount * 1000.0f / (now - g_lastFpsCalc);
            g_frameCount = 0;
            g_lastFpsCalc = now;
        }

        if (res != ESP_OK) {
            Serial.println("[Stream] Client disconnected");
            break;
        }
    }

    g_streaming = false;
    return res;
}

// ======================== START HTTP SERVERS ========================
void startServers() {
    // ---- Stream server on port 81 (separate so it doesn't block controls) ----
    httpd_config_t streamCfg   = HTTPD_DEFAULT_CONFIG();
    streamCfg.server_port      = 81;
    streamCfg.ctrl_port        = 32769;
    streamCfg.max_uri_handlers = 2;
    streamCfg.stack_size       = 8192;

    if (httpd_start(&stream_httpd, &streamCfg) == ESP_OK) {
        httpd_uri_t uri = { "/stream", HTTP_GET, stream_handler, NULL };
        httpd_register_uri_handler(stream_httpd, &uri);
        Serial.println("[HTTP] Stream server on port 81");
    }

    // ---- Control server on port 80 ----
    httpd_config_t ctrlCfg   = HTTPD_DEFAULT_CONFIG();
    ctrlCfg.server_port      = 80;
    ctrlCfg.max_uri_handlers = 8;

    if (httpd_start(&control_httpd, &ctrlCfg) == ESP_OK) {
        httpd_uri_t u1 = { "/",       HTTP_GET, index_handler,    NULL };
        httpd_uri_t u2 = { "/status", HTTP_GET, status_handler,   NULL };
        httpd_uri_t u3 = { "/set",    HTTP_GET, settings_handler, NULL };
        httpd_uri_t u4 = { "/flash",  HTTP_GET, flash_handler,    NULL };
        httpd_register_uri_handler(control_httpd, &u1);
        httpd_register_uri_handler(control_httpd, &u2);
        httpd_register_uri_handler(control_httpd, &u3);
        httpd_register_uri_handler(control_httpd, &u4);
        Serial.println("[HTTP] Control server on port 80");
    }
}

// ======================== SETUP ========================
void setup() {
    Serial.begin(115200);
    Serial.println("\n========================================");
    Serial.println("  ESP32-CAM Lane Detection  (Standalone)");
    Serial.println("========================================\n");

    // Flash LED
    pinMode(FLASH_GPIO_NUM, OUTPUT);
    digitalWrite(FLASH_GPIO_NUM, LOW);

    // Camera
    if (!initCamera()) {
        Serial.println("FATAL: Camera failed. Check wiring & board selection.");
        while (1) { delay(1000); }
    }

    // WiFi
    initWiFi();

    // HTTP servers
    startServers();

    Serial.println("\n>>> Open the IP address above in your browser <<<\n");
    g_lastFpsCalc = millis();
}

// ======================== MAIN LOOP ========================
void loop() {
    // When nobody is watching the stream, still run lane detection
    if (!g_streaming) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
            detectLanes(fb->buf, fb->width, fb->height);
            esp_camera_fb_return(fb);

            g_frameCount++;
            unsigned long now = millis();
            if (now - g_lastFpsCalc >= 1000) {
                g_fps = g_frameCount * 1000.0f / (now - g_lastFpsCalc);
                g_frameCount = 0;
                g_lastFpsCalc = now;
            }
        }
    }

    // Serial output (rate-limited to every 250ms)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 250) {
        Serial.printf("[Lane] Dir: %-6s | Offset: %+4d px | L: %3d | R: %3d | C: %3d | %.1f FPS\n",
            g_lane.dir, g_lane.offset,
            g_lane.leftPos, g_lane.rightPos, g_lane.center, g_fps);
        lastPrint = millis();
    }

    delay(10);
}
