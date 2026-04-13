from flask import Flask, Response, jsonify, request
import cv2
import os
import time
import sys

# Import communication architecture from the main station
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../Software')))
from config import Config
from comms import ESP32Comms, find_esp32_by_mac

app = Flask(__name__)

# Config
DATASET_DIR = "dataset"
RES = (416, 416)

# Ensure directory exists
os.makedirs(DATASET_DIR, exist_ok=True)

camera = None

def get_camera():
    global camera
    if camera is None:
        camera = cv2.VideoCapture(0)
    return camera

def generate_frames():
    cam = get_camera()
    while True:
        success, frame = cam.read()
        if not success:
            time.sleep(0.1)
            continue
        
        # Center crop the frame to be square before resizing to 416x416
        # The camera is likely 640x480 (4:3), YOLO prefers square inputs
        h, w, _ = frame.shape
        min_dim = min(h, w)
        start_x = w // 2 - min_dim // 2
        start_y = h // 2 - min_dim // 2
        
        cropped = frame[start_y:start_y+min_dim, start_x:start_x+min_dim]
        resized = cv2.resize(cropped, RES)
        
        ret, buffer = cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ret:
            continue
            
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>YOLO Dataset Collector</title>
    <style>
        body { 
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; 
            text-align: center; 
            background: #111; 
            color: #eee; 
            margin-top: 40px; 
        }
        .container {
            display: flex;
            flex-direction: column;
            align-items: center;
        }
        h1 { margin-bottom: 5px; color: #fff; font-weight: 500;}
        p.subtitle { color: #888; font-size: 14px; margin-top: 0; margin-bottom: 20px;}
        img { 
            border: 2px solid #333; 
            border-radius: 8px; 
            width: 416px; 
            height: 416px; 
            object-fit: cover; 
            background: #222;
        }
        button { 
            margin-top: 25px; 
            padding: 16px 36px; 
            font-size: 20px; 
            font-weight: bold;
            background: #3b82f6; 
            color: white; 
            border: none; 
            border-radius: 8px; 
            cursor: pointer; 
            transition: background 0.1s, transform 0.1s;
            box-shadow: 0 4px 6px rgba(0,0,0,0.3);
        }
        button:active { 
            background: #2563eb; 
            transform: scale(0.97);
        }
        #status { 
            margin-top: 20px; 
            font-size: 16px;
            color: #aaa; 
            min-height: 24px;
        }
        .success { color: #10b981 !important; }
    </style>
</head>
<body>
    <div class="container">
        <h1>YOLO Dataset Collector</h1>
        <p class="subtitle">416x416 Output • Square Center Crop • Press "Spacebar" to Capture</p>
        
        <div>
            <img src="/video_feed" id="video" alt="Camera Feed Loading..." />
        </div>
        
        <button onclick="takePicture()" id="btn">📸 Capture Photo</button>
        <div id="status">Waiting to capture...</div>
        
        <p class="subtitle" style="margin-top: 30px;">Hold <b>W, A, S, D</b> keys to drive the robot.</p>
    </div>

    <script>
        const statusEl = document.getElementById('status');
        let isCapturing = false;

        function takePicture() {
            if(isCapturing) return;
            isCapturing = true;
            statusEl.innerText = "Capturing...";
            statusEl.className = "";
            
            fetch('/capture', { method: 'POST' })
            .then(r => r.json())
            .then(data => {
                if(data.success) {
                    statusEl.innerText = data.message;
                    statusEl.className = "success";
                } else {
                    statusEl.innerText = "Error: " + data.message;
                    statusEl.className = "";
                }
                setTimeout(() => {
                    if (statusEl.innerText === data.message || statusEl.innerText.startsWith("Error")) {
                        statusEl.innerText = 'Ready for next photo...';
                        statusEl.className = "";
                    }
                }, 2500);
            })
            .catch(err => {
                statusEl.innerText = "Network Error!";
            })
            .finally(() => {
                isCapturing = false;
            });
        }
        
        // ── Motor Controls ──
        const keys = { w: false, a: false, s: false, d: false };
        let DRIVE_POWER = 0.8;
        let TURN_POWER = 0.6;

        function sendDrive() {
            let left = 0;
            let right = 0;

            if (keys.w) { left += DRIVE_POWER; right += DRIVE_POWER; }
            if (keys.s) { left -= DRIVE_POWER; right -= DRIVE_POWER; }
            if (keys.a) { left -= TURN_POWER; right += TURN_POWER; }
            if (keys.d) { left += TURN_POWER; right -= TURN_POWER; }

            left = Math.max(-1.0, Math.min(1.0, left));
            right = Math.max(-1.0, Math.min(1.0, right));

            fetch('/drive', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({l: left, r: right})
            }).catch(e => {});
        }

        // Add spacebar and WASD shortcuts
        document.addEventListener('keydown', function(event) {
            const key = event.key.toLowerCase();
            if(event.code === 'Space') {
                event.preventDefault(); // prevent page scroll
                takePicture();
            } else if (keys.hasOwnProperty(key)) {
                if (!keys[key]) {
                    keys[key] = true;
                    sendDrive();
                }
            }
        });

        document.addEventListener('keyup', function(event) {
            const key = event.key.toLowerCase();
            if (keys.hasOwnProperty(key)) {
                keys[key] = false;
                sendDrive();
            }
        });
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return HTML_PAGE

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/capture', methods=['POST'])
def capture():
    cam = get_camera()
    success, frame = cam.read()
    if success:
        # Re-apply the center square crop to the saved image
        h, w, _ = frame.shape
        min_dim = min(h, w)
        start_x = w // 2 - min_dim // 2
        start_y = h // 2 - min_dim // 2
        
        cropped = frame[start_y:start_y+min_dim, start_x:start_x+min_dim]
        resized = cv2.resize(cropped, RES)
        
        filename = f"img_{int(time.time()*1000)}.jpg"
        filepath = os.path.join(DATASET_DIR, filename)
        cv2.imwrite(filepath, resized)
        
        count = len(os.listdir(DATASET_DIR))
        return jsonify({"success": True, "message": f"Saved {filename} (Total: {count})"})
        
    return jsonify({"success": False, "message": "Camera hardware failed to read frame"})

@app.route('/drive', methods=['POST'])
def drive():
    data = request.json
    l = float(data.get('l', 0.0))
    r = float(data.get('r', 0.0))
    esp32.send_motor_command(l, r)
    return jsonify({"success": True})

def cleanup():
    if camera is not None:
        camera.release()
    esp32.stop()

# Initialize Robot Comms
_discovered_ip = find_esp32_by_mac(Config.ESP32_MAC)
_esp32_ip = _discovered_ip if _discovered_ip else Config.ESP32_IP
esp32 = ESP32Comms(
    esp32_ip=_esp32_ip,
    udp_port=Config.UDP_PORT,
    imu_port=Config.IMU_PORT,
    failsafe_timeout=Config.FAILSAFE_TIMEOUT_MS,
)
esp32.start()

import atexit
atexit.register(cleanup)

if __name__ == '__main__':
    print(f"==========================================")
    print(f" Starting YOLO Data Collector")
    print(f" Data Directory: {os.path.abspath(DATASET_DIR)}")
    print(f" Resolution: {RES[0]}x{RES[1]}")
    print(f" Dashboard: http://localhost:5001")
    print(f"==========================================")
    print(f" WARNING: Make sure to stop the main ground station before running this!")
    app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
