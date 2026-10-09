import cv2
import requests
import base64

# --- CONFIGURATION ---
ESP32_STREAM_URL = "http://192.168.1.103/stream"
GROQ_API_KEY = "YOUR_GROQ_API_KEY_HERE"

# We use LLaMA 3.2 Vision because it is Groq's official lightning-fast vision model.
# (qwen/qwen3.8-27b is a text model and might throw an error with images on Groq!)
VISION_MODEL = "llama-3.2-11b-vision-preview" 

def analyze_frame(base64_image):
    print("\n🧠 Sending image to Groq AI...")
    headers = {
        "Authorization": f"Bearer {GROQ_API_KEY}",
        "Content-Type": "application/json"
    }
    
    payload = {
        "model": VISION_MODEL,
        "messages": [
            {
                "role": "system",
                "content": "You are the AI brain of an autonomous car. Tell me exactly what obstacle is in front of me. Keep it to one short sentence. For example: 'There is a person standing in the road.' or 'The road is clear.'"
            },
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": "What is in front of the car right now?"},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}}
                ]
            }
        ],
        "temperature": 0.2,
        "max_tokens": 100
    }
    
    try:
        response = requests.post("https://api.groq.com/openai/v1/chat/completions", headers=headers, json=payload)
        response_data = response.json()
        
        if "error" in response_data:
            print("❌ API Error:", response_data["error"]["message"])
        else:
            print("🤖 AI SAYS:", response_data["choices"][0]["message"]["content"], "\n")
            
    except Exception as e:
        print("❌ Network Error:", e)

def main():
    print(f"🚗 Connecting to ESP32-CAM at {ESP32_STREAM_URL} ...")
    cap = cv2.VideoCapture(ESP32_STREAM_URL)
    
    if not cap.isOpened():
        print("❌ Could not connect to the video stream. Make sure the ESP32 is running!")
        return

    print("✅ Connected!")
    print(">>> PRESS 'SPACE' to analyze what the car sees.")
    print(">>> PRESS 'Q' to quit.\n")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            time.sleep(1)
            continue
            
        cv2.imshow("ESP32-CAM Dashcam", frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):  # SPACEBAR pressed
            # Encode frame to base64
            _, buffer = cv2.imencode('.jpg', frame)
            jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            analyze_frame(jpg_as_text)
            
        elif key == ord('q'): # Q pressed
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
