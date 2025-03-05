import time
import json
import threading
from flask import Flask, render_template, jsonify, Response
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import busio
from adafruit_htu21d import HTU21D
import Adafruit_MCP3008
import os
import subprocess
import camera
import cv2
data_lock = threading.Lock()

# GPIO 설정
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# LED 핀 설정
LED_RED = 13
LED_GREEN = 6
LED_YELLOW = 19
LED_WHITE = 16
GPIO.setup(LED_RED, GPIO.OUT)
GPIO.setup(LED_GREEN, GPIO.OUT)
GPIO.setup(LED_YELLOW, GPIO.OUT)
GPIO.setup(LED_WHITE, GPIO.OUT)

# 초음파 센서 핀 설정
TRIG = 24
ECHO = 23
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# 스위치 핀 설정
SWITCH_1 = 21
SWITCH_2 = 22
GPIO.setup(SWITCH_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(SWITCH_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# I2C 설정 (온습도 센서)
sda = 2
scl = 3
i2c = busio.I2C(scl, sda)
i2c.writeto(0x40, b'\xFE')
sensor_htu21d = HTU21D(i2c)

# MCP3202 설정 (조도 센서)
mcp = Adafruit_MCP3008.MCP3008(clk=11, cs=8, miso=9, mosi=10)

# MQTT 설정
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_PUB_TOPIC = "smart_home/sensor_data"
MQTT_SUB_TOPIC = "smart_home/control"
mqtt_client = mqtt.Client()

# Flask 설정
app = Flask(__name__)

# 프로그램 상태 변수
program_state = {"is_on": True}  # 프로그램 ON/OFF 상태
sensor_data_cache = {}
prev_switch_state = 0  # 스위치1 이전 상태
latest_sensor_data = {
    "temp": None,
    "hum": None,
    "light": None,
    "distance": None,
    "led_white": False,
    "program_state": True
}  # 최신 센서 데이터를 저장할 전역 변수
auto_led_control = True  # LED 자동 제어 모드 상태

# 온습도 센서 함수
def getTemperature():
    temp = round(float(sensor_htu21d.temperature), 2)
    return temp

def getHumidity():
    hum = round(float(sensor_htu21d.relative_humidity), 2)
    return hum

# 조도 센서 함수
def getLight():
    light = mcp.read_adc(0)
    return light

# 초음파 센서 함수
def measureDistance():
    time.sleep(0.2)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = round((pulse_duration * 343 * 100) / 2, 2)
    return distance

# LED 상태 확인 함수
def get_led_state(pin):
    return GPIO.input(pin) == GPIO.HIGH

# 프로그램 상태 전환 함수
def toggle_program_state():
    global latest_sensor_data
    program_state["is_on"] = not program_state["is_on"]
    with data_lock:
        latest_sensor_data["program_state"] = program_state["is_on"]
        if not program_state["is_on"]:
            # 프로그램이 OFF일 때 모든 상태 초기화
            latest_sensor_data.update({
                "temp": None,
                "hum": None,
                "light": None,
                "distance": None,
                "led_white": False,
                "program_state": False
            })
            GPIO.output(LED_GREEN, False)
            GPIO.output(LED_WHITE, False)
            GPIO.output(LED_RED, False)
            GPIO.output(LED_YELLOW, False)
            play_audio("powerOff.wav")
        else:
            play_audio("powerOn.wav")
    mqtt_payload = json.dumps(latest_sensor_data)
    mqtt_client.publish(MQTT_PUB_TOPIC, mqtt_payload)

# 오디오 파일 재생 함수
def play_audio(file_name):
    def play_async():
        try:
            base_dir = os.path.dirname(os.path.abspath(__file__))
            audio_dir = os.path.join(base_dir, 'audio')
            audio_path = os.path.join(audio_dir, file_name)
            subprocess.run(['aplay', audio_path], check=True)
        except Exception as e:
            print(f"Error playing audio: {e}")

    threading.Thread(target=play_async, daemon=True).start()

# MQTT 데이터 전송
def publish_sensor_data():
    global latest_sensor_data
    previous_led_state = False
    user_detection = False

    while True:
        try:
            if program_state["is_on"]:
                GPIO.output(LED_GREEN, True)

                # 센서 데이터 읽기
                temp = getTemperature()
                hum = getHumidity()
                light = getLight()
                distance = measureDistance()
                led_state = get_led_state(LED_WHITE)

                warning_condition = False

                # 온습도 경고 조건
                if temp is not None and (temp < 20 or temp > 28):
                    warning_condition = True
                if hum is not None and (hum < 40 or hum > 60):
                    warning_condition = True

                # 경고 LED 제어
                GPIO.output(LED_YELLOW, warning_condition)

                sensor_data = {
                    "temp": temp,
                    "hum": hum,
                    "light": light,
                    "distance": distance,
                    "led_white": led_state,
                    "program_state": program_state["is_on"]
                }

                # None이 아닌 값만 업데이트
                with data_lock:
                    for key, value in sensor_data.items():
                        if value is not None:
                            latest_sensor_data[key] = value

                # MQTT 발행
                mqtt_payload = json.dumps(latest_sensor_data)
                result = mqtt_client.publish(MQTT_PUB_TOPIC, mqtt_payload)

                # LED 자동 제어
                if auto_led_control and light is not None:
                    if light < 100:
                        if not led_state:  # LED가 꺼져있을 때만
                            GPIO.output(LED_WHITE, True)
                            if not previous_led_state:  # 상태가 변경될 때만
                                play_audio("LedOn.wav")
                    else:
                        if led_state:  # LED가 켜져있을 때만
                            GPIO.output(LED_WHITE, False)
                            if previous_led_state:  # 상태가 변경될 때만
                                play_audio("LedOff.wav")
                previous_led_state = led_state  # 이전 상태 업데이트

                # 사용자 감지
                if distance is not None:
                    if distance < 10:
                        GPIO.output(LED_RED, True)
                        if not user_detection:  # 처음 감지될 때만
                            play_audio("userIn.wav")
                            user_detection = True
                    else:
                        GPIO.output(LED_RED, False)
                        user_detection = False  # 거리가 멀어지면 리셋
            time.sleep(3)

        except Exception as e:
            import traceback
            print(traceback.format_exc())
            time.sleep(3)

def on_message(client, userdata, message):
    global latest_sensor_data
    try:
        if message.topic == MQTT_SUB_TOPIC:
            payload = message.payload.decode("utf-8")
            control_data = json.loads(payload)

            if "led_control" in control_data:
                if control_data["led_control"] == "on":
                    GPIO.output(LED_WHITE, True)
                elif control_data["led_control"] == "off":
                    GPIO.output(LED_WHITE, False)

                with data_lock:
                    latest_sensor_data["led_white"] = get_led_state(LED_WHITE)

    except Exception as e:
        print(f"Error processing MQTT message: {e}")

def on_connect(client, userdata, flags, rc):
    client.subscribe(MQTT_SUB_TOPIC)

# 스위치 상태 모니터링 함수
def monitor_switch():
    global prev_switch_state, auto_led_control
    prev_switch2_state = 0

    while True:
        # SWITCH_1 (전원) 처리
        curr_switch_state = GPIO.input(SWITCH_1)
        if curr_switch_state == GPIO.HIGH and prev_switch_state == GPIO.LOW:
            toggle_program_state()  # 프로그램 상태 전환
        prev_switch_state = curr_switch_state

        # SWITCH_2 (조명 모드 전환) 처리
        curr_switch2_state = GPIO.input(SWITCH_2)
        if curr_switch2_state == GPIO.HIGH and prev_switch2_state == GPIO.LOW:
            auto_led_control = not auto_led_control  # 모드 전환
            play_audio("mode.wav")
            print(f"LED 제어 모드 변경: {'자동' if auto_led_control else '수동'}")

        prev_switch2_state = curr_switch2_state

        time.sleep(0.1)

# Flask 라우트
@app.route('/api/sensor_data', methods=['GET'])
def get_sensor_data():
    with data_lock:
        response_data = latest_sensor_data.copy()
        return jsonify(response_data)

@app.route('/')
def home():
    return render_template('index.html')

@app.route("/control/led/<state>", methods=["POST"])
def control_led(state):
    if not auto_led_control:  # 수동 모드일 때만 작동
        if state == "on":
            GPIO.output(LED_WHITE, True)
            play_audio("LedOn.wav")
            return jsonify({"status": "LED ON"}), 200
        elif state == "off":
            GPIO.output(LED_WHITE, False)
            play_audio("LedOff.wav")
            return jsonify({"status": "LED OFF"}), 200
        else:
            return jsonify({"error": "Invalid state"}), 400
    else:
        return jsonify({"error": "자동 모드에서는 LED를 수동으로 제어할 수 없습니다"}), 400

@app.route('/cctv')
def cctv():
    image = camera.take_picture(most_recent=True)
    if image is not None:
        file_path = './static/cctv.jpg'
        cv2.imwrite(file_path, image)
        return render_template("cctv.html", fname=f'/static/cctv.jpg?t={int(time.time())}')
    else:
        return "Error: Failed to capture image from the camera.", 500

# Flask 서버 실행
def start_flask():
    app.run(host="0.0.0.0", port=5000)

# 메인 함수
if __name__ == "__main__":
    try:
        print("스마트홈 시스템 시작...")

        if not camera.init("/dev/video0"):
            print("카메라 초기화 실패")
            exit(1)

        # MQTT 설정 및 연결
        mqtt_client.on_connect = on_connect
        mqtt_client.on_message = on_message
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()

        # 데몬 스레드 시작
        threading.Thread(target=publish_sensor_data, daemon=True).start()
        threading.Thread(target=monitor_switch, daemon=True).start()

        print("\n 웹 서버 시작")
        app.run(host="0.0.0.0", port=5000, debug=False)

    except KeyboardInterrupt:
        print("\n프로그램 종료 중...")
        GPIO.cleanup()
        mqtt_client.disconnect()
        camera.final()
        print("프로그램이 종료되었습니다.")
