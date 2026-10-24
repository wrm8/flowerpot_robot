import os
import sys
import time
import uuid
import json
import glob
import random
import asyncio
import difflib
import webbrowser
import base64
import requests
from urllib.parse import quote

import cv2
import numpy as np
import sounddevice as sd
import soundfile as sf
import pygame
import edge_tts
from faster_whisper import WhisperModel
from openai import OpenAI

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String



# ================== 基础配置 ==================

API_KEY = "sk-ws-H.REDLHEX.0D0c.MEYCIQCl19CAFRtDfiO0PoeQaIxjZW0x8OSDz4kVA23UaViHjwIhAI6Puoxg_jBgboNVqLq9YBokl_nUmjNeBgfPonAf2tIq"   # 换成你新的百炼 API Key，不要把真实 Key 发出来

BASE_URL = "https://dashscope.aliyuncs.com/compatible-mode/v1"

MODEL_NAME = "qwen-plus"
WEB_MODEL_NAME = "qwen-plus"
VISION_MODEL_NAME = "qwen-vl-plus"

CAMERA_INDEX = 0

CONTACTS_FILE = "contacts.json"
MUSIC_DIR = "music"

# 如果默认麦克风不对，可以先运行后看设备列表，再把这里改成对应编号
AUDIO_INPUT_DEVICE = 0

# ================== 天气定位配置 ==================

# 固定城市。为空时自动用网络 IP 定位。
# 如果你部署在固定地点，比如南京，可以直接写 HOME_CITY = "南京"
HOME_CITY = "镇江"

ENABLE_IP_LOCATION = True

# ================== 关键音量参数 ==================

WAKE_MIN_RMS = 0.050
COMMAND_MIN_RMS = 0.006
VOICE_THRESHOLD = 0.007

WAKE_SIMILARITY = 0.72

WAKE_TARGETS = [
    "小园该干活了",
    "小圆该干活了",
    "小源该干活了",
    "小原该干活了",
    "小园小园",
    "小圆小圆",
    "小源小源",
    "小原小原",
    "小元小元"
]


# ================== 通义千问客户端 ==================

client = OpenAI(
    api_key=API_KEY,
    base_url=BASE_URL
)


# ================== ASR 模型 ==================

asr_model = WhisperModel(
    "base",
    device="cpu",
    compute_type="int8"
)


# ================== 对话上下文 ==================

messages = [
    {
        "role": "system",
        "content": """
你叫小园，是一个家庭园艺陪伴机器人。
你的语气温柔、自然、简短。
你可以陪用户聊天，也可以回答养花、盆栽、家庭园艺相关问题。
你的回复控制在50字以内，像真实陪伴机器人一样自然。
如果用户情绪低落，要温柔安慰；如果用户问植物问题，要给实用建议。
暂时不要输出JSON，不要控制机器人，只正常聊天回复。
"""
    }
]


# ================== 文本处理 ==================

def normalize_text(text):
    if not text:
        return ""

    text = text.replace(" ", "")
    text = text.replace("\n", "")
    text = text.replace("\r", "")
    text = text.replace("，", "").replace(",", "")
    text = text.replace("。", "").replace(".", "")
    text = text.replace("！", "").replace("!", "")
    text = text.replace("？", "").replace("?", "")
    text = text.replace("：", "").replace(":", "")
    text = text.replace("、", "")
    text = text.replace("~", "")
    text = text.replace("～", "")

    convert_map = {
        "該": "该",
        "關": "关",
        "閉": "闭",
        "結": "结",
        "幹": "干",
        "兒": "儿",
        "樂": "乐",
        "音樂": "音乐",
        "聽": "听",
        "給": "给",
        "這": "这",
        "個": "个",
        "嗎": "吗",
        "麼": "么",
        "媽": "妈",
        "圓": "圆",
        "園": "园",
        "喔": "哦",
        "臺": "台",
        "程式": "程序",
        "成序": "程序",
        "层序": "程序",
        "陈序": "程序",
        "程序员": "程序",
        "來": "来"
    }

    for old, new in convert_map.items():
        text = text.replace(old, new)

    return text.strip()


def is_noise_text(text):
    text = normalize_text(text)

    noise_words = [
        "谢谢大家",
        "感谢观看",
        "哈哈哈",
        "呵呵",
        "字幕",
        "请不吝点赞",
        "订阅",
        "转发",
        "拜拜"
    ]

    return any(word in text for word in noise_words)


def is_wake_word(text):
    text = normalize_text(text)

    if not text:
        return False

    if is_noise_text(text):
        return False

    if "小" not in text:
        return False

    for wake in WAKE_TARGETS:
        if wake in text:
            return True

    for wake in WAKE_TARGETS:
        similarity = difflib.SequenceMatcher(None, text, wake).ratio()
        if similarity >= WAKE_SIMILARITY:
            print(f"模糊唤醒成功：识别文本={text}，目标={wake}，相似度={similarity:.2f}")
            return True

    return False


def is_sleep_word(text):
    text = normalize_text(text)

    sleep_words = [
        "退下吧",
        "休息吧",
        "待命吧",
        "不用了",
        "先这样",
        "结束对话",
        "安静待命",
        "你先退下",
        "你先休息"
    ]

    return any(word in text for word in sleep_words)


def is_exit_word(text):
    text = normalize_text(text)

    if not text:
        return False

    exit_words = [
        "关闭程序",
        "退出程序",
        "结束程序",
        "关闭系统",
        "关掉程序",
        "停止程序",
        "停止运行",
        "关闭运行",
        "结束运行",
        "退出运行",
        "小园关闭",
        "小圆关闭",
        "小园退出",
        "小圆退出",
        "我要退出",
        "退出小园",
        "退出小圆",
        "关掉小园",
        "关掉小圆",
        "关闭小园",
        "关闭小圆",
        "停止小园",
        "停止小圆",
        "结束小园",
        "结束小圆"
    ]

    for word in exit_words:
        if word in text:
            print("检测到退出词：" + word)
            return True

    action_words = ["关闭", "退出", "结束", "关掉", "停止"]
    target_words = ["程序", "系统", "运行", "小园", "小圆", "软件"]

    if any(a in text for a in action_words) and any(t in text for t in target_words):
        print("检测到组合退出指令：" + text)
        return True

    strong_targets = [
        "关闭程序",
        "退出程序",
        "结束程序",
        "关掉程序",
        "停止程序",
        "关闭小园",
        "关闭小圆",
        "退出小园",
        "退出小圆"
    ]

    for target in strong_targets:
        similarity = difflib.SequenceMatcher(None, text, target).ratio()
        if similarity >= 0.52:
            print(f"模糊退出成功：识别文本={text}，目标={target}，相似度={similarity:.2f}")
            return True

    return False


def shutdown_program():
    print("正在关闭小园程序...")

    try:
        if pygame.mixer.get_init():
            pygame.mixer.music.stop()
            pygame.mixer.quit()
    except Exception:
        pass

    try:
        pygame.quit()
    except Exception:
        pass

    sys.exit(0)


# ================== 联系人功能 ==================

def ensure_contacts_file():
    if os.path.exists(CONTACTS_FILE):
        return

    sample_contacts = {
        "妈妈": {
            "phone": "13800000000",
            "relation": "家属"
        },
        "儿子": {
            "phone": "13900000000",
            "relation": "家属"
        },
        "女儿": {
            "phone": "13700000000",
            "relation": "家属"
        }
    }

    with open(CONTACTS_FILE, "w", encoding="utf-8") as f:
        json.dump(sample_contacts, f, ensure_ascii=False, indent=2)

    print("已自动创建 contacts.json，请把里面的手机号改成真实联系人。")


def load_contacts():
    ensure_contacts_file()

    with open(CONTACTS_FILE, "r", encoding="utf-8") as f:
        return json.load(f)


def find_contact_from_text(text):
    text = normalize_text(text)
    contacts = load_contacts()

    for name in contacts.keys():
        if name in text:
            return name, contacts[name]

    return None, None


def handle_contact_command(text):
    text = normalize_text(text)

    if not any(word in text for word in ["联系", "打电话", "找一下", "呼叫", "拨打"]):
        return False

    name, info = find_contact_from_text(text)

    if name is None:
        speak("我还不知道要联系谁，请说联系妈妈、儿子或女儿。")
        return True

    phone = info.get("phone", "未填写")
    relation = info.get("relation", "联系人")

    reply = f"已找到{name}，他是你的{relation}，电话是{phone}。目前先为你读出号码。"
    print("小园：" + reply)
    speak(reply)

    return True


# ================== 本地音乐功能 ==================

def ensure_music_dir():
    if not os.path.exists(MUSIC_DIR):
        os.makedirs(MUSIC_DIR)
        print("已自动创建 music 文件夹，请把 mp3 / wav / ogg 歌曲放进去。")


def get_music_files():
    ensure_music_dir()

    files = []
    files.extend(glob.glob(os.path.join(MUSIC_DIR, "*.mp3")))
    files.extend(glob.glob(os.path.join(MUSIC_DIR, "*.wav")))
    files.extend(glob.glob(os.path.join(MUSIC_DIR, "*.ogg")))

    return files


def stop_music(silent=False):
    try:
        if pygame.mixer.get_init():
            pygame.mixer.music.stop()
            pygame.mixer.quit()
    except Exception:
        pass

    if not silent:
        speak("音乐已停止。")


def play_local_music(keyword=None):
    files = get_music_files()

    if not files:
        speak("我还没有找到本地音乐，请先把歌曲放到 music 文件夹里。")
        return

    selected = None

    if keyword:
        keyword = normalize_text(keyword)
        for file in files:
            filename = normalize_text(os.path.basename(file))
            if keyword in filename:
                selected = file
                break

    if selected is None:
        selected = random.choice(files)

    song_name = os.path.basename(selected)

    speak(f"好的，给你播放{song_name}。")

    try:
        pygame.mixer.init()
        pygame.mixer.music.load(selected)
        pygame.mixer.music.set_volume(0.45)
        pygame.mixer.music.play()
        print(f"正在播放音乐：{song_name}")
    except Exception as e:
        print("音乐播放失败：", e)
        speak("音乐播放失败了，请检查文件格式。")


def handle_music_command(text):
    text = normalize_text(text)

    if any(word in text for word in ["停止音乐", "关闭音乐", "关掉音乐", "别放了", "暂停音乐"]):
        stop_music()
        return True

    if any(word in text for word in ["播放音乐", "放音乐", "放一首歌", "来首歌", "播放歌曲", "听音乐", "放歌", "来首音乐"]):
        keyword = None

        for prefix in ["播放", "放一首", "放", "听", "来首"]:
            if prefix in text:
                keyword = text.split(prefix, 1)[-1]
                keyword = keyword.replace("音乐", "").replace("歌曲", "").replace("歌", "").strip()
                break

        play_local_music(keyword)
        return True

    return False


# ================== 视觉大模型功能 ==================

def capture_image(filename="front_view.jpg"):
    cap = cv2.VideoCapture(CAMERA_INDEX)

    if not cap.isOpened():
        print("摄像头打开失败。")
        return None

    ret = False
    frame = None

    for _ in range(5):
        ret, frame = cap.read()
        time.sleep(0.1)

    cap.release()

    if not ret or frame is None:
        print("拍照失败。")
        return None

    cv2.imwrite(filename, frame)
    print(f"已拍照：{filename}")
    return filename


def ask_qwen_vision(image_path, prompt):
    try:
        with open(image_path, "rb") as f:
            image_base64 = base64.b64encode(f.read()).decode("utf-8")

        image_url = f"data:image/jpeg;base64,{image_base64}"

        response = client.chat.completions.create(
            model=VISION_MODEL_NAME,
            messages=[
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": image_url
                            }
                        },
                        {
                            "type": "text",
                            "text": prompt
                        }
                    ]
                }
            ],
            temperature=0.2,
            max_tokens=220
        )

        return response.choices[0].message.content.strip()

    except Exception as e:
        print("视觉模型调用失败：", e)
        return "我刚才看图失败了，请检查摄像头、网络或视觉模型名称。"


def handle_vision_command(user_text):
    user_text = normalize_text(user_text)

    vision_keywords = [
        "看看前面",
        "看一下前面",
        "前面有什么",
        "识别一下",
        "识别前方",
        "看一下这是什么",
        "这是什么",
        "看看这是什么",
        "看看这盆花",
        "检查植物",
        "植物状态",
        "花怎么样",
        "花卉状态",
        "拍照看看",
        "用摄像头看看"
    ]

    if not any(word in user_text for word in vision_keywords):
        return False

    speak("好的，我看一下。")

    image_path = capture_image()

    if image_path is None:
        speak("我没有打开摄像头，请检查摄像头连接。")
        return True

    prompt = """
你是家庭园艺陪伴机器人的视觉助手。
请观察图片，用简短中文回答，控制在80字以内。

如果画面中是普通物体：
请说出前方主要物体名称和大致位置。

如果画面中有植物：
请说明：
1. 可能是什么植物；
2. 叶片状态是否正常；
3. 光照看起来是否充足；
4. 给一句简单养护建议。

不要编造看不清的内容，看不清就说不太确定。
"""

    result = ask_qwen_vision(image_path, prompt)

    print("视觉分析结果：" + result)
    speak(result)

    return True


# ================== 定位与天气功能 ==================

def weather_code_to_text(code):
    weather_map = {
        0: "晴",
        1: "大部晴朗",
        2: "局部多云",
        3: "阴天",
        45: "有雾",
        48: "雾凇",
        51: "小毛毛雨",
        53: "中等毛毛雨",
        55: "大毛毛雨",
        61: "小雨",
        63: "中雨",
        65: "大雨",
        71: "小雪",
        73: "中雪",
        75: "大雪",
        80: "阵雨",
        81: "中等阵雨",
        82: "强阵雨",
        95: "雷雨",
        96: "雷雨伴小冰雹",
        99: "雷雨伴大冰雹"
    }

    return weather_map.get(code, "天气情况不确定")


def get_location_by_ip():
    """
    先根据网络 IP 获取大致位置。
    """
    try:
        url = "http://ip-api.com/json/?lang=zh-CN&fields=status,message,country,regionName,city,lat,lon,query"
        response = requests.get(url, timeout=6)
        data = response.json()

        if data.get("status") != "success":
            print("IP 定位失败：", data)
            return None

        location = {
            "country": data.get("country", ""),
            "region": data.get("regionName", ""),
            "city": data.get("city", ""),
            "lat": data.get("lat"),
            "lon": data.get("lon"),
            "ip": data.get("query", "")
        }

        print("IP 定位结果：", location)
        return location

    except Exception as e:
        print("IP 定位失败：", e)
        return None


def get_city_coordinate(city):
    """
    再根据城市名获取城市经纬度。
    """
    try:
        url = "https://geocoding-api.open-meteo.com/v1/search"

        params = {
            "name": city,
            "count": 1,
            "language": "zh",
            "format": "json"
        }

        response = requests.get(url, params=params, timeout=8)
        data = response.json()

        results = data.get("results", [])

        if not results:
            print("城市经纬度查询失败：", data)
            return None

        result = results[0]

        coordinate = {
            "city": result.get("name", city),
            "country": result.get("country", ""),
            "admin1": result.get("admin1", ""),
            "lat": result.get("latitude"),
            "lon": result.get("longitude")
        }

        print("城市经纬度结果：", coordinate)
        return coordinate

    except Exception as e:
        print("城市经纬度查询失败：", e)
        return None


def get_weather_by_latlon(lat, lon):
    """
    根据经纬度查询实时天气。
    """
    try:
        url = "https://api.open-meteo.com/v1/forecast"

        params = {
            "latitude": lat,
            "longitude": lon,
            "current": "temperature_2m,relative_humidity_2m,precipitation,weather_code,wind_speed_10m",
            "timezone": "auto"
        }

        response = requests.get(url, params=params, timeout=8)
        data = response.json()

        current = data.get("current", {})

        if not current:
            print("天气接口返回异常：", data)
            return None

        weather = {
            "temperature": current.get("temperature_2m"),
            "humidity": current.get("relative_humidity_2m"),
            "precipitation": current.get("precipitation"),
            "weather_code": current.get("weather_code"),
            "wind_speed": current.get("wind_speed_10m")
        }

        print("天气查询结果：", weather)
        return weather

    except Exception as e:
        print("天气查询失败：", e)
        return None


def build_weather_reply(location_name, weather):
    weather_text = weather_code_to_text(weather.get("weather_code"))
    temp = weather.get("temperature")
    humidity = weather.get("humidity")
    precipitation = weather.get("precipitation")
    wind_speed = weather.get("wind_speed")

    plant_advice = "光照合适的话，可以让耐晒植物晒一会儿。"

    if precipitation is not None and precipitation > 0:
        plant_advice = "现在有降水，不建议把盆栽搬到室外。"
    elif temp is not None and temp >= 32:
        plant_advice = "温度偏高，绿萝等耐阴植物别暴晒。"
    elif temp is not None and temp <= 5:
        plant_advice = "温度较低，注意给怕冷植物保温。"

    reply = (
        f"定位到{location_name}。"
        f"现在{weather_text}，约{temp}℃，湿度{humidity}%，"
        f"风速{wind_speed}公里每小时。{plant_advice}"
    )

    return reply


def handle_weather_command(user_text):
    user_text = normalize_text(user_text)

    weather_keywords = [
        "天气",
        "气温",
        "温度",
        "下雨",
        "降雨",
        "冷吗",
        "热吗",
        "适合晒花",
        "适合浇水",
        "适合搬出去"
    ]

    if not any(word in user_text for word in weather_keywords):
        return False

    speak("好的，我先定位，再帮你查天气。")

    # 方案一：如果设置了固定城市，直接用固定城市
    if HOME_CITY:
        city = HOME_CITY
        coordinate = get_city_coordinate(city)

        if coordinate:
            weather = get_weather_by_latlon(coordinate["lat"], coordinate["lon"])

            if weather:
                location_name = coordinate.get("admin1", "") + coordinate.get("city", city)
                reply = build_weather_reply(location_name, weather)
                print("小园：" + reply)
                speak(reply)
                return True

    # 方案二：先根据 IP 定位，再根据城市查天气
    if ENABLE_IP_LOCATION:
        location = get_location_by_ip()

        if location:
            city = location.get("city", "")
            region = location.get("region", "")
            ip_lat = location.get("lat")
            ip_lon = location.get("lon")

            # 优先根据城市名重新查经纬度
            if city:
                coordinate = get_city_coordinate(city)

                if coordinate:
                    weather = get_weather_by_latlon(coordinate["lat"], coordinate["lon"])

                    if weather:
                        location_name = region + city
                        reply = build_weather_reply(location_name, weather)
                        print("小园：" + reply)
                        speak(reply)
                        return True

            # 如果城市查不到，就退回 IP 经纬度
            if ip_lat is not None and ip_lon is not None:
                weather = get_weather_by_latlon(ip_lat, ip_lon)

                if weather:
                    location_name = region + city if city else "当前位置"
                    reply = build_weather_reply(location_name, weather)
                    print("小园：" + reply)
                    speak(reply)
                    return True

    reply = "我暂时没定位到你的位置。你可以说：南京今天天气怎么样。"
    print("小园：" + reply)
    speak(reply)
    return True


# ================== 联网搜索功能 ==================

def ask_qwen_web(user_text):
    try:
        response = client.responses.create(
            model=WEB_MODEL_NAME,
            input=f"""
你是家庭陪伴机器人小园。
请联网搜索后回答用户问题。
回答要简短、自然，控制在80字以内。
用户问题：{user_text}
""",
            tools=[
                {"type": "web_search"}
            ]
        )

        reply = getattr(response, "output_text", None)

        if reply:
            return reply.strip()

        return str(response)

    except Exception as e:
        print("联网搜索失败：", e)
        return "我刚才联网搜索失败了，可以稍后再试一下。"


def handle_web_command(user_text):
    user_text = normalize_text(user_text)

    web_keywords = [
        "联网",
        "上网",
        "网上",
        "搜索",
        "查一下",
        "帮我查",
        "最新",
        "新闻",
        "资料",
        "找一下"
    ]

    if not any(word in user_text for word in web_keywords):
        return False

    speak("好的，我帮你联网查一下。")

    reply = ask_qwen_web(user_text)

    print("小园：" + reply)
    speak(reply)

    return True


def handle_online_music_command(user_text):
    user_text = normalize_text(user_text)

    online_music_keywords = [
        "联网播放",
        "网上播放",
        "在线播放",
        "网上找歌",
        "帮我找一首",
        "搜索歌曲",
        "找音乐"
    ]

    if not any(word in user_text for word in online_music_keywords):
        return False

    query = user_text

    for word in online_music_keywords:
        query = query.replace(word, "")

    query = query.replace("音乐", "").replace("歌曲", "").replace("歌", "").strip()

    if not query:
        query = "舒缓轻音乐"

    search_text = quote(query + " 在线播放 音乐")
    url = f"https://www.bing.com/search?q={search_text}"

    webbrowser.open(url)

    reply = f"我帮你打开了“{query}”的在线音乐搜索页面。"
    print("小园：" + reply)
    speak(reply)

    return True


# ================== 录音与识别 ==================

def calc_rms(audio):
    return float(np.sqrt(np.mean(audio ** 2)))


def record_fixed_audio(filename="wake.wav", duration=2.5, samplerate=48000):
    audio = sd.rec(
        int(duration * samplerate),
        samplerate=samplerate,
        channels=1,
        dtype="float32",
        device=AUDIO_INPUT_DEVICE
    )
    sd.wait()

    rms = calc_rms(audio)

    if rms < WAKE_MIN_RMS:
        return None, rms

    sf.write(filename, audio, samplerate)
    return filename, rms


def record_until_silence(
    filename="input.wav",
    samplerate=48000,
    silence_duration=1.3,
    max_duration=10,
    listen_timeout=10,
    threshold=VOICE_THRESHOLD
):
    print("我在听，请说话...")

    frames = []
    state = {
        "has_voice": False,
        "last_voice_time": None,
        "start_time": time.time()
    }

    def audio_callback(indata, frame_count, time_info, status):
        volume = np.sqrt(np.mean(indata ** 2))

        if volume > threshold:
            state["has_voice"] = True
            state["last_voice_time"] = time.time()

        if state["has_voice"]:
            frames.append(indata.copy())

    stream = sd.InputStream(
        samplerate=samplerate,
        channels=1,
        dtype="float32",
        callback=audio_callback,
        device=AUDIO_INPUT_DEVICE
    )

    stream.start()

    while True:
        now = time.time()

        if not state["has_voice"]:
            if now - state["start_time"] > listen_timeout:
                stream.stop()
                stream.close()
                print("等待超时，回到待命。")
                return None

        if state["has_voice"]:
            if now - state["last_voice_time"] > silence_duration:
                break

        if now - state["start_time"] > max_duration:
            break

        time.sleep(0.1)

    stream.stop()
    stream.close()

    if len(frames) == 0:
        return None

    audio = np.concatenate(frames, axis=0)

    duration = len(audio) / samplerate
    rms = calc_rms(audio)

    print(f"本次指令音量 RMS：{rms:.4f}，时长：{duration:.2f} 秒")

    if duration < 0.4 or rms < COMMAND_MIN_RMS:
        print("录音太短或音量太低。")
        return None

    sf.write(filename, audio, samplerate)
    return filename


def speech_to_text(filename):
    try:
        segments, info = asr_model.transcribe(
            filename,
            language="zh",
            beam_size=1,
            vad_filter=True,
            condition_on_previous_text=False
        )

        text = ""
        for segment in segments:
            text += segment.text

        return text.strip()

    except Exception as e:
        print("语音识别失败：", e)
        return ""


# ================== TTS ==================
async def text_to_speech_async(text, filename):
    communicate = edge_tts.Communicate(
        text=text,
        voice="zh-CN-XiaoxiaoNeural"
    )
    await communicate.save(filename)

def speak(text):
    print("小园播报：" + text)

    try:
        if pygame.mixer.get_init():
            pygame.mixer.music.stop()
            pygame.mixer.quit()
    except Exception:
        pass

    filename = f"reply_{uuid.uuid4().hex}.mp3"

    try:
        asyncio.run(asyncio.wait_for(text_to_speech_async(text, filename), timeout=8))

        # 优先hw:0,0，失败自动切系统默认声卡，避免找不到设备
        try:
            pygame.mixer.init(devicename="hw:0,0")
        except Exception as e:
            print(f"hw:0,0占用，切换默认声卡，报错：{e}")
            pygame.mixer.init()

        pygame.mixer.music.load(filename)
        pygame.mixer.music.set_volume(0.7)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            time.sleep(0.1)

    except Exception as e:
        print("语音播报失败，但程序继续运行：", e)
    finally:
        # 强制关闭音频，释放声卡给录音
        try:
            pygame.mixer.music.stop()
            pygame.mixer.music.unload()
            pygame.mixer.quit()
        except:
            pass
        time.sleep(0.5)  # 延长等待，让ALSA完全释放声卡
        if os.path.exists(filename):
            try:
                os.remove(filename)
            except PermissionError:
                pass


# ================== 普通大模型问答 ==================

def ask_qwen(user_text):
    try:
        messages.append({
            "role": "user",
            "content": user_text
        })

        response = client.chat.completions.create(
            model=MODEL_NAME,
            messages=messages,
            temperature=0.5,
            max_tokens=120
        )

        reply = response.choices[0].message.content

        messages.append({
            "role": "assistant",
            "content": reply
        })

        return reply

    except Exception as e:
        print("普通大模型调用失败：", e)
        return "我刚才连接大模型失败了，请检查网络或 API Key。"


# ================== 本地命令分发 ==================

def handle_local_command(user_text):
    user_text = normalize_text(user_text)

    if handle_vision_command(user_text):
        return True

    if handle_online_music_command(user_text):
        return True

    if handle_music_command(user_text):
        return True

    if handle_contact_command(user_text):
        return True

    # 天气单独走 IP 定位 + 城市天气
    if handle_weather_command(user_text):
        return True

    if handle_web_command(user_text):
        return True

    return False


# ================== 唤醒与主流程 ==================

def wait_for_wake_word():
    print("\n小园正在待命。请说：小圆，该干活了")

    while True:
        audio_file, rms = record_fixed_audio(duration=2.5)

        if audio_file is None:
            continue

        text = speech_to_text(audio_file)

        if text:
            print(f"监听到：{text} | 音量：{rms:.4f}")
            print("标准化监听：" + normalize_text(text))

        if is_exit_word(text):
            print("检测到关闭程序。")
            speak("好的，我先关闭啦。")
            shutdown_program()

        if is_wake_word(text):
            print("唤醒成功。")
            return


def main():
    rclpy.init()
    # 创建ROS发布节点
    voice_pub_node = Node("voice_cmd_publisher")
    action_pub = voice_pub_node.create_publisher(String, "/robot_action_cmd", 10)

    ensure_contacts_file()
    ensure_music_dir()

    print("当前音频设备列表：")
    print(sd.query_devices())

    print("小园语音陪伴系统已启动。")
    print("唤醒词：小圆，该干活了")
    print("可说：今天天气怎么样、看看前面有什么、播放音乐、联系妈妈")
    print("说“退下吧 / 休息吧”回到待命。")
    print("说“关闭程序 / 退出程序 / 关掉小园”退出。")

    speak("你好呀，我是小园。我会先待命，听到你叫我再开始工作。")

    while True:
        wait_for_wake_word()

        speak("我在，请说。")

        while True:
            audio_file = record_until_silence()

            if audio_file is None:
                print("小园回到待命状态。")
                break

            user_text = speech_to_text(audio_file)
            print("你：" + user_text)

            user_text_norm = normalize_text(user_text)
            print("标准化后：" + user_text_norm)

            # ===================== 新增ROS话题发布代码【这里！】 =====================
            cmd_msg = String()
            # 匹配抬花盆
            if "把花盆搬起来" in user_text_norm or "搬花盆" in user_text_norm or "抬花盆" in user_text_norm or "把花盆儿搬起来" in user_text_norm:
                cmd_msg.data = "lift_pot"
                print("检测到抬花盆指令，发布 lift_pot")
            # 匹配放下花盆
            elif "放下花盆" in user_text_norm or "放花盆" in user_text_norm:
                cmd_msg.data = "putdown_pot"
                print("检测到放下花盆指令，发布 putdown_pot")
            # 无搬运相关指令
            else:
                cmd_msg.data = "none"

            action_pub.publish(cmd_msg)
            voice_pub_node.get_logger().info(f"发布花盆动作指令: {cmd_msg.data}")
            # =======================================================================

            if not user_text_norm or is_noise_text(user_text_norm):
                speak("我刚才没有听清楚，可以再说一遍吗？")
                continue

            if is_exit_word(user_text_norm):
                speak("好的，我先关闭啦。")
                shutdown_program()

            if is_sleep_word(user_text_norm):
                speak("好的，那我先安静待命啦。")
                break

            if handle_local_command(user_text_norm):
                continue

            reply = ask_qwen(user_text)

            print("小园：" + reply)
            speak(reply)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("手动停止程序。")
        shutdown_program()