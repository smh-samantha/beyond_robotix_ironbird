import struct
import time

sensors = {
    1: {"pressure_pa": None, "temp_c": None, "last_ms": 0},
    2: {"pressure_pa": None, "temp_c": None, "last_ms": 0},
}
last_print_ms = 0
debug_enabled = True

def millis():
    return int(time.time() * 1000)

def bytes_to_float(b0, b1, b2, b3):
    return struct.unpack("<f", struct.pack("BBBB", b0, b1, b2, b3))[0]

def format_sensor_line(sensor_id):
    s = sensors.get(sensor_id)
    if not s or s["pressure_pa"] is None or s["temp_c"] is None:
        return "S%d OFF" % sensor_id
    age_ms = millis() - s["last_ms"]
    return "S%d P=%.0fPa T=%.1fC age=%dms" % (sensor_id, s["pressure_pa"], s["temp_c"], age_ms)

def update_from_frame(can_id, data):
    can_id = can_id & 0x1FFFFFFF
    msg_id = (can_id >> 8) & 0xFFFF
    if msg_id != 1043:
        return
    if len(data) < 9:
        return
    sensor_id = data[0]
    if sensor_id not in sensors:
        return
    p = bytes_to_float(data[1], data[2], data[3], data[4])
    t = bytes_to_float(data[5], data[6], data[7], data[8])
    sensors[sensor_id]["pressure_pa"] = p
    sensors[sensor_id]["temp_c"] = t
    sensors[sensor_id]["last_ms"] = millis()
    if debug_enabled:
        raw = " ".join(["%02X" % b for b in data[:9]])
        print("DBG id=0x%X can=0x%X sid=%d raw=%s" % (msg_id, can_id, sensor_id, raw))

def extract_data(msg):
    if hasattr(msg, "data"):
        try:
            return list(msg.data)
        except Exception:
            return []
    return []

def extract_len(msg):
    if hasattr(msg, "len"):
        return int(msg.len)
    if hasattr(msg, "dlc"):
        return int(msg.dlc)
    return None

def normalize_mav_iface(obj):
    if obj is None:
        return None
    if hasattr(obj, "recv_match") or hasattr(obj, "recvPacket") or hasattr(obj, "getPacket"):
        return obj
    inner_list = getattr(obj, "MAV", None)
    if inner_list:
        try:
            inner = inner_list[0]
            if hasattr(inner, "recv_match") or hasattr(inner, "recvPacket") or hasattr(inner, "getPacket"):
                return inner
        except Exception:
            pass
    return None

def get_mav_iface():
    obj = globals().get("MAV")
    iface = normalize_mav_iface(obj)
    if iface is not None:
        return iface
    for name in ("mav", "mavlink"):
        iface = normalize_mav_iface(globals().get(name))
        if iface is not None:
            return iface
    return None

mav = get_mav_iface()
if mav is None:
    raise RuntimeError("No MAVLink interface found (expected MAV, mav, or mavlink).")

print("sensor.py running")

def recv_any(mav_iface):
    try:
        if hasattr(mav_iface, "recv_match"):
            return mav_iface.recv_match(blocking=False)
        if hasattr(mav_iface, "recvPacket"):
            return mav_iface.recvPacket()
        if hasattr(mav_iface, "getPacket"):
            return mav_iface.getPacket()
    except Exception:
        return None
    return None

def get_msg_type(msg):
    if hasattr(msg, "get_type"):
        try:
            return msg.get_type()
        except Exception:
            return None
    if hasattr(msg, "name"):
        try:
            return str(msg.name)
        except Exception:
            return None
    return None

def is_can_frame(msg):
    mtype = get_msg_type(msg)
    if mtype in ("CAN_FRAME", "CANFD_FRAME"):
        return True
    return hasattr(msg, "id") and hasattr(msg, "data")

while True:
    msg = recv_any(mav)
    if isinstance(msg, (list, tuple)):
        msg = msg[0] if len(msg) > 0 else None
    if msg:
        if is_can_frame(msg):
            data = extract_data(msg)
            dlen = extract_len(msg)
            if dlen is not None:
                data = data[:dlen]
            update_from_frame(msg.id, data)

    now = millis()
    if now - last_print_ms > 500:
        last_print_ms = now
        print(format_sensor_line(1) + " | " + format_sensor_line(2))

    Script.Sleep(10)
