#!/usr/bin/env python3
"""Стенд для запуску simulation11 без малини.

Піднімає pty-пару і грає роль чекера ДЗ11 на UART: шле PKT_AMMO, PKT_CONFIG,
PKT_TELEMETRY і PKT_TARGET, приймає PKT_CONTROL, крутить спрощену фізику дрона
і наприкінці віддає PKT_RESULT. Друкує рядок "PTY=/dev/pts/N" - його треба
передати застосунку в --uart.

Другою роллю може бути мінімальний MAVLink GCS (HEARTBEAT/ATTITUDE/
GLOBAL_POSITION_INT + COMMAND_ACK на команду скиду). Він потрібен лише тоді,
коли справжнього чекера немає: передайте 0 як порт, щоб його вимкнути.

    ./tools/uart_sim.py <mavlink_port|0> [seconds]

Приклад разом зі справжнім MAVLink-чекером:

    checker-linux-arm64 14550 &
    ./tools/uart_sim.py 0 20 &
    ./simulation11 --uart /dev/pts/N --mavlink-port 14055 \
                   --mavlink-remote 127.0.0.1:14550
"""
import os, pty, struct, sys, time, socket, threading, math

MAGIC = b'\xA5\x5A'
PKT_TELEMETRY, PKT_TARGET, PKT_AMMO, PKT_RESULT, PKT_CONTROL, PKT_CONFIG = 1,2,3,4,5,6

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def encode(t, payload):
    body = bytes([t, len(payload)]) + payload
    return MAGIC + body + struct.pack('<H', crc16(body))

class Parser:
    def __init__(self): self.buf = b''
    def feed(self, data):
        self.buf += data; out = []
        while True:
            i = self.buf.find(MAGIC)
            if i < 0 or len(self.buf) < i+6: break
            t, ln = self.buf[i+2], self.buf[i+3]
            if len(self.buf) < i+6+ln: break
            body = self.buf[i+2:i+4+ln]
            crc = struct.unpack('<H', self.buf[i+4+ln:i+6+ln])[0]
            if crc16(body) == crc: out.append((t, self.buf[i+4:i+4+ln]))
            self.buf = self.buf[i+6+ln:]
        return out

# ---------------- MAVLink v1 ----------------
def x25(data):
    crc = 0xFFFF
    for b in data:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc

SEQ = [0]
def mav1(msgid, payload, crc_extra, sysid=255, compid=190):
    hdr = bytes([len(payload), SEQ[0] & 0xFF, sysid, compid, msgid])
    SEQ[0] += 1
    return b'\xFE' + hdr + payload + struct.pack('<H', x25(hdr + payload + bytes([crc_extra])))

def heartbeat():
    return mav1(0, struct.pack('<IBBBBB', 0, 6, 8, 0, 4, 3), 50)   # GCS

def attitude(t, roll, pitch, yaw):
    return mav1(30, struct.pack('<Iffffff', t, roll, pitch, yaw, 0, 0, 0), 39)

def command_ack(cmd, result=0):
    return mav1(77, struct.pack('<HB', cmd, result), 143)

def global_pos(t, lat, lon, alt_mm):
    return mav1(33, struct.pack('<IiiiihhhH', t, int(lat*1e7), int(lon*1e7), alt_mm, alt_mm, 0,0,0, 0xFFFF), 104)

def decode_any(data):
    """Дуже груба розпаковка v1/v2 заголовка -> (sysid, compid, msgid)."""
    out = []
    i = 0
    while i < len(data):
        if data[i] == 0xFE and i+8 <= len(data):
            ln = data[i+1]; out.append((data[i+3], data[i+4], data[i+5])); i += 8+ln
        elif data[i] == 0xFD and i+12 <= len(data):
            ln = data[i+1]; inc = data[i+2]
            msgid = data[i+7] | (data[i+8]<<8) | (data[i+9]<<16)
            out.append((data[i+5], data[i+6], msgid))
            i += 12 + ln + (13 if inc & 1 else 0)
        else:
            i += 1
    return out

MAVLINK_PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 14055
DURATION = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
# Скільки чекати на старт simulation11, перш ніж здатися.
HANDSHAKE_TIMEOUT = 300.0

stop = threading.Event()
rx_stats = {'heartbeat': 0, 'other': 0}

def gcs():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0.2)
    dst = ('127.0.0.1', MAVLINK_PORT)
    t0 = time.time()
    n = 0
    while not stop.is_set():
        now = int((time.time()-t0)*1000)
        s.sendto(heartbeat(), dst)
        s.sendto(attitude(now, 0.1*math.sin(n), 0.05, 1.57), dst)
        s.sendto(global_pos(now, 50.4501+0.0001*n, 30.5234, 100000), dst)
        n += 1
        deadline = time.time() + 1.0
        while time.time() < deadline and not stop.is_set():
            try:
                data, _ = s.recvfrom(2048)
            except socket.timeout:
                continue
            for (sid, cid, mid) in decode_any(data):
                if mid == 0:
                    rx_stats['heartbeat'] += 1
                elif mid == 76:
                    rx_stats['drop_cmd'] = rx_stats.get('drop_cmd', 0) + 1
                    if rx_stats['drop_cmd'] > 1:   # першу навмисно губимо, як чекер
                        s.sendto(command_ack(31010), dst)
                        rx_stats['acked'] = True
                else:
                    rx_stats['other'] += 1
    s.close()

# MAVLINK_PORT == 0 -> справжній чекер грає роль GCS, наш фейковий GCS вимкнено
if MAVLINK_PORT:
    threading.Thread(target=gcs, daemon=True).start()

master, slave = pty.openpty()
print(f"PTY={os.ttyname(slave)}", flush=True)
os.set_blocking(master, False)


def write(frame):
    """Поки дрон не приєднався, буфер pty нікому спорожняти, і неблокуючий
    os.write кидає BlockingIOError. Такий кадр просто губимо - це UART."""
    try:
        os.write(master, frame)
    except BlockingIOError:
        pass


def read():
    try:
        return os.read(master, 4096)
    except BlockingIOError:
        return b''


ammo = struct.pack('<16sffffB', b'VOG-17', 0.3, 0.004, 0.35, 3.0, 2)
cfg  = struct.pack('<ffffff', 10.0, 10.0, 1.0, 0.35, 0.02, 1.0)

p = Parser()
controls = 0

# Handshake: шлемо AMMO+CONFIG, доки дрон не відповість першим CONTROL. Одного
# разу мало - застосунок запускають руками, іноді через десяток секунд після нас,
# а в нього на прийом конфігу лише 5 с від моменту старту.
print("Чекаю на simulation11 (запустіть його з --uart на цей PTY)...", flush=True)
handshake_deadline = time.time() + HANDSHAKE_TIMEOUT
connected = False
while time.time() < handshake_deadline:
    write(encode(PKT_AMMO, ammo))
    write(encode(PKT_CONFIG, cfg))
    for (typ, pl) in p.feed(read()):
        if typ == PKT_CONTROL:
            controls += 1
            connected = True
    if connected:
        break
    time.sleep(0.2)

if not connected:
    print(f"Дрон не відповів за {HANDSHAKE_TIMEOUT:.0f} с - виходжу.", flush=True)
    sys.exit(1)

print("Дрон приєднався - починаю місію.", flush=True)
t0 = time.time()
x, y, dir_ = 0.0, 0.0, 0.0
speed = 0.0
last_accel, last_turn = 0.0, 0.0
last_step = time.time()
while time.time() - t0 < DURATION:
    t = time.time() - t0
    tel = struct.pack('<IfffffffB', int(t*1000), x, y, 100.0, speed*math.cos(dir_), speed*math.sin(dir_), speed, dir_, 1)
    write(encode(PKT_TELEMETRY, tel))
    write(encode(PKT_TARGET, struct.pack('<Bff', 0, 120.0, 40.0)))
    write(encode(PKT_TARGET, struct.pack('<Bff', 1, 200.0, -60.0)))
    for (typ, pl) in p.feed(read()):
        if typ == PKT_CONTROL:
            controls += 1
            last_accel, last_turn = struct.unpack('<ff', pl)
    # Інтегруємо РАЗ на ітерацію за реальним часом, а не раз на кожен CONTROL-кадр,
    # інакше позиція біжить швидше за оголошену speed і телеметрія стає неузгодженою.
    now_w = time.time()
    dt = now_w - last_step
    last_step = now_w
    speed = max(0.0, min(10.0, speed + last_accel*5.0*dt))
    dir_ += last_turn*1.0*dt
    x += speed*math.cos(dir_)*dt
    y += speed*math.sin(dir_)*dt
    time.sleep(0.05)

write(encode(PKT_RESULT, struct.pack('<BBfI', 1, 0, 1.23, int((time.time()-t0)*1000))))
time.sleep(0.5)
stop.set()
time.sleep(0.3)
print(f"RESULT controls_received={controls} mav_heartbeats_from_app={rx_stats['heartbeat']} drop_cmds={rx_stats.get('drop_cmd',0)} acked={rx_stats.get('acked',False)} other={rx_stats['other']}", flush=True)
