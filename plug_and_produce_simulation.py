#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plug & Produce Simulation
=========================

A Plug & Produce cell (two sources, two processes, a sink and a crane) with a
built-in Modbus TCP server. One file, Python standard library only, runs on
Windows, macOS and Linux. Nothing to install with pip.

START
    Windows:        py plug_and_produce_simulation.py
    macOS / Linux:  python3 plug_and_produce_simulation.py

    Needs Python 3.8 or newer with tkinter:
      Windows, macOS    the installer from python.org includes it
      macOS Homebrew    brew install python-tk
      Ubuntu / Debian   sudo apt install python3-tk
      Fedora            sudo dnf install python3-tkinter

CONNECT
    The simulation is a Modbus TCP server on 127.0.0.1, port 502.
      CMAS:    Source = modbus.tcp:127.0.0.1   Address = number in the table
      Python:  ModbusTcpClient('127.0.0.1')    (pip install pymodbus)

    Linux does not let normal users open port 502. There the simulation uses
    port 5020 instead and shows it in the window. Connect with
      ModbusTcpClient('127.0.0.1', port=5020)
    or allow port 502 once with
      sudo sysctl net.ipv4.ip_unprivileged_port_start=502

SIGNALS   (booleans: 0 = False, 1 = True)
    Address  Name       I/O      Meaning
    0        reset      Output   write 1 to stop both processes
    1        setX       Output   crane target X
    2        setY       Output   crane target Y
    3        vacuum     Output   gripper vacuum
    4        startp1    Output   run Process1
    5        startp2    Output   run Process2
    15       atX        Input    crane position X
    16       atY        Input    crane position Y
    17       source1    Input    part at Source1
    18       source2    Input    part at Source2
    19       p1running  Input    Process1 is running
    20       p2running  Input    Process2 is running
    21       p1sensor   Input    part in Process1
    22       p2sensor   Input    part in Process2
    23       button1    Input    Button 1 pressed
    24       button2    Input    Button 2 pressed

    All signals are holding registers (read 3, write 6 and 16). Coil and
    discrete-input function codes (1, 2, 5, 15) and input registers (4) map to
    the same addresses.

POSITIONS
    Parts are picked and placed at Y = 82. Larger Y is higher.
    Source1 X=55, Source2 X=158, Process1 X=450, Process2 X=650, Sink X=945.
    Travel sideways only at a safe height, or the crane collides.

BEHAVIOUR
    - The crane moves towards (setX, setY). When it has arrived, atX = setX and
      atY = setY. If it hits something it stops where it is.
    - vacuum = 1 picks up a part when the gripper is on top of it. vacuum = 0
      releases it. Released just above a station (at Y = 82) the part lands in
      the station; released anywhere else it falls and becomes scrap.
    - startp1 = 1 starts Process1. p1running is 1 while it runs and goes back
      to 0 when the process time is over; startp1 is then cleared to 0.
      Writing startp1 = 0 while it runs stops it.
    - Generate puts a part at a source. Reset clears the whole cell.
    - Click an output in the signal list to write it by hand.

OPTIONS
    --port 502               Modbus TCP port
    --host 127.0.0.1         use 0.0.0.0 to accept other computers
    --process-time 4         seconds a process runs
    --speed 250              crane speed in units per second
    --station process2=700   move a station (source1, source2, process1,
                             process2, sink); can be repeated
"""

import argparse
import math
import queue
import socket
import socketserver
import struct
import sys
import threading
import time

try:
    import tkinter as tk
    from tkinter import messagebox, simpledialog, ttk
    import tkinter.font as tkfont
except ImportError:
    tk = None

IS_WINDOWS = sys.platform.startswith("win")

TK_MISSING = """\
tkinter is not installed, so the simulation window cannot open.
  Windows, macOS:   install Python from https://www.python.org
  macOS Homebrew:   brew install python-tk
  Ubuntu / Debian:  sudo apt install python3-tk
  Fedora:           sudo dnf install python3-tkinter"""

# ---------------------------------------------------------------------------
# Modbus addresses
# ---------------------------------------------------------------------------

REG_RESET, SET_X, SET_Y, VACUUM, START_P1, START_P2 = 0, 1, 2, 3, 4, 5
AT_X, AT_Y, SOURCE1, SOURCE2 = 15, 16, 17, 18
P1_RUNNING, P2_RUNNING, P1_SENSOR, P2_SENSOR = 19, 20, 21, 22
BUTTON1, BUTTON2 = 23, 24

N_REGISTERS = 256
DEFAULT_PORT = 502
FALLBACK_PORT = 5020

SIGNALS = [
    (REG_RESET, "reset", "out"),
    (SET_X, "setX", "out"),
    (SET_Y, "setY", "out"),
    (VACUUM, "vacuum", "out"),
    (START_P1, "startp1", "out"),
    (START_P2, "startp2", "out"),
    (AT_X, "atX", "in"),
    (AT_Y, "atY", "in"),
    (SOURCE1, "source1", "in"),
    (SOURCE2, "source2", "in"),
    (P1_RUNNING, "p1running", "in"),
    (P2_RUNNING, "p2running", "in"),
    (P1_SENSOR, "p1sensor", "in"),
    (P2_SENSOR, "p2sensor", "in"),
    (BUTTON1, "button1", "in"),
    (BUTTON2, "button2", "in"),
]
SIGNAL_NAMES = {addr: name for addr, name, _ in SIGNALS}
INPUT_ADDRS = {addr for addr, _, io in SIGNALS if io == "in"}

# ---------------------------------------------------------------------------
# Geometry, in simulation units. Y is the height of the gripper's underside.
# ---------------------------------------------------------------------------

FLOOR_Y = 10
PEDESTAL_TOP = 57
PART_H = 25
PART_HALF_W = 25
PICK_Y = PEDESTAL_TOP + PART_H          # 82
WALL_TOP = 115
WALL_HALF_T = 4
SLOT_HALF_W = 52
PEDESTAL_HALF_W = 42
GRIPPER_HALF_W = 18
GRIPPER_H = 8
RAIL_Y = 350
X_MIN, X_MAX = 0, 1050
Y_MIN, Y_MAX = 0, 320
HOME_X, HOME_Y = 500, 150
ALIGN_TOL = 10          # how far off-centre a pick or place may be
GRASP_TOL = 3           # how far above a part the vacuum still grips
MAX_SAFE_DROP = 15      # a longer fall damages the part
MIN_BUTTON_PULSE = 0.5  # seconds button1/button2 stay 1 after a click
DELIVER_TIME = 1.6      # seconds a delivered part stays visible on the sink
EPS = 0.01

STATION_DEFS = [
    # key, name, kind, x, start register, running register, sensor register
    ("source1", "Source1", "source", 55, None, None, SOURCE1),
    ("source2", "Source2", "source", 158, None, None, SOURCE2),
    ("process1", "Process1", "process", 450, START_P1, P1_RUNNING, P1_SENSOR),
    ("process2", "Process2", "process", 650, START_P2, P2_RUNNING, P2_SENSOR),
    ("sink", "Sink", "sink", 945, None, None, None),
]
DEFAULT_X = {d[0]: d[3] for d in STATION_DEFS}


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def to_signed(v):
    return v - 0x10000 if v >= 0x8000 else v


def approach(cur, target, step):
    if abs(target - cur) <= step:
        return float(target)
    return cur + step if target > cur else cur - step


def overlaps(a, b):
    """Boxes are (x0, x1, y0, y1). Touching is not overlapping."""
    return (min(a[1], b[1]) - max(a[0], b[0]) > EPS
            and min(a[3], b[3]) - max(a[2], b[2]) > EPS)


# ---------------------------------------------------------------------------
# The cell
# ---------------------------------------------------------------------------

class Station:
    def __init__(self, key, name, kind, x, start_reg, running_reg, sensor_reg):
        self.key, self.name, self.kind, self.x = key, name, kind, x
        self.start_reg, self.running_reg, self.sensor_reg = start_reg, running_reg, sensor_reg
        self.running = False
        self.elapsed = 0.0


class Part:
    def __init__(self, pid, origin):
        self.id = pid
        self.origin = origin
        self.station = origin
        self.x = float(origin.x)
        self.y = float(PEDESTAL_TOP)     # underside of the part
        self.state = "resting"           # resting, carried, scrap, delivered
        self.processed = []
        self.fade = 0.0

    def box(self):
        return (self.x - PART_HALF_W, self.x + PART_HALF_W, self.y, self.y + PART_H)


class Cell:
    """All simulation state. Safe to call from the GUI and the Modbus threads."""

    def __init__(self, process_time=4.0, speed=250.0, layout=None):
        self.lock = threading.RLock()
        self.events = queue.Queue()
        self.reg = [0] * N_REGISTERS
        self.process_time = process_time
        self.speed = speed
        self.buttons = [0, 0]
        self.set_layout(layout or {})

    def log(self, text):
        self.events.put((time.strftime("%H:%M:%S"), text))

    # -- layout ------------------------------------------------------------

    @staticmethod
    def check_layout(layout):
        xs = dict(DEFAULT_X)
        for key, x in layout.items():
            if key not in DEFAULT_X:
                return "Unknown station '%s' (use %s)" % (key, ", ".join(DEFAULT_X))
            xs[key] = x
        for key, x in xs.items():
            if not 30 <= x <= 1000:
                return "%s X must be between 30 and 1000" % key
        order = sorted(xs.items(), key=lambda kv: kv[1])
        for (k1, x1), (k2, x2) in zip(order, order[1:]):
            if x2 - x1 < 100:
                return "%s and %s must be at least 100 apart" % (k1, k2)
        return None

    def set_layout(self, layout):
        xs = dict(DEFAULT_X)
        xs.update(layout)
        with self.lock:
            self.stations = []
            for key, name, kind, _, sreg, rreg, snreg in STATION_DEFS:
                self.stations.append(Station(key, name, kind, xs[key], sreg, rreg, snreg))
            self.walls = []
            self.static = [((-1e6, 1e6, -1e6, FLOOR_Y), "the floor")]
            for st in self.stations:
                self.static.append(((st.x - PEDESTAL_HALF_W, st.x + PEDESTAL_HALF_W,
                                     FLOOR_Y, PEDESTAL_TOP), st.name))
                if st.kind != "sink":
                    for wx in (st.x - SLOT_HALF_W, st.x + SLOT_HALF_W):
                        box = (wx - WALL_HALF_T, wx + WALL_HALF_T, FLOOR_Y, WALL_TOP)
                        self.walls.append(box)
                        self.static.append((box, "a wall of " + st.name))
            self.reset(quiet=True)

    def station(self, key):
        return next(st for st in self.stations if st.key == key)

    @property
    def processes(self):
        return [st for st in self.stations if st.kind == "process"]

    # -- commands from the GUI --------------------------------------------

    def reset(self, quiet=False):
        with self.lock:
            self.parts = []
            self.carried = None
            self.next_id = 1
            self.x, self.y = float(HOME_X), float(HOME_Y)
            for i in range(N_REGISTERS):
                self.reg[i] = 0
            self.reg[SET_X], self.reg[SET_Y] = HOME_X, HOME_Y
            for st in self.stations:
                st.running, st.elapsed = False, 0.0
            self.delivered = self.scrapped = self.collisions = 0
            self.blocked = False
            self.blocked_target = None
            self.last_collision = -1e9
            self.collision_text = ""
            self.process_reset = False
            self._write_inputs()
        if not quiet:
            self.log("Cell reset")

    def remove_scrap(self):
        with self.lock:
            n = sum(1 for p in self.parts if p.state == "scrap")
            self.parts = [p for p in self.parts if p.state != "scrap"]
        self.log("Removed %d scrap part(s)" % n)

    def generate(self, key):
        with self.lock:
            st = self.station(key)
            box = (st.x - PART_HALF_W, st.x + PART_HALF_W, PEDESTAL_TOP, PICK_Y)
            if any(overlaps(box, p.box()) for p in self.parts if p.state in ("resting", "scrap")):
                msg = "%s is occupied, no part generated" % st.name
            elif any(overlaps(box, b) for b in self._crane_boxes(self.x, self.y)):
                msg = "The crane is in the way at %s, no part generated" % st.name
            else:
                part = Part(self.next_id, st)
                self.next_id += 1
                self.parts.append(part)
                self._write_inputs()
                msg = "Part #%d generated at %s" % (part.id, st.name)
        self.log(msg)

    def set_button(self, index, value):
        with self.lock:
            self.buttons[index] = value
            self.reg[BUTTON1 + index] = value

    # -- Modbus access -----------------------------------------------------

    def read(self, addr, count):
        with self.lock:
            return self.reg[addr:addr + count]

    def write(self, addr, value, who="Modbus"):
        value &= 0xFFFF
        with self.lock:
            if addr in INPUT_ADDRS:
                return          # inputs belong to the simulation
            old = self.reg[addr]
            name = SIGNAL_NAMES.get(addr)
            if name and (old != value or (addr == REG_RESET and value)):
                self.log("%s: %s = %d" % (who, name, to_signed(value)))
            self.reg[addr] = value
            if addr == REG_RESET:
                self.reg[addr] = 0
                if value:
                    self.process_reset = True
            elif addr == VACUUM:
                # Grip or release at once: clients often write the next move
                # straight after the vacuum, before the next simulation step.
                self._update_vacuum()
                self._write_inputs()
            elif addr in (START_P1, START_P2) and value:
                st = next(s for s in self.processes if s.start_reg == addr)
                if not st.running:
                    # Report running at once, so a client that reads straight
                    # after writing never sees a stale 0.
                    self.reg[st.running_reg] = 1

    # -- simulation step ---------------------------------------------------

    def step(self, dt):
        with self.lock:
            self._update_processes(dt)
            self._update_vacuum()
            self._update_crane(dt)
            for p in list(self.parts):
                if p.state == "delivered":
                    p.fade -= dt
                    if p.fade <= 0:
                        self.parts.remove(p)
            self._write_inputs()

    def part_at(self, st):
        return next((p for p in self.parts if p.state == "resting" and p.station is st), None)

    def _update_processes(self, dt):
        reset, self.process_reset = self.process_reset, False
        for st in self.processes:
            start = self.reg[st.start_reg] != 0
            if reset:
                if st.running:
                    self.log("%s stopped by reset" % st.name)
                st.running = False
                self.reg[st.start_reg] = 0
            elif st.running:
                if not start:
                    st.running = False
                    self.log("%s stopped before it was finished" % st.name)
                else:
                    st.elapsed += dt
                    if st.elapsed >= self.process_time:
                        st.running = False
                        self.reg[st.start_reg] = 0
                        part = self.part_at(st)
                        if part:
                            part.processed.append(st.name)
                            self.log("%s finished, part #%d processed" % (st.name, part.id))
                        else:
                            self.log("%s finished (it was empty)" % st.name)
            elif start:
                st.running, st.elapsed = True, 0.0
                part = self.part_at(st)
                self.log("%s started %s" % (st.name, "with part #%d" % part.id if part else "(empty)"))

    def _crane_boxes(self, x, y):
        boxes = [(x - GRIPPER_HALF_W, x + GRIPPER_HALF_W, y, y + GRIPPER_H)]
        if self.carried is not None:
            boxes.append((x - PART_HALF_W, x + PART_HALF_W, y - PART_H, y))
        return boxes

    def _obstacles(self):
        obs = list(self.static)
        for p in self.parts:
            if p.state == "resting":
                obs.append((p.box(), "part #%d in %s" % (p.id, p.station.name)))
            elif p.state == "scrap":
                obs.append((p.box(), "scrap part #%d" % p.id))
        return obs

    def _hit(self, x, y, obstacles):
        boxes = self._crane_boxes(x, y)
        for box, name in obstacles:
            if any(overlaps(b, box) for b in boxes):
                return name
        return None

    def _update_crane(self, dt):
        tx = clamp(to_signed(self.reg[SET_X]), X_MIN, X_MAX)
        ty = clamp(to_signed(self.reg[SET_Y]), Y_MIN, Y_MAX)
        step = self.speed * dt
        sx, sy = self.x, self.y
        nx, ny = approach(sx, tx, step), approach(sy, ty, step)
        dx, dy = nx - sx, ny - sy
        if dx == 0 and dy == 0:
            self.blocked = False
            return
        obstacles = self._obstacles()
        stuck = self._hit(sx, sy, obstacles) is not None   # let it move out of an overlap
        n = max(1, int(math.ceil(max(abs(dx), abs(dy)) / 2.0)))
        fx, fy, hit = sx, sy, None
        for i in range(1, n + 1):
            px = nx if i == n else sx + dx * i / n
            py = ny if i == n else sy + dy * i / n
            if not stuck:
                hit = self._hit(px, py, obstacles)
                if hit:
                    break
            fx, fy = px, py
        self.x, self.y = fx, fy
        if self.carried is not None:
            self.carried.x, self.carried.y = fx, fy - PART_H
        if hit:
            self.last_collision = time.monotonic()
            if not self.blocked or self.blocked_target != (tx, ty):
                self.collisions += 1
                self.collision_text = "the crane hit %s" % hit
                self.log("COLLISION at X=%d Y=%d: the crane hit %s and stopped"
                         % (round(fx), round(fy), hit))
            self.blocked, self.blocked_target = True, (tx, ty)
        else:
            self.blocked = False

    def _update_vacuum(self):
        vacuum = self.reg[VACUUM] != 0
        if vacuum and self.carried is None:
            for p in self.parts:
                if (p.state == "resting" and abs(p.x - self.x) <= ALIGN_TOL
                        and -EPS <= self.y - (p.y + PART_H) <= GRASP_TOL):
                    src = p.station
                    p.state, p.station = "carried", None
                    p.x, p.y = self.x, self.y - PART_H
                    self.carried = p
                    msg = "Picked part #%d from %s" % (p.id, src.name)
                    if src.kind == "process" and src.running:
                        msg += " (warning: %s is still running)" % src.name
                    self.log(msg)
                    break
        elif not vacuum and self.carried is not None:
            self._release()

    def _release(self):
        p, self.carried = self.carried, None
        p.x, bottom = self.x, self.y - PART_H
        landing = FLOOR_Y
        for box, _ in self._obstacles():
            if (min(p.x + PART_HALF_W, box[1]) - max(p.x - PART_HALF_W, box[0]) > EPS
                    and box[3] <= bottom + EPS):
                landing = max(landing, box[3])
        drop = bottom - landing
        st = next((s for s in self.stations if abs(s.x - p.x) <= ALIGN_TOL), None)
        if st and abs(landing - PEDESTAL_TOP) < EPS and drop <= MAX_SAFE_DROP:
            p.x, p.y = float(st.x), float(PEDESTAL_TOP)
            if st.kind == "sink":
                p.state, p.fade = "delivered", DELIVER_TIME
                self.delivered += 1
                route = " -> ".join([p.origin.name] + p.processed + [st.name])
                self.log("Part #%d delivered (%s)" % (p.id, route))
            else:
                p.state, p.station = "resting", st
                msg = "Placed part #%d in %s" % (p.id, st.name)
                if st.kind == "process" and st.running:
                    msg += " (warning: %s is already running)" % st.name
                self.log(msg)
        else:
            p.y, p.state = landing, "scrap"
            self.scrapped += 1
            if st is None:
                why = "was released away from a station"
            elif abs(landing - PEDESTAL_TOP) >= EPS:
                why = "landed on top of something in %s" % st.name
            else:
                why = "was dropped from %d units above %s" % (round(drop), st.name)
            self.log("Part #%d %s and is damaged (scrap)" % (p.id, why))

    def _write_inputs(self):
        r = self.reg
        r[AT_X], r[AT_Y] = int(round(self.x)), int(round(self.y))
        for st in self.stations:
            if st.sensor_reg is not None:
                r[st.sensor_reg] = 1 if self.part_at(st) else 0
            if st.running_reg is not None:
                r[st.running_reg] = 1 if st.running else 0
        r[BUTTON1], r[BUTTON2] = self.buttons


# ---------------------------------------------------------------------------
# Modbus TCP server
# ---------------------------------------------------------------------------

def handle_pdu(cell, pdu):
    """Answer one Modbus request PDU. Returns the response PDU."""
    fc = pdu[0]

    def error(code):
        return bytes([fc | 0x80, code])

    try:
        if fc in (1, 2, 3, 4):
            addr, count = struct.unpack(">HH", pdu[1:5])
            if not 1 <= count <= (2000 if fc <= 2 else 125):
                return error(3)
            if addr + count > N_REGISTERS:
                return error(2)
            values = cell.read(addr, count)
            if fc >= 3:
                return bytes([fc, 2 * count]) + struct.pack(">%dH" % count, *values)
            bits = bytearray((count + 7) // 8)
            for i, v in enumerate(values):
                if v:
                    bits[i // 8] |= 1 << (i % 8)
            return bytes([fc, len(bits)]) + bytes(bits)
        if fc in (5, 6):
            addr, value = struct.unpack(">HH", pdu[1:5])
            if addr >= N_REGISTERS:
                return error(2)
            if fc == 5:
                if value not in (0x0000, 0xFF00):
                    return error(3)
                value = 1 if value else 0
            cell.write(addr, value)
            return pdu[:5]
        if fc in (15, 16):
            addr, count, nbytes = struct.unpack(">HHB", pdu[1:6])
            data = pdu[6:6 + nbytes]
            expected = (count + 7) // 8 if fc == 15 else 2 * count
            if count < 1 or nbytes != expected or len(data) != nbytes:
                return error(3)
            if addr + count > N_REGISTERS:
                return error(2)
            if fc == 15:
                values = [(data[i // 8] >> (i % 8)) & 1 for i in range(count)]
            else:
                values = struct.unpack(">%dH" % count, data)
            for i, v in enumerate(values):
                cell.write(addr + i, v)
            return pdu[:5]
        return error(1)
    except struct.error:
        return error(3)


def _recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


class ModbusHandler(socketserver.BaseRequestHandler):
    def handle(self):
        server, sock = self.server, self.request
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        server.client_change(+1, self.client_address[0])
        try:
            while True:
                header = _recv_exact(sock, 7)
                if header is None:
                    break
                tid, pid, length, unit = struct.unpack(">HHHB", header)
                if not 2 <= length <= 260:
                    break
                pdu = _recv_exact(sock, length - 1)
                if pdu is None:
                    break
                if pid != 0:
                    continue
                resp = handle_pdu(server.cell, pdu)
                sock.sendall(struct.pack(">HHHB", tid, 0, len(resp) + 1, unit) + resp)
        except OSError:
            pass
        finally:
            server.client_change(-1, self.client_address[0])


class ModbusServer(socketserver.ThreadingTCPServer):
    daemon_threads = True
    # On Windows SO_REUSEADDR would let two simulations share a port silently.
    allow_reuse_address = not IS_WINDOWS

    def __init__(self, address, cell):
        self.cell = cell
        self.clients = 0
        self._clients_lock = threading.Lock()
        socketserver.ThreadingTCPServer.__init__(self, address, ModbusHandler)

    def server_bind(self):
        if IS_WINDOWS and hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        socketserver.ThreadingTCPServer.server_bind(self)

    def client_change(self, delta, host):
        with self._clients_lock:
            self.clients += delta
            n = self.clients
        if delta > 0 and n == 1:
            self.cell.log("Modbus client connected from %s" % host)
        elif delta < 0 and n == 0:
            self.cell.log("Modbus client disconnected")


def start_server(cell, host, port, port_given):
    """Returns (server, error message)."""
    try:
        server = ModbusServer((host, port), cell)
    except PermissionError as e:
        if port_given or IS_WINDOWS:
            return None, "Could not open Modbus port %d: %s" % (port, e)
        try:
            server = ModbusServer((host, FALLBACK_PORT), cell)
        except OSError as e2:
            return None, "Could not open Modbus port %d or %d: %s" % (port, FALLBACK_PORT, e2)
        cell.log("Port %d needs administrator rights here, so port %d is used instead. "
                 "Connect with ModbusTcpClient('127.0.0.1', port=%d)"
                 % (port, FALLBACK_PORT, FALLBACK_PORT))
    except OSError as e:
        return None, ("Could not open Modbus port %d: %s\n\n"
                      "Is another simulation already running?" % (port, e))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, None


# ---------------------------------------------------------------------------
# Window
# ---------------------------------------------------------------------------

# Everything visible is drawn on canvases with explicit colours, so the window
# looks the same on Windows, macOS and Linux.
COL = {
    "stage": "#121820", "panel": "#0d1218", "text": "#dde5ec", "dim": "#7c8a98",
    "faint": "#46525f", "floor": "#1b222c", "floor_line": "#252e3a",
    "table": "#2b3542", "steel": "#7d8996", "rail": "#56616d", "yellow": "#fcc419",
    "cyan": "#3bd5f5", "red": "#ff5c5c", "orange": "#ffa94d", "green": "#51cf66",
    "button": "#252f3b", "button_edge": "#44515f", "bar": "#0f141b",
}
STATION_ACCENT = {"source1": "#4dabf7", "source2": "#f06595", "process1": "#38d9a9",
                  "process2": "#9775fa", "sink": "#51cf66"}
PART_COLOUR = {"source1": "#4dabf7", "source2": "#f06595"}
BOOLEAN_ADDRS = {REG_RESET, VACUUM, START_P1, START_P2} | (INPUT_ADDRS - {AT_X, AT_Y})

# The 3D view. Simulation X and Y are unchanged; Z is depth (positive towards
# the viewer) and only affects the picture, never the physics.
TABLE_HALF_D = 45
WALL_HALF_D = 60
PART_HALF_D = 20
GANTRY_Z = 95
GANTRY_Y = 365
SCENE = ((-70, 1100), (0, 400), (-290, 240))
PERSPECTIVE = 2600.0
VIEWS = {"3D": (-24.0, 24.0), "Front": (0.0, 0.0), "Top": (0.0, 78.0)}
_l = (-0.35, 0.85, 0.45)
LIGHT = tuple(v / math.sqrt(sum(c * c for c in _l)) for v in _l)
BOX_FACES = (   # normal, corner indices into (x0|x1, y0|y1, z0|z1)
    ((0, 1, 0), ((0, 1, 0), (1, 1, 0), (1, 1, 1), (0, 1, 1))),
    ((0, 0, 1), ((0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1))),
    ((0, 0, -1), ((1, 0, 0), (0, 0, 0), (0, 1, 0), (1, 1, 0))),
    ((1, 0, 0), ((1, 0, 1), (1, 0, 0), (1, 1, 0), (1, 1, 1))),
    ((-1, 0, 0), ((0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0))),
)
_shades = {}


def shade(colour, k):
    key = (colour, round(k, 2))
    if key not in _shades:
        r, g, b = (int(colour[i:i + 2], 16) for i in (1, 3, 5))
        _shades[key] = "#%02x%02x%02x" % tuple(max(0, min(255, int(c * key[1]))) for c in (r, g, b))
    return _shades[key]


class App:
    def __init__(self, root, cell, server, server_error, host):
        self.root, self.cell, self.server, self.host = root, cell, server, host
        self.port = server.server_address[1] if server else None
        self.u = max(1.0, float(root.tk.call("tk", "scaling")) / (96 / 72))
        self.hot = []                 # screen buttons: (x0, y0, x1, y1, press)
        self.held = None              # index of Button 1/2 being held
        self._btn_token = [0, 0]
        self._btn_since = [0.0, 0.0]
        self._panel_key = None
        self._panel_hover = None
        self.panel_rows = []
        self.yaw, self.pitch = VIEWS["3D"]
        self.zoom, self.pan = 1.0, [0.0, 0.0]
        self.drag = None

        root.title("Plug & Produce Simulation")
        root.configure(bg=COL["bar"])
        w = min(int(1440 * self.u), root.winfo_screenwidth() - 60)
        h = min(int(760 * self.u), root.winfo_screenheight() - 100)
        root.geometry("%dx%d" % (w, h))
        root.minsize(int(860 * self.u), int(500 * self.u))
        root.protocol("WM_DELETE_WINDOW", root.destroy)

        self._build_menu()
        self._build_widgets()

        if server:
            cell.log("Modbus TCP server listening on %s:%d" % (host, self.port))
        else:
            cell.log("NO MODBUS SERVER: " + server_error.replace("\n\n", " "))
            root.after(300, lambda: messagebox.showerror("Modbus server", server_error, parent=root))

        self.last = time.monotonic()
        self._tick()

    def px(self, n):
        return int(round(n * self.u))

    # -- layout ------------------------------------------------------------

    def _build_menu(self):
        menubar = tk.Menu(self.root)
        sim = tk.Menu(menubar, tearoff=0)
        sim.add_command(label="Reset cell", command=self.cell.reset)
        sim.add_command(label="Remove scrap parts", command=self.cell.remove_scrap)
        sim.add_separator()
        sim.add_command(label="Process time...", command=self._ask_process_time)
        sim.add_command(label="Crane speed...", command=self._ask_speed)
        sim.add_command(label="Station positions...", command=self._ask_layout)
        sim.add_separator()
        sim.add_command(label="Quit", command=self.root.destroy)
        menubar.add_cascade(label="Simulation", menu=sim)
        view = tk.Menu(menubar, tearoff=0)
        for name in VIEWS:
            view.add_command(label=name, command=lambda n=name: self._set_view(n))
        menubar.add_cascade(label="View", menu=view)
        helpmenu = tk.Menu(menubar, tearoff=0)
        helpmenu.add_command(label="Signals and connection...", command=self._show_help)
        menubar.add_cascade(label="Help", menu=helpmenu)
        self.root.config(menu=menubar)

    def _build_widgets(self):
        self.mono = tkfont.nametofont("TkFixedFont").copy()
        self.mono.configure(size=10)
        self.small = tkfont.nametofont("TkDefaultFont").copy()
        self.small.configure(size=9)
        self.head = tkfont.nametofont("TkDefaultFont").copy()
        self.head.configure(size=9, weight="bold")
        self.big = tkfont.nametofont("TkDefaultFont").copy()
        self.big.configure(size=18, weight="bold")

        panel = tk.Frame(self.root, bg=COL["panel"], width=self.px(300))
        panel.pack(side="right", fill="y")
        panel.pack_propagate(False)
        self.stage = tk.Canvas(self.root, bg=COL["stage"], highlightthickness=0, bd=0)
        self.stage.pack(side="left", fill="both", expand=True)
        c = self.stage
        c.bind("<ButtonPress-1>", self._on_press)
        c.bind("<B1-Motion>", self._on_drag)
        c.bind("<ButtonRelease-1>", self._on_release)
        c.bind("<Motion>", self._on_motion)
        c.bind("<Double-Button-1>", lambda e: self._set_view("3D") if not self._hot_at(e.x, e.y) else None)
        c.bind("<MouseWheel>", lambda e: self._zoom_by(1.1 if e.delta > 0 else 1 / 1.1))
        c.bind("<Button-4>", lambda e: self._zoom_by(1.1))
        c.bind("<Button-5>", lambda e: self._zoom_by(1 / 1.1))

        self.row_h = self.px(21)
        self.sig = tk.Canvas(panel, bg=COL["panel"], highlightthickness=0, bd=0,
                             height=self.row_h * (len(SIGNALS) + 2) + self.px(96))
        self.sig.pack(fill="x", padx=self.px(12), pady=(self.px(12), 0))
        self.sig.bind("<Button-1>", self._on_panel_click)
        self.sig.bind("<Motion>", self._on_panel_motion)

        tk.Label(panel, text="EVENTS", bg=COL["panel"], fg=COL["dim"], font=self.head,
                 anchor="w").pack(fill="x", padx=self.px(12), pady=(self.px(6), self.px(4)))
        box = tk.Frame(panel, bg=COL["panel"])
        box.pack(fill="both", expand=True, padx=(self.px(12), 0), pady=(0, self.px(12)))
        scroll = tk.Scrollbar(box, orient="vertical")
        self.logtext = tk.Text(box, wrap="word", relief="flat", bd=0, highlightthickness=0,
                               bg="#0a0e13", fg=COL["text"], font=self.small,
                               padx=self.px(6), pady=self.px(6), spacing1=1,
                               yscrollcommand=scroll.set, state="disabled", cursor="arrow")
        scroll.configure(command=self.logtext.yview)
        scroll.pack(side="right", fill="y")
        self.logtext.pack(side="left", fill="both", expand=True)
        for tag, colour in (("time", COL["faint"]), ("alert", COL["red"]), ("warn", COL["orange"]),
                            ("good", COL["green"]), ("io", "#8fb3cf")):
            self.logtext.tag_configure(tag, foreground=colour)

    # -- loop --------------------------------------------------------------

    def _tick(self):
        now = time.monotonic()
        dt = min(0.05, now - self.last)
        self.last = now
        self.cell.step(dt)
        self._draw_stage()
        self._draw_panel()
        self._pump_log()
        self.root.after(25, self._tick)

    def _pump_log(self):
        entries = []
        while True:
            try:
                entries.append(self.cell.events.get_nowait())
            except queue.Empty:
                break
        if not entries:
            return
        t = self.logtext
        at_end = t.yview()[1] > 0.98
        t.configure(state="normal")
        for stamp, msg in entries:
            try:
                print("%s  %s" % (stamp, msg), flush=True)
            except Exception:
                pass
            low = msg.lower()
            if msg.startswith(("COLLISION", "NO MODBUS")):
                tag = "alert"
            elif "scrap" in low or "warning" in low or "stopped before" in low:
                tag = "warn"
            elif "delivered" in low:
                tag = "good"
            elif msg.startswith(("Modbus:", "Panel:")):
                tag = "io"
            else:
                tag = ""
            t.insert("end", stamp + "  ", "time")
            t.insert("end", msg + "\n", tag)
        excess = int(t.index("end-1c").split(".")[0]) - 500
        if excess > 0:
            t.delete("1.0", "%d.0" % (excess + 1))
        t.configure(state="disabled")
        if at_end:
            t.see("end")

    # -- camera ------------------------------------------------------------

    def _view(self, X, Y, Z):
        """World point -> (u, v, depth) before perspective; depth grows towards the viewer."""
        x, y = X - 525.0, Y - 170.0
        u = x * self.ca - Z * self.sa
        w = x * self.sa + Z * self.ca
        return u, y * self.cp - w * self.sp, y * self.sp + w * self.cp

    def P(self, X, Y, Z):
        u, v, d = self._view(X, Y, Z)
        k = PERSPECTIVE / (PERSPECTIVE - d)
        return self.ox + u * k * self.s, self.oy - v * k * self.s

    def _setup_camera(self, W, H):
        a, p = math.radians(self.yaw), math.radians(self.pitch)
        self.ca, self.sa, self.cp, self.sp = math.cos(a), math.sin(a), math.cos(p), math.sin(p)
        us, vs = [], []
        for X in SCENE[0]:
            for Y in SCENE[1]:
                for Z in SCENE[2]:
                    u, v, d = self._view(X, Y, Z)
                    k = PERSPECTIVE / (PERSPECTIVE - d)
                    us.append(u * k)
                    vs.append(v * k)
        top, bottom = self.px(34), self.px(52)
        self.s = min((W - self.px(24)) / (max(us) - min(us)),
                     (H - top - bottom) / (max(vs) - min(vs))) * self.zoom
        self.ox = W / 2 - (max(us) + min(us)) / 2 * self.s + self.pan[0]
        self.oy = top + (H - top - bottom) / 2 + (max(vs) + min(vs)) / 2 * self.s + self.pan[1]

    def _facing(self, n, point):
        u, v, d = self._view(*point)
        nu = n[0] * self.ca - n[2] * self.sa
        nw = n[0] * self.sa + n[2] * self.ca
        nv = n[1] * self.cp - nw * self.sp
        nd = n[1] * self.sp + nw * self.cp
        return u * nu + v * nv + (d - PERSPECTIVE) * nd < 0

    def _set_view(self, name):
        self.yaw, self.pitch = VIEWS[name]
        self.zoom, self.pan = 1.0, [0.0, 0.0]

    def _zoom_by(self, f):
        self.zoom = clamp(self.zoom * f, 0.5, 4.0)

    # -- drawing primitives ------------------------------------------------

    def _flat(self, points):
        out = []
        for p in points:
            out.extend(self.P(*p))
        return out

    def _text_font(self, size, bold=False):
        return ("Helvetica", -max(9, int(size * self.s)), "bold" if bold else "normal")

    def _box(self, b, colour, extras=()):
        """Draw a box now: b = (x0, x1, y0, y1, z0, z1)."""
        c = self.stage
        xs, ys, zs = (b[0], b[1]), (b[2], b[3]), (b[4], b[5])
        for n, idx in BOX_FACES:
            corners = [(xs[i], ys[j], zs[k]) for i, j, k in idx]
            centre = tuple(sum(p[a] for p in corners) / 4 for a in range(3))
            if not self._facing(n, centre):
                continue
            k = 0.5 + 0.5 * max(0.0, n[0] * LIGHT[0] + n[1] * LIGHT[1] + n[2] * LIGHT[2])
            c.create_polygon(self._flat(corners), fill=shade(colour, k),
                             outline=shade(colour, k * 0.72), tags="dyn")
        for ex in extras:
            self._extra(*ex)

    def _extra(self, kind, points, colour, normal=None, arg=None):
        """Decoration drawn on a face; skipped when that face points away."""
        c = self.stage
        if normal is not None:
            centre = tuple(sum(p[a] for p in points) / len(points) for a in range(3))
            if not self._facing(normal, centre):
                return
        if kind == "poly":
            c.create_polygon(self._flat(points), fill=colour, outline="", tags="dyn")
        elif kind == "line":
            c.create_line(self._flat(points), fill=colour, width=arg or 1, tags="dyn")
        elif kind == "text":
            text, size, bold = arg
            c.create_text(*self.P(*points[0]), text=text, fill=colour,
                          font=self._text_font(size, bold), tags="dyn")
        elif kind == "dot":
            x, y = self.P(*points[0])
            r = arg * self.s
            c.create_oval(x - r, y - r, x + r, y + r, fill=colour, width=0, tags="dyn")

    def _screen_button(self, cx, cy, label, press, active=False, width=86):
        c = self.stage
        w, h = self.px(width) / 2, self.px(13)
        c.create_rectangle(cx - w, cy - h, cx + w, cy + h, tags="dyn",
                           fill=COL["yellow"] if active else COL["button"],
                           outline=COL["yellow"] if active else COL["button_edge"])
        c.create_text(cx, cy, text=label, font=self.small, tags="dyn",
                      fill="#111111" if active else COL["text"])
        self.hot.append((cx - w, cy - h, cx + w, cy + h, press))

    # -- the scene ---------------------------------------------------------

    def _draw_stage(self):
        c = self.stage
        W, H = max(c.winfo_width(), 50), max(c.winfo_height(), 50)
        c.delete("dyn")
        self.hot = []
        self._setup_camera(W, H)
        with self.cell.lock:
            self._draw_ground(self.cell)
            self._draw_objects(self.cell)
            self._draw_gantry(self.cell)
            self._draw_hud(self.cell, W, H)

    def _draw_ground(self, cell):
        c, P = self.stage, self.P
        c.create_polygon(self._flat([(-70, FLOOR_Y, -290), (1100, FLOOR_Y, -290),
                                     (1100, FLOOR_Y, 240), (-70, FLOOR_Y, 240)]),
                         fill=COL["floor"], outline=COL["floor_line"], tags="dyn")
        for gx in range(0, 1051, 50):
            c.create_line(*P(gx, FLOOR_Y, -290), *P(gx, FLOOR_Y, 130), fill=COL["floor_line"], tags="dyn")
        for gz in range(-250, 131, 50):
            c.create_line(*P(-70, FLOOR_Y, gz), *P(1100, FLOOR_Y, gz), fill=COL["floor_line"], tags="dyn")
        # safety marking in front of the cell
        c.create_polygon(self._flat([(-10, FLOOR_Y, 118), (1060, FLOOR_Y, 118),
                                     (1060, FLOOR_Y, 126), (-10, FLOOR_Y, 126)]),
                         fill="#8a6d10", outline="", tags="dyn")

        # X ruler on the floor
        c.create_line(*P(0, FLOOR_Y, 150), *P(1050, FLOOR_Y, 150), fill=COL["faint"], tags="dyn")
        for gx in range(0, 1051, 50):
            c.create_line(*P(gx, FLOOR_Y, 150), *P(gx, FLOOR_Y, 156 if gx % 100 else 160),
                          fill=COL["faint"], tags="dyn")
        for st in cell.stations:
            accent = STATION_ACCENT[st.key]
            c.create_line(*P(st.x, FLOOR_Y, 140), *P(st.x, FLOOR_Y, 166), fill=accent, width=2, tags="dyn")
            c.create_text(*P(st.x, FLOOR_Y, 178), text=str(st.x), fill=accent, tags="dyn",
                          font=self._text_font(12, True))
        c.create_text(*P(1080, FLOOR_Y, 150), text="X", fill=COL["dim"], tags="dyn",
                      font=self._text_font(12, True))

        # Y ruler at the front left corner
        base = (-28, 130)
        c.create_line(*P(base[0], 0, base[1]), *P(base[0], 320, base[1]), fill=COL["faint"], tags="dyn")
        for gy in range(0, 321, 50):
            c.create_line(*P(base[0], gy, base[1]), *P(base[0] - 8, gy, base[1]), fill=COL["faint"], tags="dyn")
            c.create_text(*P(base[0] - 12, gy, base[1]), text=str(gy), anchor="e", fill=COL["dim"],
                          font=self._text_font(10), tags="dyn")
        c.create_line(*P(base[0], PICK_Y, base[1]), *P(base[0] - 10, PICK_Y, base[1]),
                      fill=COL["cyan"], width=2, tags="dyn")
        c.create_text(*P(base[0] - 14, PICK_Y, base[1]), text=str(PICK_Y), anchor="e",
                      fill=COL["cyan"], font=self._text_font(10, True), tags="dyn")
        c.create_text(*P(base[0], 345, base[1]), text="Y", fill=COL["dim"],
                      font=self._text_font(12, True), tags="dyn")

        # shadow under the gripper
        half_w = PART_HALF_W if cell.carried else GRIPPER_HALF_W
        half_d = PART_HALF_D if cell.carried else 14
        x = cell.x
        c.create_polygon(self._flat([(x - half_w, FLOOR_Y, -half_d), (x + half_w, FLOOR_Y, -half_d),
                                     (x + half_w, FLOOR_Y, half_d), (x - half_w, FLOOR_Y, half_d)]),
                         fill="#0b0f14", outline="", tags="dyn")

    def _draw_objects(self, cell):
        """Everything on the floor plus the hoist, drawn far to near."""
        now = time.monotonic()
        reg = cell.reg
        items = []      # (depth, draw function)

        def add(bounds, colour, extras=()):
            centre = ((bounds[0] + bounds[1]) / 2, (bounds[2] + bounds[3]) / 2, (bounds[4] + bounds[5]) / 2)
            items.append((self._view(*centre)[2], bounds, colour, extras))

        for st in cell.stations:
            accent = STATION_ACCENT[st.key]
            x0, x1 = st.x - PEDESTAL_HALF_W, st.x + PEDESTAL_HALF_W
            front = TABLE_HALF_D + 0.5
            label = [("poly", [(x0, 50, front), (x1, 50, front), (x1, PEDESTAL_TOP, front),
                               (x0, PEDESTAL_TOP, front)], accent, (0, 0, 1)),
                     ("text", [(st.x - (5 if st.sensor_reg is not None else 0), 30, front)],
                      COL["text"], (0, 0, 1), (st.name, 12, True))]
            if st.sensor_reg is not None:
                label.append(("dot", [(x1 - 8, 30, front)],
                              COL["green"] if reg[st.sensor_reg] else "#3a4450", (0, 0, 1), 4))
            if st.kind == "sink":
                stripes = []
                for i in range(11):
                    zz = TABLE_HALF_D - 4 - ((i * 30 + now * 60) % 305)
                    stripes.append(("line", [(x0 + 3, PEDESTAL_TOP + 0.5, zz), (x1 - 3, PEDESTAL_TOP + 0.5, zz)],
                                    "#3b4655", (0, 1, 0), 2))
                add((x0, x1, FLOOR_Y, PEDESTAL_TOP, -270, TABLE_HALF_D), COL["table"], stripes + label)
                continue
            add((x0, x1, FLOOR_Y, PEDESTAL_TOP, -TABLE_HALF_D, TABLE_HALF_D), COL["table"], label)
            if st.kind == "source":
                add((st.x - 26, st.x + 26, PEDESTAL_TOP, 150, -84, -52), shade(accent, 0.75))
            else:
                housing = shade(accent, 0.55)
                frac = min(1.0, st.elapsed / max(cell.process_time, 1e-6)) if st.running else 0.0
                zf = -61.5
                meter = [("poly", [(st.x - 8, 25, zf), (st.x + 8, 25, zf), (st.x + 8, 165, zf),
                                   (st.x - 8, 165, zf)], "#141a21", (0, 0, 1))]
                if frac > 0:
                    meter.append(("poly", [(st.x - 8, 25, zf), (st.x + 8, 25, zf),
                                           (st.x + 8, 25 + 140 * frac, zf), (st.x - 8, 25 + 140 * frac, zf)],
                                  accent, (0, 0, 1)))
                add((st.x - 30, st.x + 30, FLOOR_Y, 190, -84, -62), housing, meter)
                add((st.x - 34, st.x + 34, 190, 214, -88, -34), housing)
                if st.running:
                    light = COL["orange"] if int(now * 4) % 2 else "#7a4a12"
                else:
                    light = "#2f9e44"
                add((st.x + 16, st.x + 28, 214, 244, -66, -54), light)

        for x0, x1, y0, y1 in cell.walls:
            add((x0, x1, y0, y1, -WALL_HALF_D, WALL_HALF_D), COL["steel"])

        for p in cell.parts:
            if p is not cell.carried:
                add(*self._part_box(p))
        for gx in (-40, 1090):
            for gz in (-GANTRY_Z, GANTRY_Z):
                add((gx - 8, gx + 8, FLOOR_Y, GANTRY_Y - 8, gz - 8, gz + 8), COL["rail"])

        # hoist: cable, block, gripper plate, suction pad and the carried part
        x, y = cell.x, cell.y
        hit = cell.blocked or now - cell.last_collision < 1.5
        vacuum = reg[VACUUM] != 0
        add((x - 2.5, x + 2.5, y + GRIPPER_H + 16, GANTRY_Y - 14, -2.5, 2.5), "#9aa5b1")
        add((x - 9, x + 9, y + GRIPPER_H, y + GRIPPER_H + 16, -9, 9), "#6c7784")
        add((x - GRIPPER_HALF_W, x + GRIPPER_HALF_W, y + 3, y + GRIPPER_H, -14, 14),
            COL["red"] if hit else "#c9d2da")
        add((x - GRIPPER_HALF_W + 3, x + GRIPPER_HALF_W - 3, y, y + 3, -11, 11),
            COL["cyan"] if vacuum else "#5b6672")
        if cell.carried is not None:
            add(*self._part_box(cell.carried))

        items.sort(key=lambda it: it[0])
        for _, bounds, colour, extras in items:
            self._box(bounds, colour, extras)

    def _part_box(self, p):
        x0, x1, y0, y1 = p.box()
        z = 0.0
        if p.state == "delivered":
            z = -240.0 * (1.0 - max(0.0, p.fade) / DELIVER_TIME)
        z0, z1 = z - PART_HALF_D, z + PART_HALF_D
        if p.state == "scrap":
            return (x0, x1, y0, y1, z0, z1), "#6b737c", ()
        extras = []
        if "Process2" in p.processed:
            band = "#ffd43b"
            extras += [("poly", [(x0, y0 + 8, z1 + 0.3), (x1, y0 + 8, z1 + 0.3), (x1, y0 + 14, z1 + 0.3),
                                 (x0, y0 + 14, z1 + 0.3)], band, (0, 0, 1)),
                       ("poly", [(x1 + 0.3, y0 + 8, z1), (x1 + 0.3, y0 + 8, z0), (x1 + 0.3, y0 + 14, z0),
                                 (x1 + 0.3, y0 + 14, z1)], band, (1, 0, 0)),
                       ("poly", [(x0 - 0.3, y0 + 8, z0), (x0 - 0.3, y0 + 8, z1), (x0 - 0.3, y0 + 14, z1),
                                 (x0 - 0.3, y0 + 14, z0)], band, (-1, 0, 0))]
        if "Process1" in p.processed:
            ring = [(p.x + 8 * math.cos(i * math.pi / 4), y1 + 0.3, z + 8 * math.sin(i * math.pi / 4))
                    for i in range(8)]
            extras.append(("poly", ring, "#10151c", (0, 1, 0)))
            extras.append(("poly", [(p.x - 8, y1 + 0.3, z1 + 0.3), (p.x + 8, y1 + 0.3, z1 + 0.3),
                                    (p.x + 8, y1 - 6, z1 + 0.3), (p.x - 8, y1 - 6, z1 + 0.3)],
                           "#10151c", (0, 0, 1)))
        return (x0, x1, y0, y1, z0, z1), PART_COLOUR[p.origin.key], extras

    def _draw_gantry(self, cell):
        x = cell.x
        y0, y1 = GANTRY_Y - 8, GANTRY_Y + 8
        self._box((-48, 1098, y0, y1, -GANTRY_Z - 8, -GANTRY_Z + 8), COL["rail"])
        self._box((x - 24, x + 24, GANTRY_Y - 14, y1, -22, 22), "#2b3441")
        self._box((-48, 1098, y0, y1, GANTRY_Z - 8, GANTRY_Z + 8), COL["rail"])
        stripes = []
        for i in range(-3, 4):
            zz = i * 26
            stripes.append(("poly", [(x - 14, y1 + 18.3, zz - 6), (x + 14, y1 + 18.3, zz + 6),
                                     (x + 14, y1 + 18.3, zz + 12), (x - 14, y1 + 18.3, zz)],
                            "#1d1d1d", (0, 1, 0)))
        self._box((x - 14, x + 14, y1, y1 + 18, -GANTRY_Z - 12, GANTRY_Z + 12), COL["yellow"], stripes)

    def _draw_hud(self, cell, W, H):
        c, P, reg = self.stage, self.P, cell.reg
        x, y = cell.x, cell.y

        tx = clamp(to_signed(reg[SET_X]), X_MIN, X_MAX)
        ty = clamp(to_signed(reg[SET_Y]), Y_MIN, Y_MAX)
        if (tx, ty) != (round(x), round(y)):
            c.create_line(*P(x, y, 0), *P(tx, ty, 0), fill="#2d7890", dash=(4, 4), tags="dyn")
            ax, ay = P(tx, ty, 0)
            r = self.px(8)
            c.create_oval(ax - r, ay - r, ax + r, ay + r, outline=COL["cyan"], width=2, tags="dyn")
            c.create_line(ax - r * 1.6, ay, ax + r * 1.6, ay, fill=COL["cyan"], tags="dyn")
            c.create_line(ax, ay - r * 1.6, ax, ay + r * 1.6, fill=COL["cyan"], tags="dyn")

        lx, ly = P(clamp(x, 60, 990), GANTRY_Y + 30, -GANTRY_Z - 20)
        ly -= self.px(18)
        item = c.create_text(lx, ly, text="X %d   Y %d" % (round(x), round(y)), fill=COL["text"],
                             font=self.mono, tags="dyn")
        bx0, by0, bx1, by1 = c.bbox(item)
        back = c.create_rectangle(bx0 - self.px(6), by0 - self.px(3), bx1 + self.px(6), by1 + self.px(3),
                                  fill="#0b0f14", outline=COL["faint"], tags="dyn")
        c.tag_lower(back, item)
        if reg[VACUUM]:
            vx, vy = P(x + GRIPPER_HALF_W + 8, y + 4, 14)
            c.create_text(vx, vy, text="VACUUM", anchor="w", fill=COL["cyan"], font=self.head, tags="dyn")

        pad = self.px(12)
        if self.server:
            n = self.server.clients
            dot = COL["green"] if n else COL["faint"]
            status = "MODBUS  %s:%d   %d client%s" % (self.host, self.port, n, "" if n == 1 else "s")
        else:
            dot, status = COL["red"], "NO MODBUS SERVER"
        r = self.px(4)
        cy = self.px(18)
        c.create_oval(pad - r, cy - r, pad + r, cy + r, fill=dot, width=0, tags="dyn")
        c.create_text(pad + self.px(10), cy, text=status, anchor="w", font=self.mono,
                      fill=COL["dim"] if self.server else COL["red"], tags="dyn")
        if cell.blocked or time.monotonic() - cell.last_collision < 1.5:
            item = c.create_text(W / 2, cy, text="COLLISION  -  " + cell.collision_text,
                                 fill="white", font=self.head, tags="dyn")
            bx0, by0, bx1, by1 = c.bbox(item)
            back = c.create_rectangle(bx0 - pad, by0 - self.px(5), bx1 + pad, by1 + self.px(5),
                                      fill="#b3261e", width=0, tags="dyn")
            c.tag_lower(back, item)

        for st in cell.stations:
            if st.kind == "source":
                gx, gy = P(st.x, FLOOR_Y, 232)
                self._screen_button(gx, gy, "Generate", lambda k=st.key: self.cell.generate(k), width=78)

        bar_top = H - self.px(46)
        c.create_rectangle(0, bar_top, W, H, fill=COL["bar"], width=0, tags="dyn")
        by = bar_top + self.px(23)
        bx = self.px(12) + self.px(43)
        self._screen_button(bx, by, "Button 1", lambda: self._hold(0), active=reg[BUTTON1] != 0)
        self._screen_button(bx + self.px(96), by, "Button 2", lambda: self._hold(1), active=reg[BUTTON2] != 0)
        self._screen_button(bx + self.px(192), by, "Reset", self.cell.reset)
        vx = W - self.px(12) - self.px(35)
        for name in reversed(list(VIEWS)):
            active = (self.yaw, self.pitch) == VIEWS[name]
            self._screen_button(vx, by, name, lambda n=name: self._set_view(n), active=active, width=70)
            vx -= self.px(78)
        c.create_text(vx + self.px(30), by, anchor="e", fill=COL["faint"], font=self.small, tags="dyn",
                      text="drag to rotate  -  shift-drag to pan  -  wheel to zoom")

    # -- stage input -------------------------------------------------------

    def _hot_at(self, ex, ey):
        for x0, y0, x1, y1, press in self.hot:
            if x0 <= ex <= x1 and y0 <= ey <= y1:
                return press
        return None

    def _on_motion(self, e):
        self.stage.configure(cursor="hand2" if self._hot_at(e.x, e.y) else "")

    def _on_press(self, e):
        press = self._hot_at(e.x, e.y)
        if press:
            press()
            self.drag = None
        else:
            self.drag = (e.x, e.y, self.yaw, self.pitch, list(self.pan), bool(e.state & 0x0001))

    def _on_drag(self, e):
        if not self.drag:
            return
        x0, y0, yaw, pitch, pan, shift = self.drag
        if shift:
            self.pan = [pan[0] + e.x - x0, pan[1] + e.y - y0]
        else:
            self.yaw = clamp(yaw + (e.x - x0) * 0.3, -75.0, 75.0)
            self.pitch = clamp(pitch + (e.y - y0) * 0.3, 0.0, 85.0)

    def _on_release(self, e):
        self.drag = None
        if self.held is not None:
            i, self.held = self.held, None
            token = self._btn_token[i]
            wait = max(0.0, MIN_BUTTON_PULSE - (time.monotonic() - self._btn_since[i]))
            self.root.after(int(wait * 1000), lambda: self._button_off(i, token))

    def _hold(self, i):
        self.held = i
        self._btn_token[i] += 1
        self._btn_since[i] = time.monotonic()
        self.cell.set_button(i, 1)

    def _button_off(self, i, token):
        if token == self._btn_token[i]:
            self.cell.set_button(i, 0)

    # -- signal panel ------------------------------------------------------

    def _draw_panel(self):
        cell = self.cell
        values = tuple(cell.read(0, 25))
        key = (values, cell.delivered, cell.scrapped, cell.collisions, self._panel_hover)
        if key == self._panel_key:
            return
        self._panel_key = key
        c, rh = self.sig, self.row_h
        c.delete("all")
        w = max(c.winfo_width(), self.px(270))
        self.panel_rows = []
        y = 0
        for addr, name, io in SIGNALS:
            if addr in (REG_RESET, AT_X):
                c.create_text(0, y + rh / 2, anchor="w", font=self.head, fill=COL["dim"],
                              text="OUTPUTS  -  you write" if io == "out" else "INPUTS  -  you read")
                y += rh
            v = to_signed(values[addr])
            if io == "out" and self._panel_hover == addr:
                c.create_rectangle(-4, y + 1, w, y + rh - 1, fill="#19222d", width=0)
            c.create_text(self.px(22), y + rh / 2, anchor="e", font=self.mono, fill=COL["faint"],
                          text=str(addr))
            c.create_text(self.px(32), y + rh / 2, anchor="w", font=self.mono,
                          fill="#8fc4ea" if io == "out" else COL["text"], text=name)
            if addr in BOOLEAN_ADDRS:
                r = self.px(5)
                cx, cy = w - self.px(30), y + rh / 2
                c.create_oval(cx - r, cy - r, cx + r, cy + r, width=0,
                              fill=COL["green"] if v else "#2c3540")
            c.create_text(w - self.px(4), y + rh / 2, anchor="e", font=self.mono,
                          fill=COL["text"] if v else COL["dim"], text=str(v))
            if io == "out":
                self.panel_rows.append((y, y + rh, addr, name))
            y += rh

        y += self.px(14)
        stats = (("DELIVERED", cell.delivered, COL["green"]), ("SCRAP", cell.scrapped, COL["orange"]),
                 ("COLLISIONS", cell.collisions, COL["red"]))
        cw = w / 3
        for i, (label, n, colour) in enumerate(stats):
            x0 = i * cw
            c.create_rectangle(x0 + 2, y, x0 + cw - 4, y + self.px(62), fill="#131a22", width=0)
            c.create_text(x0 + cw / 2 - 1, y + self.px(24), font=self.big,
                          fill=colour if n else COL["faint"], text=str(n))
            c.create_text(x0 + cw / 2 - 1, y + self.px(48), font=self.small, fill=COL["dim"], text=label)

    def _panel_row(self, ey):
        for y0, y1, addr, name in self.panel_rows:
            if y0 <= ey < y1:
                return addr, name
        return None

    def _on_panel_motion(self, e):
        row = self._panel_row(e.y)
        self._panel_hover = row[0] if row else None
        self.sig.configure(cursor="hand2" if row else "")

    def _on_panel_click(self, e):
        row = self._panel_row(e.y)
        if row:
            self._edit_output(*row)

    # -- dialogs -----------------------------------------------------------

    def _edit_output(self, addr, name):
        current = to_signed(self.cell.read(addr, 1)[0])
        value = simpledialog.askinteger("Write output", "New value for [%d] %s:" % (addr, name),
                                        initialvalue=current, minvalue=-32768, maxvalue=65535,
                                        parent=self.root)
        if value is not None:
            self.cell.write(addr, value, who="Panel")

    def _ask_process_time(self):
        v = simpledialog.askfloat("Process time", "Seconds a process runs:",
                                  initialvalue=self.cell.process_time, minvalue=0.5,
                                  maxvalue=600, parent=self.root)
        if v is not None:
            self.cell.process_time = v
            self.cell.log("Process time set to %g s" % v)

    def _ask_speed(self):
        v = simpledialog.askfloat("Crane speed", "Crane speed (units per second):",
                                  initialvalue=self.cell.speed, minvalue=20, maxvalue=2000,
                                  parent=self.root)
        if v is not None:
            self.cell.speed = v
            self.cell.log("Crane speed set to %g units/s" % v)

    def _ask_layout(self):
        win = tk.Toplevel(self.root)
        win.title("Station positions")
        win.transient(self.root)
        win.resizable(False, False)
        frm = ttk.Frame(win, padding=14)
        frm.pack()
        ttk.Label(frm, text="X position of each station.\nApplying resets the cell.").grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 10))
        entries = {}
        for i, st in enumerate(self.cell.stations, start=1):
            ttk.Label(frm, text=st.name).grid(row=i, column=0, sticky="w", padx=(0, 16), pady=2)
            var = tk.StringVar(value=str(st.x))
            ttk.Entry(frm, textvariable=var, width=8).grid(row=i, column=1, sticky="w", pady=2)
            entries[st.key] = var

        def apply():
            try:
                layout = {k: int(v.get()) for k, v in entries.items()}
            except ValueError:
                messagebox.showerror("Station positions", "Positions must be whole numbers.", parent=win)
                return
            err = Cell.check_layout(layout)
            if err:
                messagebox.showerror("Station positions", err, parent=win)
                return
            self.cell.set_layout(layout)
            self.cell.log("Stations moved: " + ", ".join(
                "%s X=%d" % (st.name, st.x) for st in self.cell.stations))
            win.destroy()

        def defaults():
            for k, v in entries.items():
                v.set(str(DEFAULT_X[k]))

        buttons = ttk.Frame(frm)
        buttons.grid(row=len(entries) + 1, column=0, columnspan=2, sticky="ew", pady=(12, 0))
        ttk.Button(buttons, text="Defaults", command=defaults).pack(side="left")
        ttk.Button(buttons, text="Cancel", command=win.destroy).pack(side="right")
        ttk.Button(buttons, text="Apply", command=apply).pack(side="right", padx=(0, 6))

    def _show_help(self):
        port = self.port or DEFAULT_PORT
        client = "ModbusTcpClient('127.0.0.1')" if port == DEFAULT_PORT else \
            "ModbusTcpClient('127.0.0.1', port=%d)" % port
        lines = [
            "CONNECT",
            "  Modbus TCP server on %s, port %d" % (self.host, port),
            "  CMAS:    Source = modbus.tcp:127.0.0.1, Address = number below",
            "  Python:  " + client,
            "",
            "SIGNALS (0 = False, 1 = True)",
            "  Address  Name       I/O",
        ]
        for addr, name, io in SIGNALS:
            lines.append("  %-8d %-10s %s" % (addr, name, "Output" if io == "out" else "Input"))
        lines += [
            "",
            "POSITIONS",
            "  Pick and place at Y = 82. Larger Y is higher.",
            "  " + ", ".join("%s X=%d" % (st.name, st.x) for st in self.cell.stations),
            "  Travel sideways only at a safe height, or the crane collides.",
            "",
            "BEHAVIOUR",
            "  atX/atY equal setX/setY when the crane has arrived.",
            "  vacuum = 1 on top of a part picks it up; vacuum = 0 releases it.",
            "  A part released anywhere but just above a free station is scrap.",
            "  startp1 = 1 runs Process1 for the process time; p1running is 1",
            "  meanwhile. startp1 is cleared when it finishes. startp1 = 0 stops it.",
            "  Click an output in the signal list to write it by hand.",
            "",
            "VIEW",
            "  Drag to rotate, shift-drag to pan, mouse wheel to zoom.",
            "  Front shows the cell from the side, as in the course figures.",
        ]
        win = tk.Toplevel(self.root)
        win.title("Signals and connection")
        win.transient(self.root)
        text = tk.Text(win, width=74, height=len(lines) + 1, font="TkFixedFont",
                       relief="flat", padx=12, pady=10, bg=COL["panel"], fg=COL["text"])
        text.insert("1.0", "\n".join(lines))
        text.configure(state="disabled")
        text.pack(fill="both", expand=True)


# ---------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Plug & Produce Simulation with a Modbus TCP server.")
    parser.add_argument("--port", type=int, default=None,
                        help="Modbus TCP port (default 502, 5020 on Linux without rights)")
    parser.add_argument("--host", default="127.0.0.1",
                        help="address to listen on (default 127.0.0.1)")
    parser.add_argument("--process-time", type=float, default=4.0,
                        help="seconds a process runs (default 4)")
    parser.add_argument("--speed", type=float, default=250.0,
                        help="crane speed in units per second (default 250)")
    parser.add_argument("--station", action="append", default=[], metavar="NAME=X",
                        help="move a station, e.g. --station process2=700")
    args = parser.parse_args(argv)

    layout = {}
    for item in args.station:
        key, _, value = item.partition("=")
        try:
            layout[key.strip().lower()] = int(value)
        except ValueError:
            parser.error("--station expects NAME=X, for example process2=700")
    err = Cell.check_layout(layout)
    if err:
        parser.error(err)

    if tk is None:
        print(TK_MISSING)
        return 1

    cell = Cell(process_time=args.process_time, speed=args.speed, layout=layout)
    port = args.port if args.port is not None else DEFAULT_PORT
    server, error = start_server(cell, args.host, port, args.port is not None)

    if IS_WINDOWS:
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass

    root = tk.Tk()
    App(root, cell, server, error, args.host)
    try:
        root.mainloop()
    finally:
        if server:
            server.shutdown()
            server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
