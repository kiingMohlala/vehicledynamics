"""Virtual CAN bus message encoder/decoder for vehicle signals."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any
import struct


@dataclass
class CANMessage:
    arbitration_id: int
    data: bytes
    name: str = ""
    timestamp: float = 0.0


@dataclass
class CANBus:
    """In-memory virtual CAN: encode common vehicle signals."""
    log: list[CANMessage] = field(default_factory=list)

    # ID map (11-bit style)
    ID_RPM = 0x100
    ID_TPS = 0x101
    ID_WHEEL_SPEED = 0x110
    ID_ABS = 0x120
    ID_BRAKE = 0x121
    ID_STEER = 0x130
    ID_SOC = 0x140

    def publish(self, state: dict[str, Any], t: float = 0.0) -> list[CANMessage]:
        msgs = []
        if "engine_rpm" in state:
            rpm = int(max(0, min(state["engine_rpm"], 20000)))
            msgs.append(CANMessage(self.ID_RPM, struct.pack(">H", rpm), "RPM", t))
        if "throttle" in state:
            tps = int(max(0, min(state["throttle"], 1.0)) * 1000)
            msgs.append(CANMessage(self.ID_TPS, struct.pack(">H", tps), "TPS", t))
        if "wheel_speed" in state:
            # pack 4 x uint16 in 0.01 m/s
            ws = state["wheel_speed"]
            vals = [int(max(0, min(float(w), 100.0)) * 100) for w in list(ws)[:4]]
            while len(vals) < 4:
                vals.append(0)
            msgs.append(CANMessage(self.ID_WHEEL_SPEED, struct.pack(">4H", *vals), "WHEEL_SPEED", t))
        if "brake" in state:
            br = int(max(0, min(state["brake"], 1.0)) * 1000)
            msgs.append(CANMessage(self.ID_BRAKE, struct.pack(">H", br), "BRAKE", t))
        if "steer" in state:
            # steer in 0.0001 rad, signed
            ang = int(max(-3.0, min(float(state["steer"]), 3.0)) * 10000)
            msgs.append(CANMessage(self.ID_STEER, struct.pack(">h", ang), "STEER", t))
        if "battery_soc" in state:
            soc = int(max(0, min(state["battery_soc"], 1.0)) * 1000)
            msgs.append(CANMessage(self.ID_SOC, struct.pack(">H", soc), "SOC", t))
        if "abs_active" in state:
            flag = 1 if state["abs_active"] else 0
            msgs.append(CANMessage(self.ID_ABS, struct.pack(">B", flag), "ABS", t))
        self.log.extend(msgs)
        return msgs

    def decode(self, msg: CANMessage) -> dict[str, Any]:
        if msg.arbitration_id == self.ID_RPM:
            return {"engine_rpm": struct.unpack(">H", msg.data)[0]}
        if msg.arbitration_id == self.ID_TPS:
            return {"throttle": struct.unpack(">H", msg.data)[0] / 1000.0}
        if msg.arbitration_id == self.ID_BRAKE:
            return {"brake": struct.unpack(">H", msg.data)[0] / 1000.0}
        if msg.arbitration_id == self.ID_STEER:
            return {"steer": struct.unpack(">h", msg.data)[0] / 10000.0}
        if msg.arbitration_id == self.ID_SOC:
            return {"battery_soc": struct.unpack(">H", msg.data)[0] / 1000.0}
        if msg.arbitration_id == self.ID_ABS:
            return {"abs_active": struct.unpack(">B", msg.data)[0] == 1}
        if msg.arbitration_id == self.ID_WHEEL_SPEED:
            vals = struct.unpack(">4H", msg.data)
            return {"wheel_speed": [v / 100.0 for v in vals]}
        return {}
