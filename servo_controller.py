\
"""
servo_controller.py

Axis servo execution layer for DYNAMIXEL Protocol 2.0 motors (e.g., XL330) via U2D2.

Key design choice:
- Maintain an internal command state (cmd_ticks) per motor.
- Absolute targets set cmd_ticks directly.
- Relative nudges add to cmd_ticks (NOT to present readings) to avoid integrating measurement jitter.
- Execute with time-synced streamed setpoints so multiple motors arrive at targets at roughly the same time.

"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import time
import json
import pathlib

from dynamixel_sdk import (  # type: ignore
    PortHandler, PacketHandler,
    GroupSyncWrite, GroupBulkRead,
    COMM_SUCCESS,
    DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD
)


@dataclass
class DxlConfig:
    device: str
    baudrate: int
    protocol: float
    ids: List[int]
    ticks_per_rev: int

    addr_operating_mode: int
    addr_torque_enable: int
    addr_goal_position: int
    addr_present_position: int
    addr_present_current: int

    mode_extended_position: int


def load_config(path: str | pathlib.Path = "config.json") -> DxlConfig:
    p = pathlib.Path(path)
    cfg = json.loads(p.read_text(encoding="utf-8"))

    add = cfg["addresses"]
    modes = cfg["modes"]

    return DxlConfig(
        device=cfg["device"],
        baudrate=int(cfg["baudrate"]),
        protocol=float(cfg.get("protocol", 2.0)),
        ids=[int(x) for x in cfg["ids"]],
        ticks_per_rev=int(cfg.get("ticks_per_rev", 4096)),

        addr_operating_mode=int(add["operating_mode"]),
        addr_torque_enable=int(add["torque_enable"]),
        addr_goal_position=int(add["goal_position"]),
        addr_present_position=int(add["present_position"]),
        addr_present_current=int(add.get("present_current", 126)),

        mode_extended_position=int(modes.get("extended_position", 4)),
    )


class AxisServo:
    """
    Axis servo execution layer.

    Public API:
      - connect(), close()
      - scan_ids()
      - ensure_extended_mode()
      - soft_reset(mode)
      - set_targets_abs_deg({...})
      - nudge_rel_deg({...})
      - execute_time_synced(...)
    """
    LEN_4 = 4
    LEN_2 = 2

    TORQUE_ENABLE = 1
    TORQUE_DISABLE = 0

    def __init__(self, cfg: DxlConfig):
        self.cfg = cfg
        self.port = PortHandler(cfg.device)
        self.packet = PacketHandler(cfg.protocol)

        # controller_state
        self.cmd_ticks: Dict[int, int] = {i: 0 for i in cfg.ids}
        self.last_present_ticks: Dict[int, Optional[int]] = {i: None for i in cfg.ids}
        self.last_present_current: Dict[int, Optional[int]] = {i: None for i in cfg.ids}

        self.connected = False

        # Some firmwares/models may not expose Present Current; detect at runtime
        self._supports_current: Optional[bool] = None

    # ----- conversions -----
    @staticmethod
    def _int32(x: int) -> int:
        x &= 0xFFFFFFFF
        return x - 0x100000000 if x & 0x80000000 else x

    @staticmethod
    def _i16(x: int) -> int:
        x &= 0xFFFF
        return x - 0x10000 if x & 0x8000 else x

    @staticmethod
    def _u32(x: int) -> int:
        return x & 0xFFFFFFFF

    def deg_to_ticks(self, deg: float) -> int:
        return int(round(deg * self.cfg.ticks_per_rev / 360.0))

    def ticks_to_deg(self, ticks: int) -> float:
        return ticks * 360.0 / self.cfg.ticks_per_rev

    # ----- transport -----
    def connect(self) -> None:
        if not self.port.openPort():
            raise RuntimeError("openPort failed")
        if not self.port.setBaudRate(self.cfg.baudrate):
            raise RuntimeError("setBaudRate failed")
        self.connected = True

    def close(self) -> None:
        if self.connected:
            self.port.closePort()
        self.connected = False

    # ----- operations -----
    def scan_ids(self) -> List[int]:
        online: List[int] = []
        for dxl_id in range(0, 253):
            _, comm, err = self.packet.ping(self.port, dxl_id)
            if comm == COMM_SUCCESS and err == 0:
                online.append(dxl_id)
        return online

    def ensure_extended_mode(self, ids: Optional[List[int]] = None) -> None:
        ids = ids or self.cfg.ids
        for dxl_id in ids:
            # torque off
            self.packet.write1ByteTxRx(self.port, dxl_id, self.cfg.addr_torque_enable, self.TORQUE_DISABLE)
            # set mode
            self.packet.write1ByteTxRx(self.port, dxl_id, self.cfg.addr_operating_mode, self.cfg.mode_extended_position)
            # torque on
            self.packet.write1ByteTxRx(self.port, dxl_id, self.cfg.addr_torque_enable, self.TORQUE_ENABLE)

    def soft_reset(self, mode: str = "to_present") -> None:
        pos, _ = self.bulk_read(want_current=False)
        if mode == "to_present":
            for dxl_id in self.cfg.ids:
                self.cmd_ticks[dxl_id] = pos[dxl_id]
        elif mode == "to_nearest_turn":
            r = self.cfg.ticks_per_rev
            for dxl_id in self.cfg.ids:
                self.cmd_ticks[dxl_id] = int(round(pos[dxl_id] / r) * r)
        else:
            raise ValueError("mode must be 'to_present' or 'to_nearest_turn'")

    def set_targets_abs_deg(self, abs_deg_by_id: Dict[int, float]) -> None:
        for dxl_id, deg in abs_deg_by_id.items():
            t = self.deg_to_ticks(deg)
            t = max(-2**31, min(2**31 - 1, t))
            self.cmd_ticks[dxl_id] = t

    def nudge_rel_deg(self, delta_deg_by_id: Dict[int, float]) -> None:
        for dxl_id, ddeg in delta_deg_by_id.items():
            dt = self.deg_to_ticks(ddeg)
            t = self.cmd_ticks[dxl_id] + dt
            t = max(-2**31, min(2**31 - 1, t))
            self.cmd_ticks[dxl_id] = t

    # ----- bulk read/write -----
    def bulk_read(self, want_current: bool = True) -> Tuple[Dict[int, int], Dict[int, Optional[int]]]:
        br = GroupBulkRead(self.port, self.packet)

        for dxl_id in self.cfg.ids:
            br.addParam(dxl_id, self.cfg.addr_present_position, self.LEN_4)

        # Current is optional; if we haven't determined support yet, try once
        if want_current:
            if self._supports_current is False:
                want_current = False
            else:
                for dxl_id in self.cfg.ids:
                    br.addParam(dxl_id, self.cfg.addr_present_current, self.LEN_2)

        comm = br.txRxPacket()
        if comm != COMM_SUCCESS:
            raise RuntimeError("BulkRead failed: " + self.packet.getTxRxResult(comm))

        pos: Dict[int, int] = {}
        cur: Dict[int, Optional[int]] = {i: None for i in self.cfg.ids}

        # Read positions (required)
        for dxl_id in self.cfg.ids:
            if not br.isAvailable(dxl_id, self.cfg.addr_present_position, self.LEN_4):
                raise RuntimeError(f"PresentPosition not available for ID {dxl_id}")
            raw_pos = br.getData(dxl_id, self.cfg.addr_present_position, self.LEN_4)
            pos[dxl_id] = self._int32(raw_pos)

        # Read current (optional)
        if want_current:
            ok_all = True
            for dxl_id in self.cfg.ids:
                if br.isAvailable(dxl_id, self.cfg.addr_present_current, self.LEN_2):
                    raw_cur = br.getData(dxl_id, self.cfg.addr_present_current, self.LEN_2)
                    cur[dxl_id] = self._i16(raw_cur)
                else:
                    ok_all = False

            # Decide support based on first attempt
            if self._supports_current is None:
                self._supports_current = ok_all

        self.last_present_ticks.update(pos)
        self.last_present_current.update(cur)
        return pos, cur

    def sync_write_goal_positions(self, goal_ticks: Dict[int, int]) -> None:
        sw = GroupSyncWrite(self.port, self.packet, self.cfg.addr_goal_position, self.LEN_4)
        for dxl_id, goal in goal_ticks.items():
            u = self._u32(goal)
            param = [
                DXL_LOBYTE(DXL_LOWORD(u)),
                DXL_HIBYTE(DXL_LOWORD(u)),
                DXL_LOBYTE(DXL_HIWORD(u)),
                DXL_HIBYTE(DXL_HIWORD(u)),
            ]
            if not sw.addParam(dxl_id, bytearray(param)):
                raise RuntimeError(f"SyncWrite addParam failed for ID {dxl_id}")
        comm = sw.txPacket()
        sw.clearParam()
        if comm != COMM_SUCCESS:
            raise RuntimeError("SyncWrite failed: " + self.packet.getTxRxResult(comm))

    # ----- time-synced execution -----
    def execute_time_synced(
        self,
        duration_s: float = 1.0,
        poll_hz: int = 50,
        eps_deg: float = 2.0,
        settle_n: int = 3,
        timeout_factor: float = 2.0,
        smoothstep: bool = True,
        current_guard: Optional[Tuple[int, int]] = None,  # (min,max) optional
        log: bool = True,
    ) -> Dict[str, object]:
        """
        Streamed setpoints so all motors reach goals in ~duration_s.

        This avoids relying on vendor-specific ProfileVelocity units, and yields good synchronization.

        Returns a dict summary with start/goal/final positions and error.
        """
        start_pos, start_cur = self.bulk_read(want_current=True)
        goal = {i: self.cmd_ticks[i] for i in self.cfg.ids}

        steps = max(1, int(duration_s * poll_hz))
        dt = duration_s / steps
        eps_ticks = abs(self.deg_to_ticks(eps_deg))

        if log:
            print(f"[EXEC] duration={duration_s:.3f}s steps={steps} poll={poll_hz}Hz eps≈{eps_deg}deg ({eps_ticks} ticks)")

        t0 = time.time()

        def shape(a: float) -> float:
            if not smoothstep:
                return a
            return a * a * (3 - 2 * a)

        # 1) Streamed setpoints
        for k in range(1, steps + 1):
            alpha = k / steps
            a = shape(alpha)
            mid = {i: int(round(start_pos[i] + (goal[i] - start_pos[i]) * a)) for i in self.cfg.ids}
            self.sync_write_goal_positions(mid)

            # optional guard
            if current_guard is not None and (k % max(1, poll_hz // 10) == 0):
                _, cur = self.bulk_read(want_current=True)
                mn, mx = current_guard
                bad = [i for i in self.cfg.ids if (cur[i] is not None and (cur[i] < mn or cur[i] > mx))]
                if bad:
                    if log:
                        print("[GUARD] current out of range, stop:", bad)
                    break

            # keep timing
            spent = time.time() - t0
            target = k * dt
            sleep_s = target - spent
            if sleep_s > 0:
                time.sleep(sleep_s)

        # 2) settle check
        deadline = time.time() + max(0.2, duration_s * timeout_factor)
        ok_streak = 0
        last_max_err = None

        while time.time() < deadline:
            pos, cur = self.bulk_read(want_current=True)
            max_err = max(abs(goal[i] - pos[i]) for i in self.cfg.ids)
            last_max_err = max_err

            if max_err <= eps_ticks:
                ok_streak += 1
                if ok_streak >= settle_n:
                    if log:
                        print(f"[DONE] settled. max_err={max_err} ticks (~{self.ticks_to_deg(max_err):.2f} deg)")
                    return {
                        "settled": True,
                        "start_pos": start_pos,
                        "start_current": start_cur,
                        "goal": goal,
                        "final_pos": pos,
                        "final_current": cur,
                        "max_err_ticks": max_err,
                        "max_err_deg": self.ticks_to_deg(max_err),
                    }
            else:
                ok_streak = 0

            time.sleep(1.0 / max(5, poll_hz))

        if log:
            deg = self.ticks_to_deg(last_max_err or 0)
            print(f"[TIMEOUT] not settled. last max_err={last_max_err} ticks (~{deg:.2f} deg)")

        pos, cur = self.bulk_read(want_current=True)
        return {
            "settled": False,
            "start_pos": start_pos,
            "start_current": start_cur,
            "goal": goal,
            "final_pos": pos,
            "final_current": cur,
            "max_err_ticks": last_max_err,
            "max_err_deg": self.ticks_to_deg(last_max_err) if last_max_err is not None else None,
        }
