#!/usr/bin/env python

import shutil
import subprocess
from collections import deque
from datetime import datetime, timedelta
from typing import Deque, Dict, Optional, Tuple

import rospy


def _read_boot_time() -> datetime:
    """Best-effort read of the system boot timestamp."""
    try:
        with open("/proc/uptime", "r", encoding="utf-8") as f:
            uptime_seconds = float(f.readline().split()[0])
        return datetime.now() - timedelta(seconds=uptime_seconds)
    except (OSError, ValueError):
        # Fall back to "now" if /proc/uptime is not available (e.g. on non-Linux)
        return datetime.now()


class SystemMonitor:
    """Collects system telemetry and keeps a boot-to-present event log."""

    _EVENT_HISTORY = 9

    def __init__(self) -> None:
        self.boot_time: datetime = _read_boot_time()
        self.paused: bool = False

        # Runtime telemetry caches
        self._last_cpu_total: Optional[int] = None
        self._last_cpu_idle: Optional[int] = None
        self._last_cpu_reading: Optional[float] = None
        self._last_trend_percent: Optional[float] = None
        self.cpu_history: Deque[float] = deque(maxlen=120)

        # Peak tracking since boot
        self._peak_cpu: Optional[Tuple[float, datetime]] = None
        self._peak_mem: Optional[Tuple[float, datetime]] = None
        self._peak_temp: Optional[Tuple[float, datetime]] = None

        # Alert state toggles so that we only log crossings once.
        self._high_cpu_active = False
        self._high_mem_active = False
        self._low_disk_active = False
        self._high_temp_active = False

        # Human facing state
        self.ip_address: str = ""
        self.timeline: Deque[Tuple[datetime, str]] = deque(maxlen=self._EVENT_HISTORY)

        self._log_event(f"Booted {self.boot_time:%H:%M:%S}")
        self._log_event("Display manager online")

    # ------------------------------------------------------------------
    # Public API

    def update(self) -> Optional[Dict[str, object]]:
        if self.paused:
            return None

        now = datetime.now()
        uptime_seconds = (now - self.boot_time).total_seconds()

        cpu_percent = self._read_cpu_percent()
        load_avg = self._read_load_average()
        mem_used, mem_total, mem_percent = self._read_memory()
        disk_used, disk_total, disk_percent = self._read_disk()
        temperature_c = self._read_temperature()
        ip_address = self._read_ip_address()

        trend = self._compute_trend(cpu_percent)
        self.cpu_history.append(cpu_percent)

        self._update_peaks(cpu_percent, mem_percent, temperature_c, now)
        self._update_alerts(cpu_percent, mem_percent, disk_percent, temperature_c, now)

        snapshot: Dict[str, object] = {
            "timestamp": now,
            "uptime_seconds": uptime_seconds,
            "boot_time": self.boot_time,
            "cpu_percent": cpu_percent,
            "cpu_trend": trend,
            "load_average": load_avg,
            "mem_percent": mem_percent,
            "mem_used": mem_used,
            "mem_total": mem_total,
            "disk_percent": disk_percent,
            "disk_used": disk_used,
            "disk_total": disk_total,
            "temperature_c": temperature_c,
            "ip_address": ip_address,
            "events": list(self.timeline),
            "cpu_peak": self._peak_cpu,
            "mem_peak": self._peak_mem,
            "temp_peak": self._peak_temp,
        }
        return snapshot

    def pause(self) -> None:
        if not self.paused:
            self.paused = True
            self._log_event("Monitor paused")

    def resume(self) -> None:
        if self.paused:
            self.paused = False
            self._log_event("Monitor resumed")

    def annotate(self, message: str) -> None:
        """Expose event logging so callers can annotate the timeline."""
        self._log_event(message)

    # ------------------------------------------------------------------
    # Telemetry helpers

    def _read_cpu_percent(self) -> float:
        try:
            with open("/proc/stat", "r", encoding="utf-8") as f:
                cpu_line = f.readline()
        except OSError:
            return 0.0

        if not cpu_line.startswith("cpu "):
            return 0.0

        parts = cpu_line.split()[1:]
        if len(parts) < 8:
            return 0.0

        values = list(map(int, parts[:8]))
        user, nice, system, idle, iowait, irq, softirq, steal = values
        idle_all = idle + iowait
        non_idle = user + nice + system + irq + softirq + steal
        total = idle_all + non_idle

        if self._last_cpu_total is None or self._last_cpu_idle is None:
            self._last_cpu_total = total
            self._last_cpu_idle = idle_all
            self._last_cpu_reading = 0.0
            return 0.0

        total_delta = total - self._last_cpu_total
        idle_delta = idle_all - self._last_cpu_idle
        self._last_cpu_total = total
        self._last_cpu_idle = idle_all

        if total_delta <= 0:
            return self._last_cpu_reading or 0.0

        usage = (total_delta - idle_delta) / float(total_delta) * 100.0
        usage = max(0.0, min(usage, 100.0))
        self._last_cpu_reading = usage
        return usage

    def _read_load_average(self) -> float:
        try:
            with open("/proc/loadavg", "r", encoding="utf-8") as f:
                return float(f.readline().split()[0])
        except (OSError, ValueError):
            return 0.0

    def _read_memory(self) -> Tuple[float, float, float]:
        mem_total_kb = 0.0
        mem_available_kb = 0.0
        try:
            with open("/proc/meminfo", "r", encoding="utf-8") as f:
                for line in f:
                    if line.startswith("MemTotal"):
                        mem_total_kb = float(line.split()[1])
                    elif line.startswith("MemAvailable"):
                        mem_available_kb = float(line.split()[1])
                    if mem_total_kb and mem_available_kb:
                        break
        except (OSError, ValueError):
            return 0.0, 0.0, 0.0

        if mem_total_kb <= 0:
            return 0.0, 0.0, 0.0

        mem_used_kb = max(mem_total_kb - mem_available_kb, 0.0)
        mem_percent = (mem_used_kb / mem_total_kb) * 100.0

        mem_used_gb = mem_used_kb / (1024.0 * 1024.0)
        mem_total_gb = mem_total_kb / (1024.0 * 1024.0)
        return mem_used_gb, mem_total_gb, mem_percent

    def _read_disk(self) -> Tuple[float, float, float]:
        try:
            usage = shutil.disk_usage("/")
        except OSError:
            return 0.0, 0.0, 0.0

        used_gb = usage.used / (1024.0 ** 3)
        total_gb = usage.total / (1024.0 ** 3)
        percent = (usage.used / usage.total) * 100.0 if usage.total else 0.0
        return used_gb, total_gb, percent

    def _read_temperature(self) -> Optional[float]:
        cmd = ["vcgencmd", "measure_temp"]
        try:
            output = subprocess.check_output(cmd, stderr=subprocess.DEVNULL).decode().strip()
            if "=" in output:
                temp_token = output.split("=")[-1]
                return float(temp_token.replace("'C", ""))
        except (subprocess.CalledProcessError, FileNotFoundError, ValueError):
            pass

        # Fallback to the thermal zone file if vcgencmd is unavailable.
        thermal_path = "/sys/class/thermal/thermal_zone0/temp"
        try:
            with open(thermal_path, "r", encoding="utf-8") as f:
                return float(f.readline()) / 1000.0
        except (OSError, ValueError):
            return None

    def _read_ip_address(self) -> str:
        cmd = "hostname -I | awk '{print $1}'"
        try:
            ip_candidate = subprocess.check_output(cmd, shell=True, stderr=subprocess.DEVNULL).decode().strip()
        except subprocess.CalledProcessError:
            ip_candidate = ""

        if not ip_candidate:
            ip_candidate = "No link"

        if ip_candidate != self.ip_address:
            self.ip_address = ip_candidate
            self._log_event(f"IP {self.ip_address}")

        return self.ip_address

    # ------------------------------------------------------------------
    # Trend, peak and alert bookkeeping

    def _compute_trend(self, current: float) -> str:
        previous = self._last_trend_percent
        self._last_trend_percent = current
        if previous is None:
            return "steady"

        delta = current - previous
        if delta > 1.5:
            return "up"
        if delta < -1.5:
            return "down"
        return "steady"

    def _update_peaks(self, cpu: float, mem: float, temp: Optional[float], when: datetime) -> None:
        if self._peak_cpu is None or cpu > self._peak_cpu[0]:
            self._peak_cpu = (cpu, when)
        if self._peak_mem is None or mem > self._peak_mem[0]:
            self._peak_mem = (mem, when)
        if temp is not None:
            if self._peak_temp is None or temp > self._peak_temp[0]:
                self._peak_temp = (temp, when)

    def _update_alerts(
        self,
        cpu: float,
        mem: float,
        disk_percent: float,
        temp: Optional[float],
        when: datetime,
    ) -> None:
        if cpu >= 85.0 and not self._high_cpu_active:
            self._high_cpu_active = True
            self._log_event(f"CPU high {cpu:.0f}%", when)
        elif cpu < 75.0 and self._high_cpu_active:
            self._high_cpu_active = False
            self._log_event("CPU recovered", when)

        if mem >= 80.0 and not self._high_mem_active:
            self._high_mem_active = True
            self._log_event(f"MEM high {mem:.0f}%", when)
        elif mem < 70.0 and self._high_mem_active:
            self._high_mem_active = False
            self._log_event("MEM recovered", when)

        if disk_percent >= 90.0 and not self._low_disk_active:
            self._low_disk_active = True
            self._log_event(f"Disk {disk_percent:.0f}%", when)
        elif disk_percent < 80.0 and self._low_disk_active:
            self._low_disk_active = False
            self._log_event("Disk recovered", when)

        if temp is not None:
            if temp >= 70.0 and not self._high_temp_active:
                self._high_temp_active = True
                self._log_event(f"Temp high {temp:.0f}C", when)
            elif temp < 60.0 and self._high_temp_active:
                self._high_temp_active = False
                self._log_event("Temp cooled", when)

    # ------------------------------------------------------------------
    # Timeline helpers

    def _log_event(self, message: str, when: Optional[datetime] = None) -> None:
        when = when or datetime.now()
        trimmed = message.strip()
        if not trimmed:
            return
        self.timeline.append((when, trimmed[:32]))
        rospy.logdebug(f"[SystemMonitor] {when:%H:%M:%S} - {trimmed}")
