#!/usr/bin/env python

from datetime import datetime
from enum import Enum
from typing import Callable, List, Optional, Sequence, Tuple

import rospy
from std_msgs.msg import ColorRGBA, Empty, Int32MultiArray, String

from system_monitor import SystemMonitor


RED = ColorRGBA(r=255.0, g=0.0, b=0.0, a=1.0)
GREEN = ColorRGBA(r=0.0, g=255.0, b=0.0, a=1.0)
BLUE = ColorRGBA(r=0.0, g=0.0, b=255.0, a=1.0)
PURPLE = ColorRGBA(r=128.0, g=0.0, b=128.0, a=1.0)
BLACK = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)


class ScreenMode(Enum):
    MONITOR = 0
    ULTRASONIC = 1
    STATIC = 2


class DisplayManager:
    def __init__(self) -> None:
        rospy.init_node("display_manager")
        rospy.loginfo("Running Display Manager Node...")

        self.current_color = BLACK
        self.current_text = ""

        self.screen_mode = ScreenMode.MONITOR
        self.button_taps = 0

        self.system_monitor = SystemMonitor()
        self.system_monitor.annotate("Mode: monitor")

        self.monitor_pages: Sequence[Callable[[dict], List[str]]] = (
            self._render_overview,
            self._render_peaks,
            self._render_timeline,
        )
        self.monitor_page_index = 0
        self.monitor_page_tick = 0
        self.monitor_page_hold = 2  # updates before rotating to the next page

        self.color_pub = rospy.Publisher("oled_color", ColorRGBA, queue_size=10)
        self.text_pub = rospy.Publisher("oled_text", String, queue_size=10)
        rospy.Subscriber("ultrasonic_data", Int32MultiArray, self.ultrasonic_callback)
        rospy.Subscriber("button_press", Empty, self.button_callback)

        self.monitor_timer = rospy.Timer(rospy.Duration(2), self.publish_system_stats)

    # ------------------------------------------------------------------
    # Callbacks

    def ultrasonic_callback(self, msg: Int32MultiArray) -> None:
        if self.screen_mode != ScreenMode.ULTRASONIC:
            return

        left_dist, right_dist = msg.data[0], msg.data[1]
        rospy.logdebug(f"Received ultrasonic data: {left_dist}, {right_dist}")

        if left_dist < 5 or right_dist < 5:
            self.current_color = RED
        else:
            self.current_color = GREEN
        self.publish_color()

    def button_callback(self, _msg: Empty) -> None:
        self.button_taps += 1
        self.update_mode()

    # ------------------------------------------------------------------
    # Publishers

    def publish_color(self) -> None:
        self.color_pub.publish(self.current_color)

    def publish_system_stats(self, _event: Optional[rospy.TimerEvent]) -> None:
        if self.screen_mode != ScreenMode.MONITOR:
            return

        snapshot = self.system_monitor.update()
        if not snapshot:
            return

        page_renderer = self.monitor_pages[self.monitor_page_index]
        lines = page_renderer(snapshot)
        stats_str = "|".join(lines)
        if self.current_text != stats_str:
            rospy.logdebug(f"OLED update on page {self.monitor_page_index}: {lines}")
            self.text_pub.publish(stats_str)
            self.current_text = stats_str

        self.monitor_page_tick += 1
        if self.monitor_page_tick >= self.monitor_page_hold:
            self.monitor_page_tick = 0
            self.monitor_page_index = (self.monitor_page_index + 1) % len(self.monitor_pages)

    # ------------------------------------------------------------------
    # Mode management

    def static_screen(self) -> None:
        self.current_color = PURPLE
        self.publish_color()

    def update_mode(self) -> None:
        if self.screen_mode == ScreenMode.MONITOR:
            self.system_monitor.pause()

        modes = list(ScreenMode)
        self.screen_mode = modes[self.button_taps % len(ScreenMode)]

        if self.screen_mode == ScreenMode.STATIC:
            self.system_monitor.annotate("Mode: static")
            self.static_screen()
        elif self.screen_mode == ScreenMode.ULTRASONIC:
            self.system_monitor.annotate("Mode: ultrasonic")
            self.current_color = BLUE
            self.publish_color()
        elif self.screen_mode == ScreenMode.MONITOR:
            self.system_monitor.annotate("Mode: monitor")
            self.current_color = BLACK
            self.publish_color()
            self.system_monitor.resume()
            self.monitor_page_index = 0
            self.monitor_page_tick = 0
            self.publish_system_stats(None)

    # ------------------------------------------------------------------
    # Page renderers

    def _render_overview(self, snapshot: dict) -> List[str]:
        now = snapshot["timestamp"]
        uptime = self._format_uptime(snapshot["uptime_seconds"])
        line1 = f"UP {uptime:>6} {now:%H:%M}"

        cpu_percent = snapshot["cpu_percent"]
        trend_symbol = {"up": "^", "down": "v", "steady": "~"}.get(snapshot["cpu_trend"], "~")
        load_avg = snapshot["load_average"]
        line2 = f"CPU {cpu_percent:>3.0f}% {trend_symbol}{load_avg:.2f}"

        mem_percent = snapshot["mem_percent"]
        line3 = f"MEM {mem_percent:>3.0f}% {self._format_bar(mem_percent)}"

        disk_used = int(round(snapshot["disk_used"]))
        disk_total = int(round(snapshot["disk_total"]))
        disk_used = max(0, min(disk_used, 99))
        disk_total = max(0, min(disk_total, 99))
        temp = snapshot["temperature_c"]
        temp_str = "--" if temp is None else f"{int(round(temp)):02d}"
        line4 = f"DSK {disk_used:02}/{disk_total:02}G T{temp_str}"

        return [line1, line2, line3, line4]

    def _render_peaks(self, snapshot: dict) -> List[str]:
        ip_address = snapshot["ip_address"]
        line1 = f"NET {ip_address[:13]}"

        line2 = self._format_peak_line("CPU^", snapshot.get("cpu_peak"), "%")
        line3 = self._format_peak_line("MEM^", snapshot.get("mem_peak"), "%")
        line4 = self._format_peak_line("TMP^", snapshot.get("temp_peak"), "C")
        return [line1, line2, line3, line4]

    def _render_timeline(self, snapshot: dict) -> List[str]:
        boot_time = snapshot["boot_time"]
        line1 = f"BOOT {boot_time:%m-%d %H:%M}"

        events: List[Tuple] = [evt for evt in snapshot["events"] if not evt[1].startswith("Booted")]
        recent = list(reversed(events[-3:]))
        lines = [self._format_event_line(evt) for evt in recent]
        while len(lines) < 3:
            lines.append("--:-- (idle)")
        return [line1] + lines[:3]

    # ------------------------------------------------------------------
    # Formatting helpers

    @staticmethod
    def _format_uptime(seconds: float) -> str:
        total_seconds = int(seconds)
        days, rem = divmod(total_seconds, 86400)
        hours, rem = divmod(rem, 3600)
        minutes, _ = divmod(rem, 60)
        if days:
            return f"{days}d{hours:02}h"
        return f"{hours:02}h{minutes:02}m"

    @staticmethod
    def _format_bar(percent: float, width: int = 8) -> str:
        filled = int(round(percent / 100.0 * width))
        filled = max(0, min(width, filled))
        return "#" * filled + "." * (width - filled)

    @staticmethod
    def _format_peak_line(label: str, peak: Optional[Tuple[float, datetime]], unit: str) -> str:
        if not peak:
            return f"{label} -- --:--"
        value, when = peak
        if when is None:
            return f"{label} {value:>3.0f}{unit} --:--"
        return f"{label} {value:>3.0f}{unit} {when:%H:%M}"

    @staticmethod
    def _format_event_line(event: Optional[Tuple[datetime, str]]) -> str:
        if not event:
            return "--:-- (idle)"
        when, message = event
        return f"{when:%H:%M} {message[:10]}"


if __name__ == "__main__":
    try:
        DisplayManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
