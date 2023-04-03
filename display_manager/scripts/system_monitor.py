import subprocess
import time
import rospy
from std_msgs.msg import String

class SystemMonitor:
    def __init__(self):
        self.ip = ""
        self.cpu = ""
        self.mem_usage = ""
        self.disk = ""
        self.temperature = ""
        self.paused = False
        self.update() # set initial values.

    def update(self):
        # Get IP address
        cmd = "hostname -I | cut -d\' \' -f1 | head --bytes -1"
        ipString = subprocess.check_output(cmd, shell=True).decode().strip()
        self.ip = f"IP: {ipString}"

        # Get CPU usage
        cmd = "top -bn1 | grep load | awk '{printf \"%.2fLA\", $(NF-2)}'"
        cpuString = subprocess.check_output(cmd, shell=True).decode().strip()
        self.cpu = f"CPU: {cpuString}"

        # Get memory usage
        cmd = "free -m | awk 'NR==2{printf \"%.2f%%\", $3*100/$2 }'"
        memString = subprocess.check_output(cmd, shell=True).decode().strip()
        self.mem_usage = f"MEM: {memString}"

        # Get disk usage
        cmd = "df -h | awk '$NF==\"/\"{printf \"%d/%dGB\", $3,$2}'"
        diskString = subprocess.check_output(cmd, shell=True).decode().strip()
        self.disk = f"DISK: {diskString}"

        # Get temperature
        cmd = "vcgencmd measure_temp | cut -d '=' -f 2 | head --bytes -1"
        tempString = subprocess.check_output(cmd, shell=True).decode().strip()
        self.temperature = f"TEMP: {tempString}"

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False
