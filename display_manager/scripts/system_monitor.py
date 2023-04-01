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

    def update(self):
        # Get IP address
        cmd = "hostname -I | cut -d\' \' -f1 | head --bytes -1"
        self.ip = subprocess.check_output(cmd, shell=True).decode().strip()

        # Get CPU usage
        cmd = "top -bn1 | grep load | awk '{printf \"%.2fLA\", $(NF-2)}'"
        self.cpu = subprocess.check_output(cmd, shell=True).decode().strip()

        # Get memory usage
        cmd = "free -m | awk 'NR==2{printf \"%.2f%%\", $3*100/$2 }'"
        self.mem_usage = subprocess.check_output(cmd, shell=True).decode().strip()

        # Get disk usage
        cmd = "df -h | awk '$NF==\"/\"{printf \"%d/%dGB\", $3,$2}'"
        self.disk = subprocess.check_output(cmd, shell=True).decode().strip()

        # Get temperature
        cmd = "vcgencmd measure_temp | cut -d '=' -f 2 | head --bytes -1"
        self.temperature = subprocess.check_output(cmd, shell=True).decode().strip()

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False
