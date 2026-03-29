#!/usr/bin/env python3

import os
import time
import subprocess
import shutil
import logging
from datetime import datetime

TARGET_DIR = "/home/ws/mapdump"
SOURCE_PCD_FILE = "/home/ws/src/point_lio/PCD/scans.pcd"

# 设置日志配置
LOG_DIR = "/home/ws/.log/mapsaver"
os.makedirs(LOG_DIR, exist_ok=True)
log_filename = os.path.join(
    LOG_DIR, datetime.now().strftime("mapsaver-%y%m%d-%H%M%S.log")
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    handlers=[logging.FileHandler(log_filename), logging.StreamHandler()],
)


def check_status():
    try:
        cmd = "echo '' | nc -U /tmp/health_check.sock | od -An -tu1 | tr -d ' '"
        result = subprocess.check_output(cmd, shell=True, text=True).strip()
        return result == "0"
    except Exception as e:
        logging.error(f"Status check error: {e}")
        return False


def save_map():
    os.makedirs(TARGET_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%y%m%d-%H%M%S")
    map_base_path = os.path.join(TARGET_DIR, timestamp)

    # 获取 pgm 和 yaml
    logging.info(f"Saving map as: {map_base_path}")
    save_cmd = f"bash -c 'source /home/ws/.script/envinit.bash && ros2 run nav2_map_server map_saver_cli -f {map_base_path}'"
    subprocess.run(save_cmd, shell=True)

    # 复制 pcd
    target_pcd = f"{map_base_path}.pcd"
    if os.path.exists(SOURCE_PCD_FILE):
        shutil.copy2(SOURCE_PCD_FILE, target_pcd)
        logging.info(f"Copied PCD to: {target_pcd}")
    else:
        logging.warning(f"Source PCD file {SOURCE_PCD_FILE} not found.")


def main():
    while True:
        logging.info("Checking health status...")
        if not check_status():
            logging.warning("Health check failed (status is not 0). Waiting ...")
        else:
            logging.info("Health check passed. Starting map save.")
            save_map()

        time.sleep(10)


if __name__ == "__main__":
    main()
