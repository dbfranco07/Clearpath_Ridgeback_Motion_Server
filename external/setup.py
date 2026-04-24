#!/usr/bin/env python3
"""
setup.py for ridgeback_image_motion — complete version
=======================================================
Replace ~/ridgeback_ws/src/ridgeback_image_motion/setup.py
with this file.

Changes from original:
  + jetson_watchdog entry point added
  + velocity_gate    entry point added
  + simple_wanderer  entry point added (if not already there)
  + launch files added to data_files so ros2 launch can find them
"""

import os
from glob import glob
from setuptools import setup

package_name = "ridgeback_image_motion"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        # Include all launch files
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py") + glob("*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Ridgeback R100 motion, image, web dashboard, and safety nodes",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # ── Ridgeback PC nodes (run via SSH on robot) ──────────────────
            "motion_server    = ridgeback_image_motion.motion_server:main",
            "image_publisher  = ridgeback_image_motion.image_publisher:main",
            # ── Jetson nodes ───────────────────────────────────────────────
            "web_controller   = ridgeback_image_motion.web_controller:main",
            "jetson_watchdog  = ridgeback_image_motion.jetson_watchdog:main",
            "velocity_gate    = ridgeback_image_motion.velocity_gate_node:main",
            # ── PC nodes ───────────────────────────────────────────────────
            "simple_wanderer  = ridgeback_image_motion.simple_wanderer:main",
        ],
    },
)
