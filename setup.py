from setuptools import setup
import os
from glob import glob


package_name = "sura_imu"


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="usuario",
    maintainer_email="iedo@uji.es",
    description="SURA IMU pipeline and attitude estimation launch package",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "calibration_wizard = sura_imu.calibration_wizard:main",
            "calibration_filter = sura_imu.calibration_filter:main",
            "calibrate_gyro = sura_imu.calibration_wizard:main_gyro",
            "calibrate_magnetometer = sura_imu.calibration_wizard:main_mag",
            "calibrate_compass = sura_imu.calibration_wizard:main_compass",
        ],
    },
)
