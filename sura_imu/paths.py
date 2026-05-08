import os

from ament_index_python.packages import get_package_share_directory


def package_config_path(filename):
    package_share = get_package_share_directory("sura_imu")
    source_candidate = os.path.normpath(
        os.path.join(package_share, "..", "..", "..", "src", "sura_imu", "config", filename)
    )
    if os.path.isdir(os.path.dirname(source_candidate)):
        return source_candidate
    return os.path.join(package_share, "config", filename)
