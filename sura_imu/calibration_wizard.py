import argparse
import math
import os
import sys
import time
from collections import deque
from dataclasses import dataclass
from threading import Lock

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

from sura_imu.paths import package_config_path


def clamp(value, low, high):
    return max(low, min(high, value))


def mean(values):
    if not values:
        return 0.0
    return sum(values) / float(len(values))


def stddev(values):
    if len(values) < 2:
        return 0.0
    mu = mean(values)
    variance = sum((value - mu) ** 2 for value in values) / float(len(values))
    return math.sqrt(variance)


def norm3(vector):
    return math.sqrt(sum(component * component for component in vector))


def wrap_angle_deg(angle_deg):
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    if wrapped == -180.0:
        return 180.0
    return wrapped


def circular_mean_deg(angles_deg):
    if not angles_deg:
        return 0.0
    sin_sum = sum(math.sin(math.radians(angle)) for angle in angles_deg)
    cos_sum = sum(math.cos(math.radians(angle)) for angle in angles_deg)
    return math.degrees(math.atan2(sin_sum, cos_sum))


def circular_std_deg(angles_deg):
    if len(angles_deg) < 2:
        return 0.0
    sin_mean = mean([math.sin(math.radians(angle)) for angle in angles_deg])
    cos_mean = mean([math.cos(math.radians(angle)) for angle in angles_deg])
    r = math.sqrt(sin_mean * sin_mean + cos_mean * cos_mean)
    if r <= 1.0e-9:
        return 180.0
    return math.degrees(math.sqrt(max(0.0, -2.0 * math.log(r))))


def quality_label(score):
    if score >= 90.0:
        return "excellent"
    if score >= 75.0:
        return "good"
    if score >= 55.0:
        return "fair"
    return "poor"


def format_vector(vector, precision=5):
    return "(" + ", ".join(f"{value:.{precision}f}" for value in vector) + ")"


def prompt(message):
    print(message)
    try:
        input("> pulsa Enter para continuar ")
    except KeyboardInterrupt:
        print("\nCalibracion cancelada por el usuario.")
        raise SystemExit(130)


@dataclass
class GyroCalibrationResult:
    bias: tuple[float, float, float]
    noise_stddev: tuple[float, float, float]
    quality_score: float
    quality_label: str
    sample_count: int
    duration_sec: float


@dataclass
class MagnetometerCalibrationResult:
    offset: tuple[float, float, float]
    scale: tuple[float, float, float]
    quality_score: float
    quality_label: str
    sample_count: int
    duration_sec: float
    octant_coverage: int
    residual_percent: float
    axis_ranges: tuple[float, float, float]


@dataclass
class CompassCalibrationResult:
    yaw_offset_deg: float
    heading_stddev_deg: float
    consistency_deg: float
    quality_score: float
    quality_label: str
    heading_samples: int


class CalibrationWizardNode(Node):
    def __init__(self, imu_topic, mag_topic):
        super().__init__("sura_imu_calibration_wizard")
        self._lock = Lock()
        self._latest_imu = None
        self._latest_mag = None
        self._imu_samples = deque(maxlen=50000)
        self._mag_samples = deque(maxlen=50000)

        self.create_subscription(Imu, imu_topic, self._on_imu, 50)
        self.create_subscription(MagneticField, mag_topic, self._on_mag, 50)

    def _on_imu(self, msg):
        stamp = self.get_clock().now().nanoseconds * 1.0e-9
        with self._lock:
            self._latest_imu = (stamp, msg)
            self._imu_samples.append((stamp, msg))

    def _on_mag(self, msg):
        stamp = self.get_clock().now().nanoseconds * 1.0e-9
        with self._lock:
            self._latest_mag = (stamp, msg)
            self._mag_samples.append((stamp, msg))

    def wait_for_topics(self, require_mag=True, timeout_sec=10.0):
        start = time.time()
        while time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._lock:
                imu_ready = self._latest_imu is not None
                mag_ready = self._latest_mag is not None
            if imu_ready and (mag_ready or not require_mag):
                return True
        return False

    def latest_accel(self):
        with self._lock:
            if self._latest_imu is None:
                return None
            msg = self._latest_imu[1]
            return (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            )

    def latest_mag(self):
        with self._lock:
            if self._latest_mag is None:
                return None
            msg = self._latest_mag[1]
            return (
                msg.magnetic_field.x,
                msg.magnetic_field.y,
                msg.magnetic_field.z,
            )

    def capture_gyro(self, duration_sec):
        end_time = time.time() + duration_sec
        captured = []
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
            with self._lock:
                if self._latest_imu is not None:
                    msg = self._latest_imu[1]
                    captured.append(
                        (
                            msg.angular_velocity.x,
                            msg.angular_velocity.y,
                            msg.angular_velocity.z,
                        )
                    )
        return captured

    def capture_mag(self, duration_sec, progress_callback=None):
        end_time = time.time() + duration_sec
        captured = []
        last_progress = 0.0
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
            with self._lock:
                if self._latest_mag is not None:
                    msg = self._latest_mag[1]
                    captured.append(
                        (
                            msg.magnetic_field.x,
                            msg.magnetic_field.y,
                            msg.magnetic_field.z,
                        )
                    )
            elapsed = duration_sec - max(0.0, end_time - time.time())
            if progress_callback and elapsed - last_progress >= 1.0:
                last_progress = elapsed
                progress_callback(captured)
        return captured

    def capture_heading_window(self, duration_sec, mag_offset, mag_scale):
        end_time = time.time() + duration_sec
        headings = []
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)
            accel = self.latest_accel()
            mag = self.latest_mag()
            if accel is None or mag is None:
                continue
            heading = tilt_compensated_heading_deg(accel, mag, mag_offset, mag_scale)
            if heading is not None:
                headings.append(heading)
        return headings


def compute_octant_coverage(samples):
    octants = set()
    for x_value, y_value, z_value in samples:
        octants.add(
            (
                1 if x_value >= 0.0 else 0,
                1 if y_value >= 0.0 else 0,
                1 if z_value >= 0.0 else 0,
            )
        )
    return len(octants)


def compute_mag_calibration(samples):
    xs = [sample[0] for sample in samples]
    ys = [sample[1] for sample in samples]
    zs = [sample[2] for sample in samples]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    min_z, max_z = min(zs), max(zs)

    offset = (
        0.5 * (max_x + min_x),
        0.5 * (max_y + min_y),
        0.5 * (max_z + min_z),
    )

    half_ranges = (
        0.5 * (max_x - min_x),
        0.5 * (max_y - min_y),
        0.5 * (max_z - min_z),
    )
    safe_half_ranges = tuple(max(value, 1.0e-9) for value in half_ranges)
    average_radius = sum(safe_half_ranges) / 3.0
    scale = tuple(average_radius / value for value in safe_half_ranges)

    corrected = []
    for sample in samples:
        corrected.append(
            (
                (sample[0] - offset[0]) * scale[0],
                (sample[1] - offset[1]) * scale[1],
                (sample[2] - offset[2]) * scale[2],
            )
        )

    radii = [norm3(sample) for sample in corrected]
    radius_mean = mean(radii)
    residual_percent = 100.0 * stddev(radii) / max(radius_mean, 1.0e-9)
    octant_coverage = compute_octant_coverage(samples)
    axis_ranges = tuple(2.0 * value for value in half_ranges)

    sample_score = clamp(len(samples) / 1800.0, 0.0, 1.0)
    octant_score = octant_coverage / 8.0
    balance_ratio = min(safe_half_ranges) / max(safe_half_ranges)
    balance_score = clamp(balance_ratio, 0.0, 1.0)
    residual_score = clamp(1.0 - residual_percent / 35.0, 0.0, 1.0)

    quality_score = 100.0 * (
        0.30 * sample_score
        + 0.25 * octant_score
        + 0.20 * balance_score
        + 0.25 * residual_score
    )

    return MagnetometerCalibrationResult(
        offset=offset,
        scale=scale,
        quality_score=quality_score,
        quality_label=quality_label(quality_score),
        sample_count=len(samples),
        duration_sec=0.0,
        octant_coverage=octant_coverage,
        residual_percent=residual_percent,
        axis_ranges=axis_ranges,
    )


def tilt_compensated_heading_deg(accel, mag, mag_offset, mag_scale):
    ax_value, ay_value, az_value = accel
    mx_value, my_value, mz_value = mag

    mx_value = (mx_value - mag_offset[0]) * mag_scale[0]
    my_value = (my_value - mag_offset[1]) * mag_scale[1]
    mz_value = (mz_value - mag_offset[2]) * mag_scale[2]

    accel_norm = norm3(accel)
    mag_norm = norm3((mx_value, my_value, mz_value))
    if accel_norm < 1.0e-6 or mag_norm < 1.0e-9:
        return None

    ax_value /= accel_norm
    ay_value /= accel_norm
    az_value /= accel_norm

    roll = math.atan2(ay_value, az_value)
    pitch = math.atan2(-ax_value, math.sqrt(ay_value * ay_value + az_value * az_value))

    xh = mx_value * math.cos(pitch) + mz_value * math.sin(pitch)
    yh = (
        mx_value * math.sin(roll) * math.sin(pitch)
        + my_value * math.cos(roll)
        - mz_value * math.sin(roll) * math.cos(pitch)
    )

    if abs(xh) < 1.0e-12 and abs(yh) < 1.0e-12:
        return None

    heading = math.degrees(math.atan2(yh, xh))
    return (heading + 360.0) % 360.0


def gyro_quality_score(noise_stddev_xyz):
    average_noise = mean(noise_stddev_xyz)
    score = 100.0 * clamp(1.0 - average_noise / 0.03, 0.0, 1.0)
    return score


def run_gyro_calibration(node, duration_sec):
    print("")
    print("=== Calibracion de giroscopio ===")
    print("Coloca el robot completamente quieto y evita vibraciones.")
    print(f"Voy a medir durante {duration_sec:.1f} s el bias y el ruido del gyro.")
    prompt("Cuando este inmovil, comenzamos la captura")

    start = time.time()
    samples = node.capture_gyro(duration_sec)
    duration = time.time() - start
    if len(samples) < 20:
        raise RuntimeError("No he recibido suficientes muestras del gyro para calibrar.")

    x_values = [sample[0] for sample in samples]
    y_values = [sample[1] for sample in samples]
    z_values = [sample[2] for sample in samples]

    bias = (mean(x_values), mean(y_values), mean(z_values))
    noise_stddev = (stddev(x_values), stddev(y_values), stddev(z_values))
    score = gyro_quality_score(noise_stddev)

    result = GyroCalibrationResult(
        bias=bias,
        noise_stddev=noise_stddev,
        quality_score=score,
        quality_label=quality_label(score),
        sample_count=len(samples),
        duration_sec=duration,
    )

    print("")
    print(f"Muestras usadas: {result.sample_count}")
    print(f"Bias estimado [rad/s]: {format_vector(result.bias, precision=6)}")
    print(f"Ruido std [rad/s]:     {format_vector(result.noise_stddev, precision=6)}")
    print(f"Calidad: {result.quality_label} ({result.quality_score:.1f}/100)")
    return result


def run_magnetometer_calibration(node, duration_sec):
    print("")
    print("=== Calibracion de magnetometro ===")
    print("Inspiracion QGroundControl: mueve el robot despacio por muchas orientaciones.")
    print("Intenta cubrir arriba, abajo, izquierda, derecha, morro arriba y morro abajo.")
    print("Mantente lejos de herramientas, mesas metalicas, baterias sueltas y cables de potencia.")
    prompt("Cuando quieras empiezo a capturar el magnetometro")

    def progress_callback(samples):
        if len(samples) < 10:
            return
        octants = compute_octant_coverage(samples)
        xs = [sample[0] for sample in samples]
        ys = [sample[1] for sample in samples]
        zs = [sample[2] for sample in samples]
        print(
            f"[mag] muestras={len(samples):4d} octantes={octants}/8 "
            f"rangos=({max(xs)-min(xs):.5f}, {max(ys)-min(ys):.5f}, {max(zs)-min(zs):.5f})"
        )

    start = time.time()
    samples = node.capture_mag(duration_sec, progress_callback=progress_callback)
    duration = time.time() - start
    if len(samples) < 100:
        raise RuntimeError("No he recibido suficientes muestras del magnetometro.")

    result = compute_mag_calibration(samples)
    result.duration_sec = duration

    print("")
    print(f"Muestras usadas: {result.sample_count}")
    print(f"Offset hard-iron [T]: {format_vector(result.offset, precision=7)}")
    print(f"Escala soft-iron:     {format_vector(result.scale, precision=5)}")
    print(f"Rangos por eje [T]:   {format_vector(result.axis_ranges, precision=7)}")
    print(f"Cobertura 3D: {result.octant_coverage}/8 octantes")
    print(f"Residual radial: {result.residual_percent:.2f}%")
    print(f"Calidad: {result.quality_label} ({result.quality_score:.1f}/100)")

    if result.octant_coverage < 6:
        print("Consejo: faltan orientaciones. Repite girando mas el robot en 3D.")
    if result.residual_percent > 18.0:
        print("Consejo: hay bastante deformacion magnetica. Alejate de metal y repite.")

    return result


def run_compass_calibration(node, mag_result, sample_duration_sec):
    print("")
    print("=== Calibracion de compas / heading ===")
    print("Voy a estimar un offset de yaw usando orientaciones conocidas.")
    print("Alinea el morro del robot con cada direccion cuando te lo pida.")

    directions = [
        ("Norte", 0.0),
        ("Este", 90.0),
        ("Sur", 180.0),
        ("Oeste", 270.0),
    ]
    offsets = []
    per_step_stddev = []
    total_heading_samples = 0

    mag_offset = mag_result.offset if mag_result is not None else (0.0, 0.0, 0.0)
    mag_scale = mag_result.scale if mag_result is not None else (1.0, 1.0, 1.0)

    for label, reference_deg in directions:
        print("")
        print(f"Coloca el robot apuntando a {label} ({reference_deg:.0f} deg).")
        prompt("Mantelo quieto y empezamos la captura")
        heading_samples = node.capture_heading_window(sample_duration_sec, mag_offset, mag_scale)
        if len(heading_samples) < 15:
            raise RuntimeError(f"No he podido estimar bien el heading en la orientacion {label}.")

        measured_heading = circular_mean_deg(heading_samples)
        heading_std = circular_std_deg(heading_samples)
        offset = wrap_angle_deg(reference_deg - measured_heading)

        total_heading_samples += len(heading_samples)
        offsets.append(offset)
        per_step_stddev.append(heading_std)

        print(
            f"{label}: heading medido={measured_heading:.1f} deg, "
            f"offset parcial={offset:.1f} deg, dispersion={heading_std:.2f} deg"
        )

    yaw_offset_deg = circular_mean_deg(offsets)
    consistency_deg = circular_std_deg(offsets)
    heading_stddev_deg = mean(per_step_stddev)

    std_score = clamp(1.0 - heading_stddev_deg / 12.0, 0.0, 1.0)
    consistency_score = clamp(1.0 - consistency_deg / 20.0, 0.0, 1.0)
    quality_score = 100.0 * (0.55 * std_score + 0.45 * consistency_score)

    result = CompassCalibrationResult(
        yaw_offset_deg=yaw_offset_deg,
        heading_stddev_deg=heading_stddev_deg,
        consistency_deg=consistency_deg,
        quality_score=quality_score,
        quality_label=quality_label(quality_score),
        heading_samples=total_heading_samples,
    )

    print("")
    print(f"Offset final de yaw: {result.yaw_offset_deg:.2f} deg")
    print(f"Dispersion media:    {result.heading_stddev_deg:.2f} deg")
    print(f"Consistencia global: {result.consistency_deg:.2f} deg")
    print(f"Calidad: {result.quality_label} ({result.quality_score:.1f}/100)")

    if result.consistency_deg > 10.0:
        print("Consejo: la relacion heading/direccion real no ha sido muy consistente.")
        print("Repite la calibracion cuidando la alineacion fisica del robot.")

    return result


def write_calibration_yaml(output_path, gyro_result, mag_result, compass_result, args):
    directory = os.path.dirname(output_path)
    if directory:
        os.makedirs(directory, exist_ok=True)

    lines = [
        "sura_imu_calibration:",
        "  metadata:",
        f"    generated_at_unix: {time.time():.3f}",
        f"    imu_topic: '{args.imu_topic}'",
        f"    mag_topic: '{args.mag_topic}'",
        f"    mode: '{args.mode}'",
        "  gyro:",
    ]

    if gyro_result is None:
        lines.extend(
            [
                "    available: false",
            ]
        )
    else:
        lines.extend(
            [
                "    available: true",
                f"    quality_score: {gyro_result.quality_score:.3f}",
                f"    quality_label: '{gyro_result.quality_label}'",
                f"    sample_count: {gyro_result.sample_count}",
                f"    duration_sec: {gyro_result.duration_sec:.3f}",
                f"    bias_x: {gyro_result.bias[0]:.10f}",
                f"    bias_y: {gyro_result.bias[1]:.10f}",
                f"    bias_z: {gyro_result.bias[2]:.10f}",
                f"    noise_stddev_x: {gyro_result.noise_stddev[0]:.10f}",
                f"    noise_stddev_y: {gyro_result.noise_stddev[1]:.10f}",
                f"    noise_stddev_z: {gyro_result.noise_stddev[2]:.10f}",
            ]
        )

    lines.append("  magnetometer:")
    if mag_result is None:
        lines.append("    available: false")
    else:
        lines.extend(
            [
                "    available: true",
                f"    quality_score: {mag_result.quality_score:.3f}",
                f"    quality_label: '{mag_result.quality_label}'",
                f"    sample_count: {mag_result.sample_count}",
                f"    duration_sec: {mag_result.duration_sec:.3f}",
                f"    offset_x: {mag_result.offset[0]:.10f}",
                f"    offset_y: {mag_result.offset[1]:.10f}",
                f"    offset_z: {mag_result.offset[2]:.10f}",
                f"    scale_x: {mag_result.scale[0]:.10f}",
                f"    scale_y: {mag_result.scale[1]:.10f}",
                f"    scale_z: {mag_result.scale[2]:.10f}",
                f"    octant_coverage: {mag_result.octant_coverage}",
                f"    residual_percent: {mag_result.residual_percent:.4f}",
            ]
        )

    lines.append("  compass:")
    if compass_result is None:
        lines.append("    available: false")
    else:
        lines.extend(
            [
                "    available: true",
                f"    quality_score: {compass_result.quality_score:.3f}",
                f"    quality_label: '{compass_result.quality_label}'",
                f"    yaw_offset_deg: {compass_result.yaw_offset_deg:.6f}",
                f"    heading_stddev_deg: {compass_result.heading_stddev_deg:.6f}",
                f"    consistency_deg: {compass_result.consistency_deg:.6f}",
                f"    heading_samples: {compass_result.heading_samples}",
            ]
        )

    with open(output_path, "w", encoding="utf-8") as output_file:
        output_file.write("\n".join(lines) + "\n")


def default_output_path():
    return package_config_path("sura_imu_calibration.yaml")


def read_saved_magnetometer_calibration(input_path):
    if not input_path or not os.path.exists(input_path):
        return None

    current_section = None
    values = {}
    with open(input_path, "r", encoding="utf-8") as input_file:
        for raw_line in input_file:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if line.endswith(":") and not line.startswith("generated_at_unix"):
                current_section = line[:-1]
                continue
            if ":" not in line or current_section != "magnetometer":
                continue
            key, value = [part.strip() for part in line.split(":", 1)]
            values[key] = value.strip("'\"")

    if values.get("available", "false").lower() != "true":
        return None

    try:
        return MagnetometerCalibrationResult(
            offset=(
                float(values["offset_x"]),
                float(values["offset_y"]),
                float(values["offset_z"]),
            ),
            scale=(
                float(values["scale_x"]),
                float(values["scale_y"]),
                float(values["scale_z"]),
            ),
            quality_score=float(values.get("quality_score", "0.0")),
            quality_label=values.get("quality_label", "unknown"),
            sample_count=int(values.get("sample_count", "0")),
            duration_sec=float(values.get("duration_sec", "0.0")),
            octant_coverage=int(values.get("octant_coverage", "0")),
            residual_percent=float(values.get("residual_percent", "0.0")),
            axis_ranges=(0.0, 0.0, 0.0),
        )
    except KeyError:
        return None


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="Asistente guiado por consola para calibrar gyro, magnetometro y compas."
    )
    parser.add_argument(
        "--mode",
        choices=["wizard", "gyro", "mag", "compass"],
        default="wizard",
        help="Tipo de calibracion a ejecutar.",
    )
    parser.add_argument(
        "--imu-topic",
        default="/imu_broadcaster/imu",
        help="Topic sensor_msgs/Imu para datos crudos.",
    )
    parser.add_argument(
        "--mag-topic",
        default="/magnetometer_broadcaster/mag",
        help="Topic sensor_msgs/MagneticField para el magnetometro.",
    )
    parser.add_argument(
        "--gyro-duration",
        type=float,
        default=8.0,
        help="Segundos de captura en la calibracion de gyro.",
    )
    parser.add_argument(
        "--mag-duration",
        type=float,
        default=35.0,
        help="Segundos de captura para el barrido 3D del magnetometro.",
    )
    parser.add_argument(
        "--compass-duration",
        type=float,
        default=4.0,
        help="Segundos de captura en cada orientacion del compas.",
    )
    parser.add_argument(
        "--input-calibration",
        default="",
        help="YAML previo generado por este asistente para reutilizar la calibracion magnetica.",
    )
    parser.add_argument(
        "--output",
        default=default_output_path(),
        help="Ruta YAML donde guardar resultados.",
    )
    parser.add_argument(
        "--skip-save",
        action="store_true",
        help="No guardar resultados en disco.",
    )
    return parser


def run_cli(cli_args=None):
    parser = build_argument_parser()
    args = parser.parse_args(cli_args)

    rclpy.init(args=sys.argv)
    node = CalibrationWizardNode(args.imu_topic, args.mag_topic)

    gyro_result = None
    mag_result = None
    compass_result = None

    try:
        require_mag = args.mode in ("wizard", "mag", "compass")
        print("")
        print("Esperando datos ROS 2 para la calibracion...")
        if not node.wait_for_topics(require_mag=require_mag, timeout_sec=12.0):
            raise RuntimeError(
                "No llegan datos de IMU o magnetometro. Revisa topics y broadcasters."
            )

        print("Datos detectados. Empezamos el asistente.")

        if args.mode in ("wizard", "gyro"):
            gyro_result = run_gyro_calibration(node, args.gyro_duration)

        if args.mode in ("wizard", "mag"):
            mag_result = run_magnetometer_calibration(node, args.mag_duration)

        if args.mode in ("wizard", "compass"):
            if mag_result is None and args.input_calibration:
                mag_result = read_saved_magnetometer_calibration(args.input_calibration)
                if mag_result is not None:
                    print("")
                    print(
                        "Calibracion magnetica previa cargada desde "
                        f"{args.input_calibration}"
                    )
            if mag_result is None:
                print("")
                print("No he hecho calibracion mag en esta ejecucion; usare offset=0 y scale=1.")
            compass_result = run_compass_calibration(node, mag_result, args.compass_duration)

        if not args.skip_save:
            write_calibration_yaml(args.output, gyro_result, mag_result, compass_result, args)
            print("")
            print(f"Resultados guardados en: {args.output}")

        print("")
        print("Calibracion terminada.")
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")
        return 130
    except Exception as exc:
        print("")
        print(f"Error: {exc}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


def main():
    return run_cli()


def main_gyro():
    return run_cli(["--mode", "gyro", *sys.argv[1:]])


def main_mag():
    return run_cli(["--mode", "mag", *sys.argv[1:]])


def main_compass():
    return run_cli(["--mode", "compass", *sys.argv[1:]])


if __name__ == "__main__":
    raise SystemExit(main())
