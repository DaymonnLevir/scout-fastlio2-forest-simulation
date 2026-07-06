#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


FAST_TOPIC = "/Odometry"
GAZEBO_TOPIC = "/ground_truth/odom_raw"


def stamp_to_seconds(stamp, fallback_timestamp_ns):
    stamp_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9

    if stamp_time > 0.0:
        return stamp_time

    return float(fallback_timestamp_ns) * 1e-9


def quaternion_to_yaw(orientation):
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(sin_yaw, cos_yaw)


def organize_track(samples):
    if len(samples) < 2:
        raise RuntimeError("A trajetória possui menos de duas amostras.")

    data = np.asarray(samples, dtype=float)

    order = np.argsort(data[:, 0])
    data = data[order]

    _, unique_indices = np.unique(data[:, 0], return_index=True)
    data = data[unique_indices]

    return {
        "t": data[:, 0],
        "x": data[:, 1],
        "y": data[:, 2],
        "yaw": data[:, 3],
    }


def read_bag(bag_path):
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path),
        storage_id="sqlite3",
    )

    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }

    required_topics = [FAST_TOPIC, GAZEBO_TOPIC]

    for topic in required_topics:
        if topic not in topic_types:
            raise RuntimeError(f"Tópico não encontrado no rosbag: {topic}")

    message_types = {
        topic: get_message(topic_types[topic])
        for topic in required_topics
    }

    fast_samples = []
    gazebo_samples = []

    while reader.has_next():
        topic, serialized_data, bag_timestamp = reader.read_next()

        if topic not in message_types:
            continue

        message = deserialize_message(
            serialized_data,
            message_types[topic],
        )

        time_seconds = stamp_to_seconds(
            message.header.stamp,
            bag_timestamp,
        )

        position = message.pose.pose.position
        orientation = message.pose.pose.orientation

        sample = (
            time_seconds,
            position.x,
            position.y,
            quaternion_to_yaw(orientation),
        )

        if topic == FAST_TOPIC:
            fast_samples.append(sample)
        elif topic == GAZEBO_TOPIC:
            gazebo_samples.append(sample)

    return organize_track(fast_samples), organize_track(gazebo_samples)


def detect_motion_interval(gazebo, overlap_start, overlap_end):
    mask = (
        (gazebo["t"] >= overlap_start)
        & (gazebo["t"] <= overlap_end)
    )

    t = gazebo["t"][mask]
    x = gazebo["x"][mask]
    y = gazebo["y"][mask]

    if len(t) < 3:
        return overlap_start, overlap_end

    dt = np.diff(t)
    displacement = np.hypot(np.diff(x), np.diff(y))

    speed = np.divide(
        displacement,
        dt,
        out=np.zeros_like(displacement),
        where=dt > 1e-6,
    )

    # Ignora pequenas oscilações físicas com o robô parado.
    moving_indices = np.where(speed > 0.02)[0]

    if len(moving_indices) == 0:
        print("Movimento não detectado automaticamente.")
        print("Será utilizado todo o intervalo gravado.")
        return overlap_start, overlap_end

    first_motion = t[moving_indices[0]]
    last_motion = t[moving_indices[-1] + 1]

    margin_seconds = 2.0

    analysis_start = max(
        overlap_start,
        first_motion - margin_seconds,
    )

    analysis_end = min(
        overlap_end,
        last_motion + margin_seconds,
    )

    return analysis_start, analysis_end


def crop_track(track, start_time, end_time):
    mask = (
        (track["t"] >= start_time)
        & (track["t"] <= end_time)
    )

    return {
        key: values[mask]
        for key, values in track.items()
    }


def transform_to_initial_frame(track, reference_time):
    initial_x = np.interp(
        reference_time,
        track["t"],
        track["x"],
    )

    initial_y = np.interp(
        reference_time,
        track["t"],
        track["y"],
    )

    yaw_index = int(
        np.argmin(np.abs(track["t"] - reference_time))
    )

    initial_yaw = track["yaw"][yaw_index]

    dx = track["x"] - initial_x
    dy = track["y"] - initial_y

    cosine = math.cos(initial_yaw)
    sine = math.sin(initial_yaw)

    # Rotação de -yaw_inicial.
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy

    return {
        "t": track["t"],
        "x": local_x,
        "y": local_y,
        "yaw": track["yaw"] - initial_yaw,
        "initial_x": initial_x,
        "initial_y": initial_y,
        "initial_yaw": initial_yaw,
    }


def path_length(x, y):
    if len(x) < 2:
        return 0.0

    return float(
        np.sum(
            np.hypot(
                np.diff(x),
                np.diff(y),
            )
        )
    )


def save_synchronized_csv(
    output_path,
    time_values,
    fast_x,
    fast_y,
    gazebo_x,
    gazebo_y,
    errors,
):
    with output_path.open("w", newline="") as file:
        writer = csv.writer(file)

        writer.writerow([
            "tempo_s",
            "fastlio_x_m",
            "fastlio_y_m",
            "gazebo_x_m",
            "gazebo_y_m",
            "erro_posicao_m",
        ])

        initial_time = time_values[0]

        for values in zip(
            time_values,
            fast_x,
            fast_y,
            gazebo_x,
            gazebo_y,
            errors,
        ):
            t, fx, fy, gx, gy, error = values

            writer.writerow([
                f"{t - initial_time:.6f}",
                f"{fx:.6f}",
                f"{fy:.6f}",
                f"{gx:.6f}",
                f"{gy:.6f}",
                f"{error:.6f}",
            ])


def save_metrics(output_path, metrics):
    with output_path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["metrica", "valor", "unidade"])

        for name, value, unit in metrics:
            writer.writerow([
                name,
                f"{value:.6f}",
                unit,
            ])


def create_trajectory_plot(output_path, fast, gazebo):
    planned_rectangle = np.array([
        [0.0, 0.0],
        [8.0, 0.0],
        [8.0, 6.0],
        [0.0, 6.0],
        [0.0, 0.0],
    ])

    plt.figure(figsize=(10, 8))

    plt.plot(
        planned_rectangle[:, 0],
        planned_rectangle[:, 1],
        linestyle="--",
        linewidth=1.5,
        label="Trajetória planejada",
    )

    plt.plot(
        gazebo["x"],
        gazebo["y"],
        linewidth=2.0,
        label="Gazebo — ground truth",
    )

    plt.plot(
        fast["x"],
        fast["y"],
        linewidth=1.5,
        label="FAST-LIO",
    )

    plt.scatter(
        [0.0],
        [0.0],
        marker="o",
        s=80,
        label="Início",
    )

    plt.scatter(
        [gazebo["x"][-1]],
        [gazebo["y"][-1]],
        marker="x",
        s=100,
        label="Final Gazebo",
    )

    plt.scatter(
        [fast["x"][-1]],
        [fast["y"][-1]],
        marker="+",
        s=120,
        label="Final FAST-LIO",
    )

    plt.xlabel("Posição local X (m)")
    plt.ylabel("Posição local Y (m)")
    plt.title("Comparação das trajetórias — FAST-LIO e Gazebo")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(output_path, dpi=200)
    plt.close()


def create_error_plot(output_path, time_values, errors):
    relative_time = time_values - time_values[0]

    plt.figure(figsize=(10, 5))

    plt.plot(
        relative_time,
        errors,
        linewidth=1.5,
        label="Erro de posição",
    )

    plt.xlabel("Tempo desde o início da missão (s)")
    plt.ylabel("Erro horizontal (m)")
    plt.title("Erro de posição do FAST-LIO ao longo do tempo")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(output_path, dpi=200)
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Compara a trajetória estimada pelo FAST-LIO "
            "com o ground truth do Gazebo."
        )
    )

    parser.add_argument(
        "bag",
        type=Path,
        help="Diretório do rosbag.",
    )

    args = parser.parse_args()
    bag_path = args.bag.resolve()

    if not bag_path.exists():
        raise RuntimeError(
            f"Diretório do rosbag não encontrado: {bag_path}"
        )

    output_directory = bag_path / "analysis"
    output_directory.mkdir(exist_ok=True)

    print("Lendo o rosbag...")
    fast, gazebo = read_bag(bag_path)

    print(f"Amostras FAST-LIO: {len(fast['t'])}")
    print(f"Amostras Gazebo:   {len(gazebo['t'])}")

    overlap_start = max(
        fast["t"][0],
        gazebo["t"][0],
    )

    overlap_end = min(
        fast["t"][-1],
        gazebo["t"][-1],
    )

    if overlap_end <= overlap_start:
        raise RuntimeError(
            "As duas trajetórias não possuem intervalo comum."
        )

    analysis_start, analysis_end = detect_motion_interval(
        gazebo,
        overlap_start,
        overlap_end,
    )

    print(
        "Intervalo analisado: "
        f"{analysis_end - analysis_start:.2f} s"
    )

    fast = crop_track(
        fast,
        analysis_start,
        analysis_end,
    )

    gazebo = crop_track(
        gazebo,
        analysis_start,
        analysis_end,
    )

    if len(fast["t"]) < 2 or len(gazebo["t"]) < 2:
        raise RuntimeError(
            "Não há amostras suficientes no intervalo da missão."
        )

    reference_time = max(
        fast["t"][0],
        gazebo["t"][0],
    )

    fast_local = transform_to_initial_frame(
        fast,
        reference_time,
    )

    gazebo_local = transform_to_initial_frame(
        gazebo,
        reference_time,
    )

    # Usa os instantes do FAST-LIO como base e interpola o Gazebo.
    valid_mask = (
        (fast_local["t"] >= gazebo_local["t"][0])
        & (fast_local["t"] <= gazebo_local["t"][-1])
    )

    synchronized_time = fast_local["t"][valid_mask]
    synchronized_fast_x = fast_local["x"][valid_mask]
    synchronized_fast_y = fast_local["y"][valid_mask]

    synchronized_gazebo_x = np.interp(
        synchronized_time,
        gazebo_local["t"],
        gazebo_local["x"],
    )

    synchronized_gazebo_y = np.interp(
        synchronized_time,
        gazebo_local["t"],
        gazebo_local["y"],
    )

    position_errors = np.hypot(
        synchronized_fast_x - synchronized_gazebo_x,
        synchronized_fast_y - synchronized_gazebo_y,
    )

    mean_error = float(np.mean(position_errors))
    median_error = float(np.median(position_errors))
    rmse = float(np.sqrt(np.mean(position_errors ** 2)))
    maximum_error = float(np.max(position_errors))
    percentile_95 = float(np.percentile(position_errors, 95))
    final_error = float(position_errors[-1])

    fast_closure_error = float(
        np.hypot(
            synchronized_fast_x[-1],
            synchronized_fast_y[-1],
        )
    )

    gazebo_closure_error = float(
        np.hypot(
            synchronized_gazebo_x[-1],
            synchronized_gazebo_y[-1],
        )
    )

    fast_distance = path_length(
        synchronized_fast_x,
        synchronized_fast_y,
    )

    gazebo_distance = path_length(
        synchronized_gazebo_x,
        synchronized_gazebo_y,
    )

    duration = float(
        synchronized_time[-1] - synchronized_time[0]
    )

    metrics = [
        ("duracao_analisada", duration, "s"),
        ("erro_medio", mean_error, "m"),
        ("erro_mediano", median_error, "m"),
        ("rmse_posicao", rmse, "m"),
        ("erro_maximo", maximum_error, "m"),
        ("percentil_95_erro", percentile_95, "m"),
        ("erro_final_fastlio_vs_gazebo", final_error, "m"),
        ("erro_fechamento_fastlio", fast_closure_error, "m"),
        ("erro_fechamento_gazebo", gazebo_closure_error, "m"),
        ("distancia_fastlio", fast_distance, "m"),
        ("distancia_gazebo", gazebo_distance, "m"),
    ]

    save_synchronized_csv(
        output_directory / "trajetorias_sincronizadas.csv",
        synchronized_time,
        synchronized_fast_x,
        synchronized_fast_y,
        synchronized_gazebo_x,
        synchronized_gazebo_y,
        position_errors,
    )

    save_metrics(
        output_directory / "metricas.csv",
        metrics,
    )

    create_trajectory_plot(
        output_directory / "trajetorias_xy.png",
        {
            "x": synchronized_fast_x,
            "y": synchronized_fast_y,
        },
        {
            "x": synchronized_gazebo_x,
            "y": synchronized_gazebo_y,
        },
    )

    create_error_plot(
        output_directory / "erro_posicao.png",
        synchronized_time,
        position_errors,
    )

    print()
    print("============================================")
    print(" RESULTADOS")
    print("============================================")
    print(f"Erro médio:          {mean_error:.4f} m")
    print(f"RMSE:                {rmse:.4f} m")
    print(f"Erro máximo:         {maximum_error:.4f} m")
    print(f"Percentil 95%:       {percentile_95:.4f} m")
    print(f"Erro final:          {final_error:.4f} m")
    print(f"Fechamento FAST-LIO: {fast_closure_error:.4f} m")
    print(f"Fechamento Gazebo:   {gazebo_closure_error:.4f} m")
    print(f"Distância FAST-LIO:  {fast_distance:.2f} m")
    print(f"Distância Gazebo:    {gazebo_distance:.2f} m")
    print()
    print("Arquivos gerados em:")
    print(output_directory)


if __name__ == "__main__":
    main()
