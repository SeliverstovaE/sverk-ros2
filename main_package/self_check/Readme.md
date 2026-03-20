## Автоматическая проверка (self_check)

Пакет `self_check` содержит скрипт `selfcheck.py` для **самодиагностики**: проверяет, что ключевые подсистемы (PX4 DDS/px4_msgs, камера, ArUco, VPE и состояние бортового компьютера) **работают и публикуют данные**.

### Запуск

```bash
ros2 run self_check selfcheck.py
```

### Что проверяется

- **FCU (PX4 DDS)**: наличие связи с PX4 (`VehicleStatus`) и базовые статусы (`TimesyncStatus`, `FailsafeFlags`), а также расширенная проверка **батареи** по `BatteryStatus`.
- **TelemetryStatus (PX4 DDS)**: вывод полей телеметрии из `TelemetryStatus` (если доступны в вашей версии `px4_msgs`).
- **VehicleControlMode / flight mode (PX4 DDS)**: вывод **полетного режима** (по `VehicleStatus.nav_state`).
- **PX4 Local Position (PX4 DDS)**: вывод локальной позиции PX4 (`VehicleLocalPosition`) одной строкой: позиция/скорости/heading/флаги валидности.
- **IMU (PX4 DDS)**: обновление `SensorCombined`.
- **Attitude (PX4 DDS)**: обновление `VehicleAttitude`, вывод углов и предупреждение при сильном наклоне.
- **Local position (ArUco)**: обновление `PoseWithCovarianceStamped` из `--pose-topic`.
- **Velocity estimation (from ArUco pose)**: оценка максимальных скоростей по окну измерений.
- **Camera**: обновление `sensor_msgs/Image`.
- **ArUco markers**: обновление массива маркеров (`--markers-topic`, тип через `--markers-pytype`).
- **VPE (vision input vs PX4 estimate)**: сравнение входной одометрии (`--visual-odom-topic`) и PX4 odometry (`--vehicle-odometry-topic`).
- **SBC health**: свободное место на диске + (на Raspberry Pi) `vcgencmd get_throttled`. В Docker без `/dev/vchiq` проверка троттлинга будет пропущена.
- **CPU usage**: загрузка CPU по `/proc/stat`.

> Внимательно относитесь к строкам `WARNING`. Особенно к **Velocity estimation**: при проблемах с оценкой скоростей автономные режимы лучше не запускать.

### Часто используемые параметры

#### Топики PX4 DDS

- `--vehicle-status-topic` (по умолчанию `/fmu/out/vehicle_status`)
- `--vehicle-control-mode-topic` (по умолчанию `/fmu/out/vehicle_control_mode`)
- `--telemetry-status-topic` (по умолчанию `/fmu/in/telemetry_status`)
- `--vehicle-local-position-topic` (по умолчанию `/fmu/out/vehicle_local_position`)
- `--imu-topic` (по умолчанию `/fmu/out/sensor_combined`)
- `--attitude-topic` (по умолчанию `/fmu/out/vehicle_attitude`)
- `--battery-topic` (по умолчанию `/fmu/out/battery_status`)
- `--failsafe-topic` (по умолчанию `/fmu/out/failsafe_flags`)
- `--estimator-flags-topic` (по умолчанию `/fmu/out/estimator_status_flags`)

#### VPE

- `--visual-odom-topic` (по умолчанию `/fmu/in/vehicle_visual_odometry`)
- `--vehicle-odometry-topic` (по умолчанию `/fmu/out/vehicle_odometry`)
- `--vpe-compare-window` (сек), `--vpe-pos-threshold` (м), `--vpe-yaw-threshold-deg` (град)

#### ArUco / камера

- `--pose-topic` (по умолчанию `/aruco_map/pose_cov`)
- `--markers-topic` (по умолчанию `/markers`)
- `--markers-pytype` (по умолчанию `aruco_det_loc.msg.MarkerArray`)
- `--image-topic` (по умолчанию `/camera_1/image_raw`)

#### Батарея

- `--battery-cells` (кол-во банок; если 0 — проверка “на банку” не выполняется)
- `--battery-min-cell-v` (по умолчанию 3.50)
- `--battery-min-remaining` (по умолчанию 0.20)
- `--battery-warn-level` (по умолчанию 2)
- `--battery-max-current-a` (0 = отключено)
- `--battery-max-temp-c` (0 = отключено)

### Отключение проверок

Для отключения конкретной проверки используйте флаги `--no-*`, например:

```bash
ros2 run self_check selfcheck.py --no-vpe --no-sbc-health
```
