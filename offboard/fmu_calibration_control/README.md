# fmu_calibration_control

Узел ROS 2 для **калибровки полётника** и **быстрого дизарма** через топик `/fmu/in/vehicle_command` (uXRCE-DDS, [dds_topics](https://docs.px4.io/main/en/middleware/dds_topics)).

## Зависимости

- `px4_msgs` (из `sverk_drone/px4/px4_msgs`)
- PX4 с uXRCE-DDS agent и подпиской на `/fmu/in/vehicle_command`

## Сборка

Из `sverk_ws`:

```bash
colcon build --packages-select fmu_calibration_control
source install/setup.bash
```

## Запуск

```bash
ros2 launch fmu_calibration_control calibration_control.launch.py
```

Или только узел:

```bash
ros2 run fmu_calibration_control calibration_control
```

## API

### Дизарм и аварийное отключение

- **`/fmu_calibration_control/disarm`** — обычный дизарм. PX4 примет только если дрон «на земле» (иначе: *Disarming denied: not landed*).
- **`/fmu_calibration_control/force_disarm`** — принудительный дизарм (param2=21196), обход проверки «not landed». Использовать, когда нужно снять арм в воздухе без остановки моторов по логике flight termination.
- **`/fmu_calibration_control/kill_switch`** — **Kill Switch**: `VEHICLE_CMD_DO_FLIGHTTERMINATION` — немедленная остановка моторов (в т.ч. в воздухе). Эквивалент аварийной кнопки.

```bash
# Обычный дизарм (только когда приземлился)
ros2 service call /fmu_calibration_control/disarm std_srvs/srv/Trigger

# Принудительный дизарм (игнорирует "not landed")
ros2 service call /fmu_calibration_control/force_disarm std_srvs/srv/Trigger

# Kill Switch — немедленная остановка моторов
ros2 service call /fmu_calibration_control/kill_switch std_srvs/srv/Trigger
```

### Калибровка

Топик `/fmu_calibration_control/request_calibration` (тип `std_msgs/msg/UInt8`). Значение = тип калибровки (см. MAV_CMD_PREFLIGHT_CALIBRATION, param1):

| Значение | Тип        | Описание              |
|----------|-------------|------------------------|
| 0        | gyro        | Гироскоп               |
| 1        | mag         | Магнитометр (компас)   |
| 2        | baro        | Барометр               |
| 3        | temperature | Температура (PX4)      |
| 4        | accel       | Акселерометр           |
| 5        | level       | Уровень горизонта      |

Примеры:

```bash
# Калибровка гироскопа
ros2 topic pub --once /fmu_calibration_control/request_calibration std_msgs/msg/UInt8 "{data: 0}"

# Калибровка магнитометра
ros2 topic pub --once /fmu_calibration_control/request_calibration std_msgs/msg/UInt8 "{data: 1}"

# Калибровка акселерометра
ros2 topic pub --once /fmu_calibration_control/request_calibration std_msgs/msg/UInt8 "{data: 4}"

# Быстрый дизарм / Kill Switch при "not landed"
ros2 service call /fmu_calibration_control/force_disarm std_srvs/srv/Trigger
ros2 service call /fmu_calibration_control/kill_switch std_srvs/srv/Trigger
```

**Важно:** калибровка принимается PX4 только в pre-arm состоянии (полётник не в полёте). Дизарм можно вызывать в любой момент для экстренного снятия арма.

## Связь с DDS-топиками PX4

По [dds_topics](https://docs.px4.io/main/en/middleware/dds_topics):

- **Subscription:** `/fmu/in/vehicle_command` — тип `px4_msgs/msg/VehicleCommand`.
- Команды: `VEHICLE_CMD_PREFLIGHT_CALIBRATION = 241`, `VEHICLE_CMD_COMPONENT_ARM_DISARM = 400`.

Ответы и подтверждения можно смотреть в `/fmu/out/vehicle_command_ack` и `/fmu/out/vehicle_status`.
