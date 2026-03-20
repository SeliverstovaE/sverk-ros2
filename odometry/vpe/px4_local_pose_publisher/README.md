# px4_local_pose_publisher

ROS 2-узел, который подписывается на визуальную позу (например, из ArUco/VIO), совмещает её с локальной позицией PX4 через offset и публикует оценку позиции в полётный контроллер PX4 по протоколу uXRCE-DDS (топик `vehicle_visual_odometry`).

## Зависимости

- ROS 2 Humble
- px4_ros2_interface_lib (LocalPositionMeasurementInterface)
- px4_msgs
- tf2_ros, geometry_msgs
- PX4 1.15.x с uXRCE-DDS

## Запуск

```bash
ros2 launch px4_local_pose_publisher pose_subscriber_tf2.launch.py
```

С переопределением параметров через аргументы launch:

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource([...]),
    launch_arguments={
        'camera_frame_id': 'main_camera_optical',
        'publish_rate_hz': '35.0',
    }.items()
)
```

## Топики

| Направление | Топик | Тип | Описание |
|------------|--------|-----|----------|
| Вход | `pose_cov_topic` (по умолч. `aruco_map/pose_cov`) | `geometry_msgs/msg/PoseWithCovarianceStamped` | Поза камеры в системе ArUco/карты (с ковариацией). |
| Вход | `vehicle_local_position_topic` (по умолч. `/fmu/out/vehicle_local_position`) | `px4_msgs/msg/VehicleLocalPosition` | Локальная позиция от PX4 для вычисления offset. |
| Выход | `fmu/in/vehicle_visual_odometry` (через px4_ros2) | px4_msgs (VehicleOdometry) | Визуальная одометрия в PX4 для EKF2. |

## Передаваемые параметры

Все параметры объявлены в узле и могут передаваться через launch (файл или `launch_arguments`).

### Топики

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `pose_cov_topic` | string | `aruco_map/pose_cov` | Топик входящей позы с ковариацией (ArUco/VIO). |
| `vehicle_local_position_topic` | string | `/fmu/out/vehicle_local_position` | Топик локальной позиции PX4 для расчёта offset. |

### Системы координат (TF frame_id)

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `camera_frame_id` | string | `main_camera_optical` | Frame_id камеры, в которой задана входящая поза. |
| `base_link_frame_id` | string | `base_link` | Frame_id корпуса дрона. |
| `world_frame_id` | string | `map` | Глобальная мировая система (например, map). |
| `aruco_map_frame_id` | string | `aruco_map_detected` | Система координат карты ArUco (источник позы). |
| `offset_frame_id` | string | `aruco_map` | Frame_id для публикации статического TF offset (map → aruco_map). |

### Поведение и таймауты

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `data_timeout_sec` | float | `0.5` | Таймаут (с) без данных с `pose_cov_topic`; при превышении отправляется fallback с высокой дисперсией. |
| `offset_timeout` | float | `3.0` (в коде) / `5.0` (в launch) | Максимальное время жизни offset (с); при `allow_offset_timeout_reset == true` по истечении offset сбрасывается. |
| `allow_offset_timeout_reset` | bool | `false` | Разрешить ли сброс offset по таймауту в полёте (обычно выключено для стабильности). |
| `allow_send_vio` | bool | `false` | Разрешить ли отправку данных с одометии по Aruco на полетный контроллер (обычно выключено для стабильности полетов). |
| `publish_rate_hz` | float | `30.0` (в коде) / `35.0` (в launch) | Частота (Гц) публикации в PX4. |
| `fallback_variance` | float | `0.5` | Дисперсия позиции/ориентации при отправке fallback (когда нет актуальной визуальной позы). |

## Поведение

1. Узел подписывается на `pose_cov_topic` и на `vehicle_local_position_topic`.
2. При первом появлении и визуальной позы, и позиции PX4 вычисляется **offset** (преобразование world → aruco_map) и публикуется статический TF `world_frame_id` → `offset_frame_id`.
3. Каждые `1/publish_rate_hz` секунд поза из ArUco переводится в world с учётом offset, затем в base_link через TF (camera → base_link), конвертируется в NED и отправляется в PX4 через `LocalPositionMeasurementInterface` (топик `fmu/in/vehicle_visual_odometry`) с поддержкой `reset_counter`.
4. При сбросе offset (релокализация и т.п.) увеличивается `reset_counter`, чтобы EKF2 корректно обработал разрыв оценки.
5. Если данные с `pose_cov_topic` не приходят дольше `data_timeout_sec`, в PX4 уходит fallback-измерение с высокой дисперсией.

---

## API (заголовки): `pose_subscriber_tf2`

Заголовки узла `pose_subscriber_tf2`: классы и параметры, используемые при публикации VIO в PX4.

### Файлы

| Файл | Описание |
|------|----------|
| `pose_subscriber_tf2.hpp` | Узел VPE: подписка на позу ArUco/VIO и `vehicle_local_position`, публикация в PX4. |
| `pose_subscriber.hpp` | Подписчик позы (другая нода). |
| `local_pose_publisher.hpp` | Публикатор локальной позы (другая нода). |

Документация ниже относится к **pose_subscriber_tf2** (основной узел для VIO → PX4).

### Классы (`pose_subscriber_tf2.hpp`)

#### VpePublisher

Реализует публикацию визуальной одометрии в PX4 через `px4_ros2::LocalPositionMeasurementInterface`.

- **Базовый класс:** `px4_ros2::LocalPositionMeasurementInterface`
- **Поведение:** подписка на позу с ковариацией и на `VehicleLocalPosition`, расчёт offset (world ↔ aruco_map), преобразование в NED, отправка в `fmu/in/vehicle_visual_odometry`.

##### Публичные методы

| Метод | Описание |
|-------|----------|
| `VpePublisher(rclcpp::Node & node)` | Конструктор: объявление параметров, подписки, TF, таймер. |
| `void start()` | Запуск периодической публикации с частотой `publish_rate_hz`. |
| `void stop()` | Остановка таймера. |

##### Внутренние (private)

- `getTimestamp()` — время в микросекундах (единый источник для синхронизации с offboard/PX4).
- `timeFromMicros(uint64_t us)` — преобразование микросекунд в `rclcpp::Time`.
- `declareParameters()` / `setupSubscriptions()` — инициализация параметров и топиков.
- `poseCovCallback`, `vehicleLocalPositionCallback`, `timerCallback` — обработка входящих данных и цикл публикации.
- `resetOffset()`, `computeOffset()`, `applyOffset()` — управление offset (world ↔ aruco_map).
- `sendFallbackData()` — отправка fallback-измерения при отсутствии визуальной позы.
- Конвертеры ENU ↔ NED и утилиты TF (`poseMsgToTf`, `tfToPoseMsg`).

##### Параметры (соответствуют `declare_parameter` в .cpp)

Строковые (топики и frame_id):

- `pose_cov_topic`, `vehicle_local_position_topic`
- `camera_frame_id`, `base_link_frame_id`, `world_frame_id`, `aruco_map_frame_id`, `offset_frame_id`

Логика и таймауты:

- `allow_offset_timeout_reset` (bool)
- `allow_send_vio` (bool)
- `publish_rate_hz` (double)
- `offset_timeout` (double)
- `data_timeout_sec` (double)
- `fallback_variance` (float)

Константы по умолчанию в коде:

- `OFFSET_TIMEOUT_DEFAULT = 3.0`
- `DATA_TIMEOUT_DEFAULT = 0.5`
- `FALLBACK_VARIANCE_DEFAULT = 0.5f`

##### Состояние (offset и reset_counter)

- `offset_` — преобразование world → aruco_map.
- `offset_initialized_`, `reset_flag_`, `last_offset_reset_time_`.
- `reset_counter_` — счётчик сбросов, передаётся в PX4 в сообщении визуальной одометрии для EKF2.

#### VpePublisherNode

Обёртка `rclcpp::Node`, которая создаёт `VpePublisher`, регистрирует его в навигационном интерфейсе PX4 и запускает публикацию.

- **Публичные методы:** конструктор/деструктор.
- **Внутри:** `interface_` (`VpePublisher`), вызовы `doRegister()`, `start()`, таймер для поддержания работы узла.

### Соответствие параметров launch и узла

Параметры, передаваемые через launch (в т.ч. `launch_arguments`), должны совпадать с именами из таблиц раздела [Передаваемые параметры](#передаваемые-параметры) выше. Все перечисленные там параметры объявлены в `VpePublisher::declareParameters()` и используются в логике узла.
