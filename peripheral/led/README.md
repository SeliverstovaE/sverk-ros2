# ROS2 workspace for led_control

Пакеты для управления лентой WS2812 из ROS2 Humble (сервисы set_effect/set_leds, топик /led/state, визуализация событий PX4).

## Пакеты

- **led_interfaces** — сообщения и сервисы (`LEDState`, `LEDStateArray`, `SetLEDEffect`, `SetLEDs`).
- **led_control** — нода с эффектами и сервисами `/led/set_effect`, `/led/set_leds` и топиком `/led/state`.

## Зависимости

- ROS2 Humble
- Python 3.10+
- `pip install rpi5-ws2812` (до сборки или в venv)

## Сборка

```bash
cd ros2
colcon build --packages-select led_interfaces led_control
source install/setup.bash
```

## Запуск

```bash
ros2 launch led_control led.launch.py
```

Параметры в `src/led_control/config/led_params.yaml` (или свой файл через аргумент `config:=`).

Документация: [led_control/README.md](led_control/README.md).
