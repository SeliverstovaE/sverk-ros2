# aruco_det_loc - aruco detect and localize

Пакет ROS 2 с нодами для детекции ArUco‑маркеров и локализации дрона (base_link) в мировой системе координат по заранее известной карте маркеров.

---

## Сборка

```bash
colcon build --packages-select aruco_det_loc
source install/setup.bash
```

---

## Состав пакета

### aruco_detect_node

Нода детекции ArUco‑маркеров на изображении камеры.

**Назначение:**
- Детекция ArUco на входном изображении
- Публикация найденных маркеров (ID, углы в пикселях, размер)
- Опциональная оценка pose маркера относительно камеры
- RViz‑визуализация

**Подписки:**
- `image_topic` — `sensor_msgs/Image`
- `camera_info_topic` — `sensor_msgs/CameraInfo`
- `map_markers_topic` — `aruco_det_loc/msg/MarkerArray`

**Публикации:**
- `markers` — `aruco_det_loc/msg/MarkerArray`
- `visualization` — `visualization_msgs/MarkerArray`

---

### aruco_loc_node

Нода локализации камеры и основания дрона по ArUco‑маркерам.

**Назначение:**
- Оценка pose камеры в маркерной карте
- Пересчёт pose base_link с учётом TF камеры
- SolvePnP

---

## Пример запуска

### Публикация камеры из Gazebo

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image

ros2 run ros_gz_bridge parameter_bridge \
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

### Статическая трансформация base_link → camera (пример для top-down камеры)

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 \
  -0.707 -0.7071 0.0 0.0 \
  base_link main_camera_optical
```

### Запуск локализации (known_vertical_frame используется если ориентация tf base_link берется из измерений с IMU)

```bash
ros2 run aruco_det_loc aruco_loc_node --ros-args \
  -p markers_topic:=/markers \
  -p map_markers_topic:=/map_markers \
  -p camera_info_topic:=/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info \
  -p world_frame_id:=world \
  -p camera_frame_id:=main_camera_optical \
  -p base_link_frame_id:=base_link \
  -p auto_flip:=true \
  -p known_vertical_frame:=base_link
```

```bash
ros2 run aruco_det_loc aruco_detect_node --ros-args \
-p image_topic: =/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image \
-p camera_info_topic:=/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info \
-p use_map_markers:=true
-p map_markers_topic:=/map_markers \
-p dictionary_id:=3
```

### Включение/выключение изображений для дебага
Обратитесь к aruco_detect следующей командой, чтобы включить публикацию debug изображений:
```
rosservice call /toggle_debug std_srvs/srv/SetBool "{data: true}" 
```
Обратите внимание, после выключения публикации топики debug изображений все еще будут в списке "ros2 topic list", но они больше не будут обновляться.

### Просмотр визуализации

Нода aruco_detect_node, в случае если параметр /publish_visualization = true, публикует сообщения для визаулиации в топик 
/visualization. Для просмотра запустите rviz2, нажмите кнопку 'add' и на вкладке "By topic" выбрите соответствующий топик, установите параметр "Fixed Frame" в значение "main_camera_optical". 

Также, ноды aruco_detect_node и aruco_loc_node могут публиковать изображения с камеры для дебага, на которых нарисованы оси координат отдельных маркеров и ось начала координат всей маркерной карты соответственно. Для этого ноды должны быть запущены с параметрами 
/publish_debug_image (название одинаково для обеих нод) в значении true. Увидеть изображения для дебага можно выбрав соответствующие топики в rviz2.