# Описание проекта

Рассматривается система автономной посадки беспилотного летательного аппарата (БПЛА) на стационарную платформу, разработанной Мигелем Сааведрой-Руисом (MikeS96). Данный проект представляет собой набор ROS-пакетов (Robot Operating System), предназначенных для моделирования и реализации автономной посадки БПЛА в симуляционной среде Gazebo с бортовым компьютером.

В ходе курсовой работы будет проведен запуск и анализ работы системы, её настройка в симуляционной среде Gazebo, а также будет проведен эксперимент с разными детекторами объектов.

# Описание пакетов


Репозиторий [autonomous_landing_uav](https://github.com/MikeS96/autonomous_landing_uav) содержит три основных ROS-пакета, реализующих систему автономной посадки беспилотного летательного аппарата (БПЛА) на стационарную платформу. Каждый пакет выполняет специализированные функции, обеспечивая полный цикл автономной посадки: от управления БПЛА до обнаружения и отслеживания посадочной платформы. Ниже приведено описание каждого пакета с указанием их назначения и функциональности.

### 1. mavros_off_board
Пакет отвечает за базовое управление БПЛА и интеграцию с симуляционной средой Gazebo, а также с прошивкой PX4. Он включает файлы запуска (launch files), файлы описания модели (URDF, XACRO, SDF) и скрипты для управления полётом БПЛА в режиме offboard. Он обеспечивает запуск симуляции, визуализацию в RViz и взаимодействие с автопилотом PX4 через MAVROS, что позволяет контролировать БПЛА с помощью ROS.

**Функциональность**:

- Запускает симуляцию в Gazebo с использованием файлов мира (в данной работе это `grass_pad.world`). 

- Настройка виртуальной модели квадрокоптера DJI F450 с использованием URDF/XACRO для визуализации и SDF для физической симуляции.

- Поддержка четырёх файлов запуска:
    - `mavros_posix_sitl.launch`: запуск MAVROS, PX4 SITL и Gazebo для управления БПЛА в формате SDF.
    - `mavros_rviz.launch`: визуализация модели в Gazebo и RViz (URDF-формат).
    - `urdf_launcher.launch`: запуск MAVROS, PX4 SITL, RViz и Gazebo с моделью в XACRO-формате.
    - `posix_sitl.launch`: базовый запуск симуляции с поддержкой взаимодействия через MAVROS.
- Скрипты для базового управления, такие как взлёт и перемещение БПЛА.

**Ссылка**: [mavros_off_board](https://github.com/MikeS96/autonomous_landing_uav/tree/master/mavros_off_board)

### 2. object_detector
Пакет  реализует алгоритмы компьютерного зрения для обнаружения и отслеживания посадочной платформы. Он использует методы извлечения признаков (SURF/SIFT) из библиотеки OpenCV и фильтр Калмана для точного отслеживания цели. Этот пакет обрабатывает изображения с камеры БПЛА, определяет положение посадочной платформы и передаёт данные в пакет `drone_controller`.

**Функциональность**:
- Обнаружение посадочной платформы с помощью детекторов SURF/SIFT (требуется OpenCV 3.4 для поддержки этих алгоритмов).

- Отслеживание платформы с использованием фильтра Калмана для сглаживания оценок положения.

- Поддержка визуализации через графический интерфейс (GUI) для настройки параметров детектора.

- Интеграция с пакетом `find_object_2d` для обработки изображений и обнаружения объектов.

- Запуск через `simu.launch` для тестирования в симуляционной среде.

**Ссылка**: [object_detector](https://github.com/MikeS96/autonomous_landing_uav/tree/master/object_detector)

### 3. drone_controller
Пакет управляет процессом посадки БПЛА на основе данных, полученных от `object_detector`. Он использует пропорциональные и PID-контроллеры для регулирования скорости по осям X и Y, угла рыскания (yaw) и высоты (Z) квадрокоптера, обеспечивая точное позиционирование над посадочной платформой.

**Функциональность**:
- Реализация пропорциональных и PID-контроллеров для управления движением БПЛА.

- Обработка оценок положения платформы от `object_detector` для корректировки траектории.

- Управление скоростью и высотой БПЛА для плавного снижения и центрирования над платформой.

- Интеграция с MAVROS для отправки команд управления на автопилот PX4.

**Ссылка**: [drone_controller](https://github.com/MikeS96/autonomous_landing_uav/tree/master/drone_controller)


