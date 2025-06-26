# pioneer_description
Модель пионер в .xacro формате, с подключенными effort и state контроллерами

### Запустите launch файл:
```bash
roslaunch pioneer_description spawn_xacro_with_control.launch
``` 
В launch файле запускается симуляция в Gazebo, контроллеры, а также Rviz с конфигурацией
## Управление 
### Добавить усилие вперед:
```bash
rostopic pub /right_wheel_effort_controller/command std_msgs/Float64 "data: 0.1"
```
### Убрать усилие:
```bash
rostopic pub /right_wheel_effort_controller/command std_msgs/Float64 "data: 0.0"
```
### Добавить усилие назад:
```bash
rostopic pub /right_wheel_effort_controller/command std_msgs/Float64 "data: -0.1"
```
Важно для остановки.

## Как проехать один метр?
Один метр- одна клетка Gazebo, соответсвенно задать усилие 0.1, как робот пройдет половину клетки, задать усилие -0.1, затем как остановиться - задать усилие 0.0

## Как повернуться на заданный угол
### Задать угол:
```bash
rostopic pub /front_yaw_position_controller/command std_msgs/Float64 "data: 0.20"
```
Дальше задать усилие 0.1, как робот повернется на половину от угла (да, на глаз), задать усилие -0.1, затем как остановиться - задать усилие 0.0

