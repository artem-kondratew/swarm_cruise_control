# trajectory_drawer

Простенький matplotlib GUI для рисования траектории, по которой потом
поедет pacemaker (и за ним followers).

## Запуск

```bash
# из корня workspace, после colcon build + source install/setup.bash
ros2 run trajectory_drawer draw -o my_trajectory.yaml
```

Доп. флаги:

```bash
ros2 run trajectory_drawer draw \
    -o my_trajectory.yaml \
    --load existing.yaml \
    --xlim -10 10 --ylim -10 10 \
    --resample-spacing 0.5
```

## Управление в окне

| Клавиша / клик | Действие |
|---|---|
| Левый клик по пустому | Добавить waypoint в позиции курсора |
| Левый клик + drag по точке | Двигать существующую точку |
| Правый клик по точке | Удалить точку |
| `u` | Undo последнего добавления |
| `c` | Очистить все |
| `r` | Toggle uniform resample preview |
| `l` | Toggle close-loop (последний → первый) |
| `b` | Toggle cubic spline сглаживание |
| `s` | Сохранить в `--output` |
| `q` | Выйти (предупредит если есть несохранённые изменения) |

Pick radius для drag/delete — 10 пикселей вокруг точки.

## Формат выходного yaml

Совместим с `swarm_controller/pacemaker_controller`, режим
`trajectory: lanelet`:

```yaml
/**:
  ros__parameters:
    waypoints_x: [0.0, 1.0, 2.0, ...]
    waypoints_y: [0.0, 0.0, 0.5, ...]
```

## Как использовать сохранённую траекторию

Два варианта:

**Вариант A — слить в основной yaml:**
скопировать `waypoints_x` / `waypoints_y` в
`config/params_pacemaker.yaml` и поставить `trajectory: lanelet`.

**Вариант B — overrride через `--params-file`:**
передать сохранённый yaml как ДОПОЛНИТЕЛЬНЫЙ файл при запуске
pacemaker'а (последний переопределяет). Тогда основной
`params_pacemaker.yaml` не трогается.

## Сплайн-режим

`b` включает интерполяцию точек **параметрическим cubic spline** по
кумулятивной длине дуги (`scipy.interpolate.CubicSpline`). Это
позволяет:
- Кликнуть несколько control-точек и получить плавную кривую без
  ручного добавления промежуточных waypoints.
- Двигать любую control-точку — кривая мгновенно пересчитывается.
- Toggle `l` (close-loop) одновременно со сплайном — получишь
  гладкую периодическую кривую (`bc_type='periodic'`).

При сохранении (`s`) с активным сплайном в yaml пишутся **densely
sampled точки сплайна** (количество — `--spline-samples`, default 200).
Если ещё включён resample (`r`), эти точки прореживаются на
`--resample-spacing` (default 0.5 m). Это удобный pipeline:
несколько контрольных точек → сплайн → uniform spacing для
pure-pursuit pacemaker'а.

## Замечания

- Координаты — `map`-frame (как `init_x`/`init_y` в
  `params_simulator.yaml`).
- Pacemaker использует pure-pursuit с `lookahead_dist` (default 1.0 m).
  Для гладкого следования waypoints должны быть на расстоянии ≤ ~1 м.
  Жми `r` и сохрани в resampled-режиме, если кликов мало.
- Чтобы замкнуть круговую траекторию, жми `l` перед сохранением — `s`
  в этом режиме сохранит с замыкающим точкой `points[0]` в конце.
