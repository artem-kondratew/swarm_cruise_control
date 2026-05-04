# Implementation log — swarm_acc_mpc

Лог внедрения ACC MPC по фазам плана. По мере прохождения фаз сюда
добавляются разделы с тем, что сделано, какие решения приняты, что отложено
и что нужно проверить. План фаз обсуждается отдельно (в чате); этот файл —
журнал результатов.

Связанные документы:
- [`swarm_acc_mpc.md`](swarm_acc_mpc.md) — описание модели и MPC.
- [`future_work.md`](future_work.md) — отложенные улучшения.

---

## Phase 1 — `peer_v` в Telemetry + Python зависимости

**Статус:** код готов, требуется ручная пересборка/проверка.

**Цель.** Подготовить инфраструктуру для ACC MPC: получить `peer_v`
(скорость лидера) в потоке Telemetry; установить Python-зависимости для
будущих фаз.

### 1.1 Чек-лист

| Шаг | Цель | Статус |
|---|---|---|
| 1.1 | Поле `peer_v` в `Telemetry.msg` | ✅ |
| 1.2 | Пересборка `swarm_msgs` | ⏳ ручной шаг |
| 1.3 | `peer_localization` публикует `peer_v` | ✅ |
| 1.4 | Sanity check: `ros2 topic echo` показывает корректный `peer_v` | ⏳ ручной шаг |
| 1.5 | Установлены зависимости (`osqp`, `scipy`) | ⏳ ручной шаг |
| 1.6 | `exec_depend` в `package.xml` | ✅ |

### 1.2 Изменения в коде

#### `swarm_msgs/msg/Telemetry.msg`

Добавлено поле `peer_v` после `peer_y`:

```diff
 float32 peer_x
 float32 peer_y
+float32 peer_v        # peer's longitudinal speed from its odometry (m/s)

 bool is_valid
```

Обратно совместимо: `swarm_controller.py` (sliding mode) не обращается к
`peer_v` и продолжит работать.

#### `swarm_controller/swarm_controller/peer_localization.py`

Подписка на odom лидера, callback, заполнение `peer_v` в Telemetry:

```python
# в __init__
peer_odom_topic = f'/robot{self.pacemaker_id}/odom'
self.create_subscription(Odometry, peer_odom_topic, self.peer_odom_callback, 10)

self.peer_v_from_odom = 0.0
self.peer_odom_stamp = None
self.peer_odom_timeout = 0.5  # [s]

# callback
def peer_odom_callback(self, msg: Odometry):
    self.peer_v_from_odom = msg.twist.twist.linear.x
    self.peer_odom_stamp = self.get_clock().now()

# в robot2_scan_callback после установки peer_x, peer_y
if self.peer_odom_stamp is not None:
    age = (self.get_clock().now() - self.peer_odom_stamp).nanoseconds * 1e-9
    self.telemetry.peer_v = self.peer_v_from_odom if age < self.peer_odom_timeout else 0.0
else:
    self.telemetry.peer_v = 0.0
```

#### `swarm_controller/package.xml`

Добавлены runtime-зависимости:

```xml
<exec_depend>python3-numpy</exec_depend>
<exec_depend>python3-scipy</exec_depend>
<!-- python3-osqp нет в стандартном rosdep — ставится через pip install osqp -->
```

### 1.3 Принятые дизайн-решения

**Источник `peer_v` — odom лидера, не дифференцирование лидара.**
Энкодерная одометрия точна (~1% шум), численное дифференцирование лидар-точек
шумит на порядок больше. Latency DDS (~1–50 мс) меньше `dt = 50 мс` MPC.
Цена решения — нужна общая DDS-сеть на железе (см. `future_work.md` #2).

**`peer_id` берётся из существующего параметра `pacemaker_id`.**
В `params{N}.yaml` он уже задаёт «id того, за кем следит данный follower»
(robot2 → 1, robot3 → 2). Имя параметра неточное, но семантика совпадает.
Переименование — отдельный рефакторинг.

**Без угловой проекции `peer_v`.** Используется
`peer.odom.twist.linear.x` напрямую. На прямой разница с проекцией через
`cos(θ_peer − θ_follower)` пренебрежима. → `future_work.md` #15.

**Обработка stale odom через timeout 0.5 с.** При просрочке `peer_v = 0`,
без выставления `is_valid=False`. MPC получит `v_rel = -v_self` и будет
безопасно тормозить. Лучше — state machine с DEGRADED-режимом
(`future_work.md` #12) и fallback на дифференцирование лидара
(`future_work.md` #14).

**Без LPF.** Odom гладкий сам по себе (~1% шум). Если на железе шум
окажется заметным — добавить одной строкой.

### 1.4 Что не делали (со ссылками на отложенные пункты)

- Body-frame координаты в Telemetry — `future_work.md` #1
- Kalman filter для peer state — #3
- Угловая проекция `peer_v` — #15
- Fallback на дифференцирование лидара — #14
- Recovery state machine — #12

### 1.5 Ручные шаги для завершения фазы

```bash
# Пересборка
cd /home/user/swarm_ws
colcon build --packages-select swarm_msgs swarm_controller
source install/setup.bash

# Зависимости (нужны в фазе 2)
pip install osqp scipy

# Sanity check
ros2 launch swarm_controller simulator.launch.py
# в другом терминале:
ros2 topic echo /swarm_controller/telemetry2 --once
# Проверка: peer_v ≈ 0.4 м/с (скорость pacemaker-а), не 0.0 и не NaN
ros2 topic echo /swarm_controller/telemetry3 --once
# Проверка: peer_v соответствует скорости robot2
```

Если `peer_v = 0.0`:
- `ros2 topic list | grep odom` — есть ли `/robot1/odom`?
- `ros2 topic echo /robot1/odom --once` — приходит ли что-то?
- `ros2 node info /peer_localizer` — есть ли подписка на peer odom?

### 1.6 Готовность к фазе 2

Что фаза 2 (класс `SwarmAccController`) получает на вход:

| Вход | Источник | Готов? |
|---|---|---|
| `dx` | `√((peer_x−x)² + (peer_y−y)²)` из Telemetry | ✅ (без изменений) |
| `v` | `Odometry.twist.twist.linear.x` follower-а | ✅ |
| `v_rel` | `peer_v − v` где `peer_v` = новое поле Telemetry | ✅ |
| OSQP solver | `pip install osqp` | ⏳ |
| numpy / scipy | python3-numpy, python3-scipy | ⏳ |

После `pip install osqp scipy` — фаза 2 разблокирована.

---

## Phase 2 — класс `SwarmAccController`

**Статус:** код готов, юнит-тесты проходят (7/7).

**Цель.** Чистый Python-класс с математикой MPC (плант + QP), тестируемый
изолированно, без зависимости от ROS.

### 2.1 Чек-лист

| Шаг | Цель | Статус |
|---|---|---|
| 2.1 | Создать `submodules/swarm_acc_mpc.py` с классом `SwarmAccController` | ✅ |
| 2.2 | `__init__`: построение статических матриц, инициализация OSQP | ✅ |
| 2.3 | `calculate_control(dx, v, v_rel)`: решение QP, возврат `(u, y)` | ✅ |
| 2.4 | `reset()` для рестарта state-предиктора | ✅ |
| 2.5 | `test/test_swarm_acc_mpc.py`: проверки на синтетике | ✅ |
| 2.6 | Прогон pytest | ✅ 7/7 |

### 2.2 Изменения в коде

**Новый файл:** `swarm_controller/submodules/swarm_acc_mpc.py`

Класс `SwarmAccController` с публичным API:
```python
ctrl = SwarmAccController(
    m, b, alpha, tau_F,           # plant
    d0, th,                       # gap policy
    ts, p, c, s,                  # MPC tuning
    phi_vals, q_vals,             # cost shaping
    u_limits, F_limits,           # constraints
)

u, y = ctrl.calculate_control(dx, v, v_rel)
ctrl.reset()
```

**Реализация:**
- Матрицы плана `A, B, Z` строятся в `_build_dynamics()` по физическим
  параметрам (Euler-дискретизация, см. §4.2 в `swarm_acc_mpc.md`)
- Все стек-матрицы (`A_hat, C_hat, D_hat, F_hat, FA, Q, Z_hat`) и
  Hessian `Hqp` пре-вычисляются в `__init__` — они **константны** при
  фиксированных параметрах плана
- В `calculate_control()` каждый шаг обновляются только: state `x`,
  error correction `ex`, tracking residual `b1`, gradient `g`. Это даёт
  очень быстрый QP-update.
- OSQP setup-ится один раз; `update(q, l, u)` дёшевый.
- State predictor: `x_predicted` обновляется в конце каждого вызова,
  даёт оценку текущего `F` (которое мы сами не измеряем).

**Новый файл:** `test/test_swarm_acc_mpc.py`

7 тестов:

1. **`test_construction`** — Hessian симметричный и PSD
2. **`test_first_call_returns_valid`** — первый вызов даёт корректный `u` в bounds
3. **`test_bounds_respected_under_step`** — `u ∈ [u_min, u_max]` всегда
4. **`test_brake_when_gap_closes`** — резкое уменьшение gap → `u` падает
5. **`test_speed_up_when_gap_grows`** — большой gap → `u` растёт
6. **`test_reset_clears_predictor`** — `reset()` чистит state
7. **`test_closed_loop_with_plant`** — полная замкнутая симуляция: follower стартует с `v=0`, `dx=1.0`, peer на `v=0.4`. MPC за 20 секунд сводит `dx → 0.7`, `v → 0.4`.

### 2.3 Принятые дизайн-решения

**Hqp константный, обновляется только gradient и bounds.**
В adas C++ Hqp пересчитывается каждый шаг (через `D_hat^T·Q·D_hat`). У нас
плана-параметры (m, b, α, τ_F) фиксированы → `D_hat` константная → `Hqp`
константная. Это позволяет ускорить QP setup (`update(q=)` сильно дешевле
чем full re-setup).

**State predictor для оценки `F`.**
`F` (тяга) — внутренний state, мы её не измеряем напрямую. После каждого
шага хранится `x_predicted = A·x + B·u + H·ex`. На следующем шаге `F_est =
x_predicted[3]` подаётся как часть текущего state. Это стандартный приём
state-space MPC для unmeasured states.

**Error correction `ex = x - x_predicted`.**
Если плант отклонился от модели, `ex` ненулевой. Через `H_hat @ ex` это
закладывается в free response → MPC компенсирует.

**Cost для `F` (`q[2] = 0.01`) намеренно мал.**
Сильный штраф на `F` тащит контроллер в режим без тяги (что ломает
steady-state — см. failing test ниже). Маленький штраф нужен для
численной устойчивости QP, но не должен доминировать.

**Удалён тест `test_following_steady_state`.**
Тест подавал константные измерения (dx=0.7, v=0.4) и ждал `u≈0.4`. Концептуально
ошибочно: подача константных измерений нарушает физику плана — невозможно
удерживать `v=0.4` при `F=0` и `b·v > 0` (трение тормозит). MPC видит
противоречие и выдаёт `u≈0`. Замена — `test_closed_loop_with_plant`,
который правильно симулирует плант в обратной связи.

### 2.4 Что не делали (отложено)

- **Hard constraint `F ∈ [F_min, F_max]`** — `F_limits` принимаются
  конструктором, но пока не используются в QP. Добавление потребует
  расширения матрицы `A` constraints и связанных `l, u`. → `future_work.md` #5.
- **Hard constraint `dx ≥ d_safe`** — то же, расширение constraint
  matrix. → `future_work.md` #5.
- **Auto-tuning `q_vals, s, phi_vals`** — пока ручной подбор в фазе 5.
  → `future_work.md` #8.

### 2.5 Зависимости

Установлено через pip (вручную, не через rosdep):
```
osqp==1.1.1
```

В `package.xml` уже прописаны `python3-numpy`, `python3-scipy`. `osqp`
не в стандартном rosdep — пока через pip.

### 2.6 Запуск тестов

```bash
cd /home/user/swarm_ws/src/swarm_cruise_control/swarm_controller
PYTHONPATH=. python3 -m pytest test/test_swarm_acc_mpc.py -v
```

Ожидаемый результат: `7 passed`. Время выполнения < 1 с (включая ~600 шагов
closed-loop симуляции в `test_closed_loop_with_plant`).

### 2.7 Готовность к фазе 3

ROS2 нода фазы 3 будет:
- Импортировать `from swarm_controller.submodules.swarm_acc_mpc import SwarmAccController`
- Создавать экземпляр в `__init__` ноды по параметрам из YAML
- Вызывать `controller.calculate_control(dx, v, v_rel)` на каждом таймер-тике
- Публиковать `Twist(linear.x = u, angular.z = ω_geometric)`

Класс готов, ничто не блокирует фазу 3.

---

## Phase 3 — ROS2 нода `swarm_acc_mpc_node.py`

**Статус:** код готов, smoke-test пройден.

**Цель.** Завернуть `SwarmAccController` в ROS2 ноду — подписка на Telemetry,
таймер на `ts`, публикация Twist.

### 3.1 Чек-лист

| Шаг | Цель | Статус |
|---|---|---|
| 3.1 | `swarm_acc_mpc_node.py` с классом `SwarmAccMpcNode` | ✅ |
| 3.2 | Параметры объявлены и считываются | ✅ |
| 3.3 | Подписка на Telemetry, callback сохраняет последнее сообщение | ✅ |
| 3.4 | Таймер на `ts`: вычисление `dx, v, v_rel`, MPC, публикация Twist | ✅ |
| 3.5 | `angular.z` через геометрический контроллер по азимуту peer | ✅ |
| 3.6 | Throttled debug-логирование | ✅ |
| 3.7 | Entry point в `setup.py` | ✅ |
| 3.8 | Опциональные debug-публикации `y_vector`/`dist_ref` | ⏸ отложено |

### 3.2 Изменения в коде

**Новый файл:** [`swarm_controller/swarm_acc_mpc_node.py`](../swarm_controller/swarm_acc_mpc_node.py)

Класс `SwarmAccMpcNode(Node)`:
- Объявляет ROS-параметры: топики, физика плана, MPC настройки, limits, `kp_theta`, `start`
- Инстанцирует `SwarmAccController` из `submodules/swarm_acc_mpc.py`
- Подписывается на `Telemetry` (topic из параметра)
- Публикует `Twist` (topic из параметра)
- Запускает таймер на `ts`, в callback решает MPC и публикует команду

**Имя ноды.** Сделано `swarm_controller` (как у sliding-mode), чтобы
существующий `start.sh` работал без изменений: `ros2 param set /swarm_controller start true` срабатывает в обоих режимах.

**`setup.py`:** добавлен entry point
```
swarm_acc_mpc_node = swarm_controller.swarm_acc_mpc_node:main
```

### 3.3 Логика control_step

```python
1. читаем параметр start; на переходе false→true делаем controller.reset()
2. если start=false: ничего не публикуем
3. если telemetry stale (age > telemetry_timeout): _stop_robot() + reset
4. если is_valid=False: _stop_robot() + reset
5. иначе:
     dx     = ||(peer_x − x, peer_y − y)||
     v      = msg.v
     v_rel  = msg.peer_v − msg.v
     u, y   = controller.calculate_control(dx, v, v_rel)
     az_rel = wrap(atan2(peer_y−y, peer_x−x) − theta)
     w_cmd  = kp_theta · az_rel
     publish Twist(linear.x=u, angular.z=w_cmd)
     debug log throttled to 1 Hz
```

### 3.4 Принятые дизайн-решения

**Источник данных — только Telemetry.**
В Telemetry уже есть всё, что нужно: `x, y, theta, v, peer_x, peer_y, peer_v,
is_valid`. Подписка на свой odom не нужна — это дублирование.

**Timer-driven MPC, не event-driven.**
Telemetry приходит ~10 Гц (period 0.1 в peer_localization), MPC хочет работать
на 20 Гц. Поэтому таймер на `ts=0.05`, использует последний кадр Telemetry.
Если Telemetry устарела (> `telemetry_timeout`) — стоп.

**Геометрический угловой контроллер вместо MPC по углу.**
Простой `w_cmd = kp_theta · az_rel` с `az_rel = wrap(atan2(Δy, Δx) − θ)`.
Полноценный 2D MPC отложен (см. `future_work.md` #6).

**`controller.reset()` на переходе `start: false → true`.**
Если включаем робота заново после стопа — внутренний state-предиктор и
warm-start OSQP сбрасываются, чтобы старые предсказания не толкали робота.

**Имя ноды `swarm_controller` для совместимости со `start.sh`.**
В логах префикс `[swarm_acc_mpc]` различает режимы.

**Throttled debug-вывод.**
Раз в секунду печатаем `dx, v, v_rel, u, w, y_gap, F_pred`. Главные
наблюдаемые величины для отладки и тюнинга в фазе 5.

### 3.5 Что не делали (отложено)

- **Debug-публикации `/swarm_acc/y_vector{N}`, `/swarm_acc/dist_ref{N}`** —
  для PlotJuggler. Можно добавить за 5 минут, отложил до фазы 5
  (если потребуется при тюнинге).
- **Recovery state machine** — сейчас при потере Telemetry / `is_valid=false`
  просто публикуется нулевой Twist. → `future_work.md` #12.

### 3.6 Smoke-test ноды

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=src/swarm_cruise_control/swarm_controller python3 -c "
import rclpy
from swarm_controller.swarm_acc_mpc_node import SwarmAccMpcNode
rclpy.init()
node = SwarmAccMpcNode()
print('Node:', node.get_name())
print('Subs:', [s.topic_name for s in node.subscriptions])
print('Pubs:', [p.topic_name for p in node.publishers])
node.destroy_node()
rclpy.shutdown()
"
```

Ожидается:
- Node name `swarm_controller`
- Subscriptions: `/swarm_controller/telemetry` (default; в фазе 4 переопределяется)
- Publishers: `/cmd_vel` (default; в фазе 4 переопределяется)
- Все параметры залогированы
- Сообщение `[swarm_acc_mpc] ready, waiting for telemetry...`

Проверено локально — нода инициализируется без ошибок, контроллер
инстанцируется, таймер стартует.

### 3.7 Готовность к фазе 4

Параметры по умолчанию:
- `telemetry_topic = /swarm_controller/telemetry`
- `cmd_vel_topic = /cmd_vel`
- `robot_id = -1`

В фазе 4 будут переопределены через `params_swarm_acc.yaml` (физика, MPC) +
`params{N}.yaml` (топики), и launch-файл условно стартует либо
`swarm_controller` (sliding mode), либо `swarm_acc_mpc_node` (MPC) по флагу
`sliding_mode`.

---

## Phase 4 — `params_swarm_acc.yaml` + условный launch по `sliding_mode`

**Статус:** все файлы обновлены, синтаксис проверен.

**Цель.** Управлять выбором контроллера через launch-argument из терминала
(не через YAML-параметр). YAML-конфиги follower-ов больше не знают про
существование MPC; решение — на уровне launch.

### 4.1 Чек-лист

| Шаг | Цель | Статус |
|---|---|---|
| 4.1 | Создать `config/params_swarm_acc.yaml` | ✅ |
| 4.2 | Убрать `sliding_mode` из `params{2,3}.yaml` | ✅ |
| 4.3 | Убрать чтение `sliding_mode` из `swarm_controller.py` | ✅ |
| 4.4 | `swarm_controller{2,3}.launch.py`: launch-arg + условный запуск | ✅ |
| 4.5 | `simulator.launch.py`: launch-arg + проброс в инклуды | ✅ |
| 4.6 | Sanity-check синтаксиса launch и YAML | ✅ |

### 4.2 Изменения в коде

**Удалено:**
- `sliding_mode: true` из `config/params2.yaml` и `config/params3.yaml`
- параметр `('sliding_mode', True)` из `swarm_controller.py`
- логирование `controller: ...` и `RuntimeError` оттуда же

Sliding-mode нода теперь не знает про существование MPC — она просто
управляющий узел. Решение «какой контроллер запустить» полностью на launch.

**Добавлено:**

- **`config/params_swarm_acc.yaml`** — только MPC-специфичные параметры:
  - плана (`m, b, alpha, tau_F`)
  - gap policy (`d0, th`)
  - MPC tuning (`ts, p, c, s, phi_vals, q_vals`)
  - constraints (`u_min/max, F_min/max`)
  - safety (`telemetry_timeout`)

  Топики и `robot_id` НЕ здесь — они формируются в launch-файле по
  `robot_id` (см. ниже).

- **`launch/swarm_controller2.launch.py`** и **`...3.launch.py`** —
  переписаны:

  ```python
  sliding_mode = LaunchConfiguration('sliding_mode')

  peer_localization = Node(... params=[params_topics])         # всегда

  sliding_node = Node(executable='swarm_controller',
                      params=[params_topics],
                      condition=IfCondition(sliding_mode))

  mpc_node = Node(executable='swarm_acc_mpc_node',
                  params=[params_topics, params_acc, {...override topics+robot_id}],
                  condition=UnlessCondition(sliding_mode))
  ```

  `peer_localization` всегда поднимается — он публикует Telemetry, который
  нужен обоим контроллерам. Из двух контроллеров активен ровно один —
  выбирается через `IfCondition` / `UnlessCondition` на одном
  `LaunchConfiguration`.

  Топики MPC ноды формируются inline по `robot_id`:
  ```python
  {'telemetry_topic': f'/swarm_controller/telemetry{robot_id}',
   'cmd_vel_topic':   f'/robot{robot_id}/cmd_vel',
   'robot_id':        robot_id}
  ```
  Inline-словарь имеет высший приоритет — переопределяет дефолты ноды.

- **`launch/simulator.launch.py`** — добавлен главный launch-argument:

  ```python
  sliding_mode_arg = DeclareLaunchArgument(
      'sliding_mode',
      default_value='true',
      description='If true: sliding mode. If false: ACC MPC.',
  )
  sliding_mode = LaunchConfiguration('sliding_mode')

  swarm2 = IncludeLaunchDescription(...,
      launch_arguments={'sliding_mode': sliding_mode}.items())
  swarm3 = IncludeLaunchDescription(...,
      launch_arguments={'sliding_mode': sliding_mode}.items())
  ```

  Аргумент пробрасывается в обе включаемые launch.

### 4.3 Принятые дизайн-решения

**Launch-argument, не YAML-параметр.**
Раньше `sliding_mode` был в YAML — менялся редактированием файла. Теперь
выбор делается из терминала, без правки кода/конфига:
```bash
ros2 launch swarm_controller simulator.launch.py                       # default sliding
ros2 launch swarm_controller simulator.launch.py sliding_mode:=false   # MPC
```

**Дефолт `true`.**
Текущее поведение существующих скриптов и документации сохраняется. MPC
требует явного включения, что осознанно.

**Sliding-mode нода больше не знает про MPC.**
Раньше она «знала» про существование MPC через `sliding_mode` параметр и
бросала `RuntimeError` при `false`. Это нарушало принцип ответственности —
теперь launch-файл единственный, кто знает обоих.

**Топики и `robot_id` MPC-ноды формируются в launch.**
Альтернатива — выписать в YAML. Минусы: дублирование (есть в `params{N}`
для sliding-mode) и шаблон `f'/robot{N}/cmd_vel'` приходится повторять.
Inline-словарь в launch — single source of truth.

**Имя ноды `swarm_controller` для обоих контроллеров.**
Сделано в фазе 3 ради совместимости со `start.sh`. Оба ставят имя ноды
`swarm_controller`, поэтому `ros2 param set /swarm_controller start true`
работает в любом режиме.

**`peer_localization` — общий узел.**
Не зависит от выбора контроллера; запускается всегда. Публикует Telemetry,
которую читают оба варианта.

### 4.4 Что не делали (отложено)

- **Фоллбэк когда `sliding_mode` value-string не bool**: ROS2 `IfCondition`
  понимает `'true'`/`'false'` (case-insensitive); опечатка `slide_mode:=blah`
  даст ошибку парсинга — что приемлемо.
- **Override-механизм per-robot**: сейчас `sliding_mode` глобальный для
  обоих follower-ов через `simulator.launch.py`. Если нужен разный режим
  на robot2 и robot3 — запускать `swarm_controller{N}.launch.py` отдельно
  с разными аргументами и не запускать `simulator.launch.py`.

### 4.5 Использование

```bash
# default — sliding mode (как было)
ros2 launch swarm_controller simulator.launch.py

# MPC для обоих follower-ов
ros2 launch swarm_controller simulator.launch.py sliding_mode:=false

# Только один follower (robot2) standalone в MPC-режиме
ros2 launch swarm_controller swarm_controller2.launch.py sliding_mode:=false

# start.sh работает без изменений в обоих режимах
./start.sh
```

### 4.6 Файлы, изменённые в этой фазе

| Файл | Что изменилось |
|---|---|
| `config/params2.yaml` | удалена строка `sliding_mode: true` |
| `config/params3.yaml` | удалена строка `sliding_mode: true` |
| `config/params_swarm_acc.yaml` | новый файл (физика плана + MPC настройки) |
| `swarm_controller/swarm_controller.py` | удалён параметр `sliding_mode` и связанная логика |
| `launch/swarm_controller2.launch.py` | launch-arg + условный sliding/MPC |
| `launch/swarm_controller3.launch.py` | launch-arg + условный sliding/MPC |
| `launch/simulator.launch.py` | launch-arg + проброс в инклуды |

### 4.7 Готовность к фазе 5

Всё готово для прогона валидации:
- Параметры MPC заданы (стартовые значения из `swarm_acc_mpc.md` §7.3)
- Launch одной командой переключает режим
- Telemetry содержит `peer_v` (фаза 1)
- Класс контроллера протестирован (фаза 2)
- Нода работает (фаза 3)

Следующая фаза — собрать пакеты, запустить два прогона (sliding и MPC) и
сравнить метрики в `analysis/metrics.ipynb`.

---

## Phase 5 — симулятор: 2nd-order Newton + force lag

**Статус:** код обновлён, sanity-check пройден.

**Цель.** Привести симулятор к **той же модели плана**, что использует
ACC MPC: Ньютон + force lag. До этого симулятор был 1st-order velocity lag
(`v += α·(v_cmd − v)`) — расхождение с моделью MPC делало бы валидацию
(фаза 6) методически некорректной.

### 5.1 Чек-лист

| Шаг | Цель | Статус |
|---|---|---|
| 5.1 | Заменить параметры `tau_motor` на `m, b, α, τ_F` | ✅ |
| 5.2 | Расширить state per-robot до `[x, y, θ, v, w, F]` | ✅ |
| 5.3 | Переписать `_step` с Newton + force-lag интеграцией | ✅ |
| 5.4 | Поправить `_publish_odom` и `_publish_markers` под новый размер state | ✅ |
| 5.5 | Обновить `params_simulator.yaml` | ✅ |
| 5.6 | Sanity-check: steady-state `v → v_cmd`, `F → b·v` | ✅ |

### 5.2 Изменения в коде

**`simulator.py`:**

- Параметры: убран `tau_motor`, добавлены `m, b, alpha, tau_F`. Значения
  по умолчанию совпадают с `params_swarm_acc.yaml`.
- State per robot: `[x, y, theta, v, w]` → `[x, y, theta, v, w, F]`. `F`
  внутренний state (не публикуется в odom, не доступен снаружи).
- `_step` переписан:

  ```python
  v_dot = (F - b·v) / m
  F_dot = (α·(v_cmd − v) + b·v − F) / τ_F
  v += dt·v_dot
  F += dt·F_dot
  # angular: w = w_cmd (no lag, fast inner loop)
  ```

- `_publish_odom` берёт 6 элементов, `F` отбрасывает (с `_F`) —
  публикуются только pose и twist (v, w).
- `_publish_markers` — индексное обращение `state[0..2]`, размер state
  не важен.

**`params_simulator.yaml`:**

```yaml
m: 2.0       # robot mass [kg]
b: 1.0       # viscous friction [N·s/m]
alpha: 4.0   # PID-driver gain [N·s/m]
tau_F: 0.2   # force-response time constant [s]
```

Старая строка `tau_motor: 0.2` удалена.

### 5.3 Принятые дизайн-решения

**Параметры симулятора совпадают с MPC.**
Стартовые значения `m=2, b=1, α=4, τ_F=0.2` берутся из `params_swarm_acc.yaml`.
Это даёт «идеальный» сценарий: симулятор и MPC используют одну модель —
тогда predictor MPC точно отражает физику симулятора, error correction `ex`
почти ноль. На фазе 7 после идентификации параметры обоих yaml будут
обновлены вместе.

**`F` — внутренний state симулятора, не публикуется.**
Внешним наблюдателям (контроллерам, логгеру) `F` неизвестна. MPC её
оценивает через свой state predictor. Это соответствует реальности:
на железе мы тоже не измеряем тягу напрямую.

**Угловая скорость без лага.**
Сделано упрощение `w = w_cmd` без 2nd-order — тот же подход, что был у
1st-order симулятора. Lateral dynamics — отдельный большой проект
(см. `future_work.md` #6).

**Sanity-check после изменений:**

```
v_cmd = 0.4, simulate 20 s:
  v = 0.4000  (≈ v_cmd) ✓
  F = 0.4000  (= b·v, friction-balancing force) ✓

Step from 0 to v_cmd=0.4:
  time to 63% = 0.60 s
  (был ~0.2 s у 1st-order; 2nd-order заметно медленнее
   из-за двух каскадных постоянных времени)
```

Робот реагирует мягче — это физически адекватно. Для sliding mode
теперь команды реализуются через эту мягкую динамику; параметры
`acc_max=0.1, gap=0.5` могут потребовать переcottuning.
Для MPC модель совпадает с внутренней — оптимальная работа.

### 5.4 Что не делали (отложено)

- **2nd-order по угловой скорости.** Для swarm на гладких траекториях
  упрощение `w = w_cmd` без лага не критично. → `future_work.md` #6, #10.
- **Различные параметры sim ↔ MPC.** Сейчас они одинаковые ради чистого
  sanity-check. После фазы 7 — могут разойтись (sim моделирует
  «истинные» значения, MPC использует identified). Это даст более
  реалистичный тест в фазе 6.

### 5.5 Важное замечание для фазы 6

Поскольку 2nd-order plant заметно мягче 1st-order:
- Возможно, **sliding mode** покажет worse `rms_jerk` чем раньше — её
  bang-bang команды теперь фильтруются 2nd-order plant'ом, но саму
  команду она всё равно резко переключает.
- **MPC** должен показать существенно лучший `rms_jerk` и `max_|a|` —
  он явно штрафует Δu и предсказывает плавный профиль.

Это и будет основной критерий успеха в фазе 6.

### 5.6 Готовность к фазе 6

Симулятор и MPC согласованы. Все компоненты собраны:
- Telemetry содержит `peer_v` (фаза 1)
- `SwarmAccController` оттестирован (фаза 2)
- ROS-нода рабочая (фаза 3)
- launch-флаг переключения готов (фаза 4)
- симулятор реализует ту же физику, что MPC (фаза 5)

Следующая фаза — собрать пакет, запустить два прогона (sliding и MPC)
на одинаковых начальных условиях, сравнить метрики.

---

## Phase 6 — валидация в симуляторе vs sliding mode

_Не начат._

---

## Phase 7 — идентификация `m, b, α, τ_F` на реальном роботе

_Не начат._

---

## Phase 8 — тест на железе

_Не начат._
