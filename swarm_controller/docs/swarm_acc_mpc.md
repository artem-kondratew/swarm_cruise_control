# Swarm ACC MPC — динамика и постановка задачи

Документ описывает плант-модель и MPC-контроллер для адаптивного круиз-контроля
(ACC) follower-роботов swarm-проекта. Работает поверх дифференциально-приводного
робота с коллекторными моторами и PID-петлёй на STM32.

В отличие от исходного `adas/acc_mpc` (CARLA, авто), здесь используется явная
ньютоновская продольная динамика и физические параметры, идентифицируемые из
эксперимента на реальном железе.

---

## 1. Обоснование выбора уровня моделирования

Рассматривались три уровня детализации:

| Уровень | State | Параметры | Комментарий |
|---|---|---|---|
| 1 — first-order velocity lag | `v` | `τ` | Простой, но не отражает форму отклика PID |
| **2 — Newton + force lag** | `v, F` | `m, b, α, τ_F` | Принят за основу |
| 3 — полная электромех. | `i_l, i_r, ω_l, ω_r, v, θ̇` | 10+ | Избыточен для нашей физики |

**Почему Уровень 2.** Робот лёгкий (≈ 2 кг), скорости низкие (< 0.5 м/с), пол
гладкий — slip и аэродинамика отсутствуют. Запас по тяге мотора относительно
требуемых сил 10–20×, PWM не насыщается. При этом доминирующая динамика —
закрытая петля «STM32 PID + мотор + инерция робота» — имеет второй порядок и
должна быть в модели MPC, иначе предиктор не отражает форму переходного
процесса.

Уровень 2 даёт явный закон Ньютона `m·v̇ = F − b·v` и позволяет MPC оперировать
физическими ограничениями (`F_max`, `v_max`) — то, чего нет в Уровне 1.
Параметры идентифицируются из стандартных экспериментов: coast-down + step
response.

---

## 2. State и control

```
x = [dx, v, v_rel, F]ᵀ        ∈ ℝ⁴
u =  v_cmd                    ∈ ℝ
```

| Символ | Смысл | Ед. изм. | Источник |
|---|---|---|---|
| `dx` | расстояние до лидера | м | telemetry: `√((peer_x−x)² + (peer_y−y)²)` |
| `v` | продольная скорость робота | м/с | odom: `twist.twist.linear.x` |
| `v_rel` | `v_peer − v` | м/с | новое поле `peer_v` в telemetry или производная `dx` |
| `F` | суммарная тяга двух колёс | Н | оценка из state-предиктора MPC |
| `u` | команда скорости | м/с | публикуется в `cmd_vel.linear.x` |

---

## 3. Непрерывная модель

### 3.1 Кинематика гэпа

```
ḋx = v_rel
```
Тривиально из определения `dx = x_peer − x_self`.

### 3.2 Закон Ньютона по продольной оси

```
m·v̇ = F − b·v
⟹ v̇ = (F − b·v)/m
```

- `F = F_l + F_r` — суммарная тяга обоих колёс
- `b·v` — линеаризованное вязкое трение (трение качения и потери в редукторе)
- `m` — общая масса робота

### 3.3 Скорость лидера (CV-предположение)

В предсказании MPC скорость лидера считается постоянной на горизонте:
```
v̇_peer ≈ 0
v̇_rel = v̇_peer − v̇ = −v̇ = (b·v − F)/m
```

Это локальное допущение для QP. Каждый цикл MPC (50 мс) измеряется актуальная
`v_peer`, поэтому глобально лидер может ускоряться/тормозить — MPC отреагирует
на следующем шаге.

При резких манёврах pacemaker модель расширяется до CA (constant acceleration):
+1 state `a_peer` как measured disturbance.

### 3.4 Динамика тяги — модель внутреннего PID

Внутренний контур (STM32 PID + мотор + back-EMF) моделируется как «генератор
силы» с первым порядком:

```
F_eq = α·(u − v) + b·v
Ḟ = (F_eq − F)/τ_F
```

Раскрывая:
```
Ḟ = (α/τ_F)·u + ((b − α)/τ_F)·v − (1/τ_F)·F
```

Физический смысл:
- `α·(u − v)` — PID наращивает тягу пропорционально ошибке скорости
- `b·v` — компенсация трения через интегральное действие позиционной ошибки
  STM32-PID
- `τ_F` — постоянная времени отклика силы (электромеханика мотора + время
  реакции PID)

**Проверка установившегося режима.** При `Ḟ = 0` ⟹ `F = F_eq`; при `v̇ = 0` ⟹
`F = b·v`. Подставляя одно в другое: `α·(u − v) = 0` ⟹ `v = u`. Точное
отслеживание команды скорости.

---

## 4. Матрицы state-space

### 4.1 Continuous-time

```
ẋ = A_c·x + B_c·u

A_c = ⎡ 0       0          1       0       ⎤
      ⎢ 0     −b/m         0      1/m      ⎥
      ⎢ 0      b/m         0     −1/m      ⎥
      ⎣ 0    (b−α)/τ_F     0    −1/τ_F     ⎦

B_c = [ 0,  0,  0,  α/τ_F ]ᵀ
```

### 4.2 Discrete-time (Euler, шаг `ts`)

```
A_d = ⎡ 1     0                ts     0           ⎤
      ⎢ 0   1 − ts·b/m          0    ts/m         ⎥
      ⎢ 0    ts·b/m             1   −ts/m         ⎥
      ⎣ 0   ts·(b−α)/τ_F        0    1 − ts/τ_F   ⎦

B_d = [ 0,  0,  0,  ts·α/τ_F ]ᵀ
```

При желании первая строка уточняется до второго порядка по аналитике
интегрирования `v_rel(t)`:
```
dx_{k+1} = dx_k + ts·v_rel_k − ½·ts²·v̇_k
        = dx_k + ts·v_rel_k − ½·ts²·(F_k − b·v_k)/m
```

---

## 5. Output и cost

### 5.1 Output — что трекаем и штрафуем

Стандартная headway-time policy (как в `adas/src/acc_node.cpp:189`):
```
y_gap = dx − d0 − th·v
```

- `d0` — минимальный безопасный гэп при `v = 0`
- `th` — time headway: на 0.4 м/с с `th = 1.0` референс гэпа = `d0 + 0.4`

Полный output для регуляризации:
```
y = [y_gap, v_rel, F]ᵀ

C = ⎡ 1  −th   0   0 ⎤
    ⎢ 0   0    1   0 ⎥
    ⎣ 0   0    0   1 ⎦

Z = [d0, 0, 0]ᵀ        (reference)
```

### 5.2 Cost

```
J = Σ_{i=1}^{p} (y_{k+i} − Z)ᵀ·Q·(y_{k+i} − Z)         ← tracking
  + Σ_{i=0}^{c−1} Δu_{k+i}ᵀ·S·Δu_{k+i}                  ← move suppression
```

- `Q = diag(q_gap, q_vrel, q_F)` — веса по компонентам выхода
- `S` — вес на изменение управления (jerk-like штраф)
- `p` — prediction horizon, `c ≤ p` — control horizon

Дополнительная экспоненциальная reference shaping (как в `adas`):
```
y_ref(k+i) = Φ^i · y(k)
Φ = diag(φ_gap, φ_vrel, φ_F),  0 ≤ φ < 1
```
Чем дальше в горизонт, тем мягче трекинг — сглаживает переходный процесс.

---

## 6. Constraints

| Constraint | Тип | Реализация |
|---|---|---|
| `0 ≤ u ≤ v_max` | bound на input | напрямую в QP |
| `\|Δu\| ≤ j_max·ts` | rate constraint | через S или явно |
| `\|F\| ≤ F_max` | bound на state | линейный constraint |
| `0 ≤ v ≤ v_max` | bound на state | линейный constraint |
| `dx ≥ d_safe` | safety | hard или soft (slack) |

Bound на `F` — то, чего нет в Уровне 1: ограничение на физический крутящий
момент мотора задаётся явно, а не подбирается через `u_limits`.

---

## 7. Параметры и идентификация

| Параметр | Смысл | Метод | Сложность |
|---|---|---|---|
| `m` | масса робота, кг | весы | минута |
| `b` | вязкое трение, Н·с/м | coast-down | 5 экспериментов |
| `α` | gain force-генератора, Н·с/м | step-response, аналитика | один эксп. |
| `τ_F` | постоянная отклика тяги, с | step-response, фит транзиента | тот же эксп. |
| `d0` | мин. гэп при `v=0`, м | задаётся проектом | — |
| `th` | time headway, с | задаётся проектом | — |
| `F_max` | физический предел тяги, Н | datasheet мотора + тяга колеса | — |

### 7.1 Coast-down для `b`

1. Разогнать робота до `v_0 ≈ 0.4` м/с
2. Отключить питание моторов (или `cmd_vel = 0` с выключенным PID)
3. Записать `v(t)` из одометрии или энкодеров
4. Фитнуть экспоненту `v(t) = v_0·exp(−b·t/m)` (`m` уже измерена)

### 7.2 Step response для `α` и `τ_F`

1. Робот в покое, `v = 0`
2. Скачок `v_cmd = 0 → 0.3` м/с в момент `t = 0`
3. Записать `v(t)` (а также `cmd_vel_back_l/r` со STM32 — `ω` колёс,
   из них восстанавливается фактическая `v`)
4. Численно дифференцировать `v(t)` для оценки `F = m·v̇ + b·v`
5. Фитнуть первый порядок `F(t) = F_∞·(1 − exp(−t/τ_F))`
6. `α = F_∞ / (v_cmd − v_∞)` где `v_∞` — установившаяся скорость

Альтернативно: пропустить оценку `F` и фитнуть напрямую второй порядок к
`v(t)`, получая `ω_n, ζ`, затем пересчитать в `α, τ_F`:
```
ω_n² = α / (m·τ_F)
2ζω_n = 1/τ_F + b/m
```

### 7.3 Стартовые значения для разработки

До первой реальной идентификации:
```
m     = 2.0   кг
b     = 1.0   Н·с/м
α     = 4.0   Н·с/м
τ_F   = 0.2   с
d0    = 0.3   м
th    = 1.0   с
F_max = 5.0   Н        (≈ 2 мотора по 2.5 Н тяги)
```

Эти значения подобраны так, чтобы повторить наблюдаемое поведение симулятора с
`tau_motor = 0.2`. Финальные значения берутся из `identification.md` (после
экспериментов на железе).

---

## 8. MPC pipeline (per cycle)

```
┌──────────────────────────────────────────────────┐
│  inputs (per 50 ms):                             │
│   • Telemetry: x, y, peer_x, peer_y, peer_v      │
│   • Odom:      v_actual                          │
│                                                  │
│  derived:                                        │
│   • dx     = √((peer_x−x)² + (peer_y−y)²)        │
│   • v_rel  = peer_v − v_actual                   │
│   • F_est  = F_predicted_prev (state-предиктор)  │
│                                                  │
│  build state x_k, build references               │
│  solve QP (OSQP) → u_k                           │
│  publish Twist(linear.x = u_k, angular.z = ...)  │
└──────────────────────────────────────────────────┘
```

Угловая скорость (`angular.z`) сейчас вне продольного MPC — задаётся отдельным
ориентационным контроллером по азимуту peer (как в текущем sliding-mode).
Продольный MPC отвечает только за `linear.x`.

---

## 9. Интеграция в swarm_controller (Python-реализация)

Контроллер реализуется на **Python** в существующем `ament_python` пакете —
без миграции на C++. Производительность достаточная: OSQP с Python-bindings
дёргает ту же C-библиотеку, что и C++ wrapper. QP с ~10 переменных решается
за <1 мс при бюджете цикла 50 мс.

Контроллер выбирается через параметр в YAML:

```yaml
sliding_mode: true     # → Python sliding-mode controller (текущая реализация)
sliding_mode: false    # → Python MPC controller (этот документ)
```

Параметр уже добавлен в `config/params2.yaml`, `config/params3.yaml`. На стороне
`swarm_controller.py`: при `sliding_mode=false` Python-нода завершается с
`RuntimeError`, освобождая namespace для MPC-ноды.

### 9.1 Структура файлов

```
swarm_controller/
├── package.xml                          ← без изменений (ament_python)
├── setup.py                             ← +entry point swarm_acc_mpc_node
├── swarm_controller/
│   ├── swarm_controller.py              ← существует (sliding mode)
│   ├── pacemaker_controller.py          ← существует
│   ├── peer_localization.py             ← модифицируется (peer_v)
│   ├── simulator.py                     ← существует
│   ├── swarm_acc_mpc_node.py            ← новая ROS2 нода
│   └── submodules/
│       ├── swarm.py
│       ├── swarm_logic.py
│       └── swarm_acc_mpc.py             ← класс SwarmAccController (плант + QP)
├── config/
│   ├── params2.yaml                     ← +sliding_mode уже есть
│   ├── params3.yaml                     ← +sliding_mode уже есть
│   └── params_swarm_acc.yaml            ← новый: физика + MPC параметры
├── launch/
│   ├── swarm_controller2.launch.py      ← условный по sliding_mode
│   ├── swarm_controller3.launch.py      ← аналогично
│   └── ... (остальное без изменений)
├── docs/
│   ├── swarm_acc_mpc.md                 ← этот документ
│   └── identification.md                ← coast-down + step-response (план)
└── test/
    └── test_swarm_acc_mpc.py            ← unit-тесты для контроллера
```

### 9.2 Класс `SwarmAccController` (`submodules/swarm_acc_mpc.py`)

```python
class SwarmAccController:
    def __init__(self, m, b, alpha, tau_F,
                 d0, th, ts, p, c, s,
                 phi_vals, q_vals,
                 u_limits, F_limits):
        # сохранение параметров
        # построение статических матриц (H1, FA, Q, F, F_hat)
        # инициализация OSQP solver
        ...

    def calculate_control(self, dx, v, v_rel):
        """Returns (u, y): optimal v_cmd and current output vector."""
        # обновление A, B, Z (зависят от ts, m, b, α, τ_F)
        # обновление state-зависимых частей (A_hat, C_hat, D_hat, F_hat)
        # формирование Hessian Hqp и градиента g
        # решение QP через OSQP
        # обновление x_predicted, u_prev
        ...

    def reset(self):
        """Сброс state-предиктора и warm-start."""
        ...
```

API повторяет C++ интерфейс из `adas/acc_mpc`. Внутри — `numpy` для матриц
плюс `osqp` для решения QP.

### 9.3 Нода `swarm_acc_mpc_node.py`

```python
class SwarmAccMpcNode(Node):
    def __init__(self):
        super().__init__('swarm_acc_mpc')
        # параметры: telemetry_topic, odom_topic, cmd_vel_topic, robot_id,
        #            физика (m, b, alpha, tau_F),
        #            MPC (ts, p, c, s, phi_vals, q_vals, d0, th),
        #            limits (u_min/max, F_min/max), start
        self.controller = SwarmAccController(...)
        self.create_subscription(Telemetry, telemetry_topic, self.tel_cb, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        # таймер на ts (50 мс): решение QP, публикация Twist

    def tel_cb(self, msg):  # сохраняет dx, peer_v, azimuth
    def odom_cb(self, msg): # сохраняет v_actual
    def control_step(self):
        u, y = self.controller.calculate_control(dx, v, v_rel)
        twist = Twist(); twist.linear.x = u; twist.angular.z = ...
        self.cmd_pub.publish(twist)
```

Опционально для отладки: топики `/swarm_acc/y_vector{N}`, `/swarm_acc/dist_ref{N}` —
output MPC и референс гэпа в виде `geometry_msgs/Vector3` / `Float64`.

### 9.4 Wire-up через launch

`swarm_controller{N}.launch.py` на этапе генерации описания читает
`sliding_mode` из `params{N}.yaml` и условно запускает одну из двух нод:

```python
import yaml

def _read_sliding_mode(params_file):
    with open(params_file) as f:
        data = yaml.safe_load(f)
    return data['/**']['ros__parameters'].get('sliding_mode', True)

def generate_launch_description():
    params = os.path.join(get_package_share_directory('swarm_controller'),
                          'config', 'params2.yaml')
    if _read_sliding_mode(params):
        node = Node(package='swarm_controller', executable='swarm_controller', ...)
    else:
        node = Node(package='swarm_controller', executable='swarm_acc_mpc_node',
                    parameters=[params,
                                os.path.join(..., 'params_swarm_acc.yaml')])
    ...
```

### 9.5 Зависимости

В `package.xml` добавить:
```xml
<exec_depend>python3-numpy</exec_depend>
<exec_depend>python3-scipy</exec_depend>
<exec_depend>python3-osqp</exec_depend>     <!-- если есть в rosdep -->
```
Или установить через pip: `pip install osqp scipy`. `numpy` уже есть как
зависимость `rclpy`.

---

## 10. Что отличается от `adas/acc_mpc`

| Аспект | `adas/acc_mpc` (CARLA) | `swarm_acc_mpc` |
|---|---|---|
| Язык | C++ (`OsqpEigen`, `Eigen`) | Python (`osqp`, `numpy`) |
| State | `[dx, v, v_rel, a, j]` | `[dx, v, v_rel, F]` |
| State dim | 5 | 4 |
| Control | `u` = желаемое ускорение | `u` = желаемая скорость `v_cmd` |
| Plant lag | `τ` (один параметр, lag по `a`) | `τ_F` (lag по `F`) + `m, b` явно |
| Output | `[dx − d0 − th·v, v_rel, a, j]` | `[dx − d0 − th·v, v_rel, F]` |
| Output dim | 4 | 3 |
| Solver | OSQP через `OsqpEigen` | OSQP через `osqp` (Python bindings, та же C-библиотека) |
| Управление | `CarlaEgoVehicleControl` (throttle/brake) | `geometry_msgs/Twist` |
| Зависимости | `carla_msgs`, `control_toolbox` | `swarm_msgs`, `nav_msgs`, `python3-osqp` |
| Назначение | автомобиль на 10–30 м/с | малый робот на 0–0.5 м/с |

---

## 11. Что вне scope этого контроллера

Контроллер решает только **продольную задачу** для одного follower-а. Намеренно
не входит в эту реализацию:

- **Боковая динамика и `angular.z`**. Управление углом задаётся отдельным
  геометрическим контроллером по азимуту peer (тот же, что использует sliding
  mode). Полноценный 2D MPC с моделью bicycle/diff-drive — отдельный большой
  проект.

- **Lateral MPC**. Расширение state-пространства до `[..., θ, ω]` с
  динамикой угла потребует переработки матриц и принципиально иной модели
  плана.

- **Управление pacemaker-ом**. Pacemaker имеет свою задачу — следование
  предзаданной траектории. Для него требуется отдельный CC MPC (без `dx`,
  `v_rel`) с reference по скорости / профилю пути.

- **CA-модель лидера**. CV-допущение `v̇_peer = 0` принято как достаточное
  для типичных сценариев swarm. При резких манёврах pacemaker предиктор будет
  отставать на один цикл MPC (50 мс), что приемлемо. Расширение до CA с
  `a_peer` как measured disturbance остаётся опцией без переделки структуры.

- **Hard constraint безопасного гэпа**. Условие `dx ≥ d_safe` сейчас
  обеспечивается косвенно через cost (большой `q_gap`). Явный hard
  constraint с slack-переменной — расширение для production-сценариев.

- **Recovery-логика**. Поведение при `is_valid=false` (потеря лидера) — на
  уровне ноды, не контроллера: публикуется нулевой `Twist` (как в sliding
  mode).

---

## Ссылки

- Оригинальная реализация для CARLA: [`adas/src/acc_mpc.cpp`](../../../adas/src/acc_mpc.cpp),
  [`adas/include/adas/acc_mpc.hpp`](../../../adas/include/adas/acc_mpc.hpp).
- Headway-time policy: см. `dist_ref = d0 + th·v` в `adas/src/acc_node.cpp:189`.
- Симулятор робота: [`simulator.py`](../swarm_controller/simulator.py).
- STM32 PID: `robot/microros_ws/src/microros_stm32/Core/Src/robot.c`, функция `pid()`.
- Текущий sliding-mode follower: [`swarm_controller.py`](../swarm_controller/swarm_controller.py).
