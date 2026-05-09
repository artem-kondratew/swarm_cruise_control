# MPC-контроллеры follower-роботов: математика

Документ описывает математическую формулировку двух MPC-контроллеров,
используемых для адаптивного круиз-контроля (ACC) follower-роботов в
swarm-проекте: **кинематического 5-state** и **динамического 6-state
(Newton + force lag)**. Оба строятся на одной и той же state-space MPC
машинерии и решаются OSQP.

---

## 0. Постановка задачи

Пусть `dx` — расстояние от follower-а до peer-лидера, измеряемое из
LIDAR через PeerLocalizer. Цель follower'а — поддерживать **time-headway
gap policy**:

$$\text{dx}_\text{ref}(v) = d_0 + t_h \cdot v$$

где `d_0` — минимальный гэп при нулевой скорости, `t_h` — time headway.
Определяем `dx_err = dx − d_0` (offset-free state) и output:

$$y_\text{gap} = dx_{err} − t_h \cdot v$$

`y_gap → 0` означает `dx → d_0 + t_h·v`. Это формулировка трекинга, ИЗ
КОТОРОЙ автоматически вытекает «держи дистанцию пропорционально твоей
скорости».

Сторонний контракт через ROS Telemetry: follower измеряет own `v` (odom)
и получает `peer_v` от лидера (V2V). Это режим **CACC** (Cooperative
ACC), не вычно ACC.

---

## 1. Общая state-space MPC формулировка

Обе вариации используют общую структуру.

### 1.1 Plant model (LTI discrete)

$$x_{k+1} = A x_k + B u_k, \qquad y_k = C x_k$$

- `x ∈ ℝⁿ` — state (включая `dx_err`, `v`, `v_rel`)
- `u ∈ ℝ` — control input (`a_cmd`)
- `y ∈ ℝᵐ` — output для cost функции

### 1.2 Cost функция

Над prediction horizon `p`, control horizon `c ≤ p`:

$$J(\mathbf u) = \sum_{i=1}^{p} (y_{ref,k+i} − y_{k+i})^\top Q (y_{ref,k+i} − y_{k+i}) + s \sum_{m=0}^{c-1} (\Delta u_m)^2$$

- `Q = diag(q_vals)` — output weights
- Reference shaping: `y_ref,k+i = Φⁱ · y_now`, `Φ = diag(phi_vals)`,
  `0 ≤ phi < 1`. Текущий output экспоненциально затухает к нулю по
  горизонту → плавный целевой trajectory вместо резкого «прыжка к нулю»
- `s` — move suppression на `Δu_m = u_m − u_{m−1}` (≈ jerk·ts)

### 1.3 Свободный отклик и convolution matrix

Раскладываем будущий output на «свободный» (без управления) и forced:

$$Y_p = C_\text{hat} \cdot x + D_\text{hat} \cdot \mathbf u$$

- `C_hat ∈ ℝ^{p·m × n}`: вертикальный stack `C·Aⁱ` для `i = 1..p`
- `D_hat ∈ ℝ^{p·m × c}`: convolution matrix импульсного отклика
  `(C·Aⁱ⁻ᵐ⁻¹·B)` (block Toeplitz)

### 1.4 QP

После свёртки cost функции получаем quadratic program:

$$\min_{\mathbf u} \tfrac{1}{2} \mathbf u^\top H \mathbf u + g^\top \mathbf u, \quad \text{s.t.} \quad lb \le A_c \mathbf u \le ub$$

- `H = D_hatᵀ Q̄ D_hat + S` (постоянная, считается один раз в `__init__`)
- `S` — Toeplitz move-suppression matrix: `S[i,i] = 2s`,
  `S[i,i±1] = −s`, кроме диагональных краёв
- `g = −M_2 · u_\text{prev} − D_hatᵀ Q̄ b_1`, где
  `b_1 = Y_\text{ref} − F_2`,
  `F_2 = C_\text{hat} x + F_\text{hat} ex` — свободный отклик с
  коррекцией предиктора
- `Q̄ = block_diag(Q, ..., Q)` (`p` копий)

### 1.5 Predictor (velocity-form / disturbance form)

После решения QP оптимальное `u_opt = u_0`:

```
ex = x_now − x_predicted_prev      # ошибка предсказания на измеряемых state'ах
x_predicted_next = A·x_now + B·u_opt + H_obs·ex      # H_obs = I
```

`ex` ненулевая только на **измеряемых** позициях state (`dx_err`, `v`,
`v_rel`). На оценочных (`a` или `F, v_cmd, e_int`) `ex = 0` — туда
копируется значение из предыдущего предиктора. Применение `ex` ко всему
горизонту через `F_hat` действует как простой disturbance estimator.

---

## 2. Кинематический MPC (5-state)

**Файл**: [`swarm_acc_kin_mpc.py`](../swarm_controller/submodules/swarm_acc_kin_mpc.py)

### 2.1 Plant model

Робот моделируется как точечная масса с одноуровневым приводом первого
порядка по ускорению. Один параметр `τ`:

$$\dot a = (u − a)/\tau, \qquad \dot v = a$$

Команда `u = a_\text{cmd}` отслеживается фактическим ускорением `a` с
постоянной времени `τ`. Скорость `v` — интеграл от `a`.

### 2.2 State и control

$$x = [dx_{err}, v, v_{rel}, a, j]^\top \in \mathbb R^5, \qquad u = a_\text{cmd} \in \mathbb R$$

| Компонент | Смысл | Источник |
|---|---|---|
| `dx_err = dx − d_0` | offset gap state | измерение |
| `v` | own longitudinal speed | odom |
| `v_rel = peer_v − v` | relative velocity | CACC telemetry |
| `a` | actual acceleration (lagged from u) | оценочно |
| `j = (u − a)/τ` | observed jerk (наблюдаемая величина) | оценочно |

`j` — не state в смысле динамики (нет своей правой части), но добавлен
для штрафа `q_j · j²` (clean damping: `j = 0` в steady state).

### 2.3 Continuous → discrete (Euler `ts = 0.05 s`)

$$
\begin{aligned}
dx_{err,k+1} &= dx_{err,k} + t_s v_{rel,k} − \tfrac{1}{2} t_s^2 a_k \\
v_{k+1}      &= v_k + t_s a_k \\
v_{rel,k+1}  &= v_{rel,k} − t_s a_k \quad \text{(peer CV)} \\
a_{k+1}      &= (1 − t_s/\tau) a_k + (t_s/\tau) u_k \\
j_{k+1}      &= -(1/\tau) a_k + (1/\tau) u_k
\end{aligned}
$$

В матричном виде:

$$
A = \begin{pmatrix}
1 & 0 & t_s & -\tfrac{1}{2}t_s^2 & 0 \\
0 & 1 & 0 & t_s & 0 \\
0 & 0 & 1 & -t_s & 0 \\
0 & 0 & 0 & 1 - t_s/\tau & 0 \\
0 & 0 & 0 & -1/\tau & 0
\end{pmatrix},
\quad
B = \begin{pmatrix} 0 \\ 0 \\ 0 \\ t_s/\tau \\ 1/\tau \end{pmatrix}
$$

### 2.4 Output

$$y = [y_{gap}, v_{rel}, a, j]^\top, \qquad y_{gap} = dx_{err} - t_h v$$

$$
C = \begin{pmatrix}
1 & -t_h & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 1
\end{pmatrix}
$$

В steady state с `v = v_peer`: `v_rel = 0`, `a = 0`, `j = 0`, и
`y_gap = 0` если `dx_err = t_h v_peer`. **Все 4 выхода одновременно
обнуляются**, cost = 0 — единственное равновесие.

### 2.5 Cost веса

```yaml
phi_vals: [0.6, 0.95, 0.6, 0.6]
q_vals:   [10.0, 1.0, 1.0, 1.0]   # q_gap, q_vrel, q_a, q_j
s:         3.0
```

### 2.6 Constraints

- Input bound: `a_min ≤ u_m ≤ a_max` для всех `m ∈ [0, c)`
- Hard safety: `dx_{k+i} ≥ \text{gap}_\text{safe}` над всем prediction
  horizon (формально:
  `dx_{err,k+i} ≥ \text{gap}_\text{safe} - d_0`)

### 2.7 Публикация v_cmd

Контроллер выдаёт **только `a_cmd`**. Интегрирование в node:

```python
self._v_cmd_published += self.ts * a_cmd
self._v_cmd_published = clip(_v_cmd_published, [v_cmd_min, v_cmd_max])
twist.linear.x = self._v_cmd_published
```

Это **anti-windup на актюаторе**: clip применяется после QP, поэтому
оптимизатор планирует свободно.

---

## 3. Динамический MPC (6-state Newton + force lag)

**Файл**: [`swarm_acc_mpc.py`](../swarm_controller/submodules/swarm_acc_mpc.py)

### 3.1 Plant model

Двухуровневая физика, идентичная симулятору:

$$\dot v = (F - bv)/m \quad \Leftrightarrow \quad m\dot v = F - bv \quad \text{(закон Ньютона)}$$

$$\dot F = (\alpha (v_\text{cmd} - v) + bv - F) / \tau_F \quad \text{(внутренний PID-привод)}$$

| Параметр | Смысл | Идентификация |
|---|---|---|
| `m` | масса робота, кг | весы |
| `b` | вязкое трение, Н·с/м | coast-down |
| `α` | gain force-генератора, Н·с/м | step-response |
| `τ_F` | постоянная отклика тяги, с | step-response |

**Проверка steady state**: при `Ḟ = 0` ⟹ `F = α(v_cmd − v) + b·v`; при
`v̇ = 0` ⟹ `F = b·v`. Подставляя: `α(v_cmd − v) = 0` ⟹ `v = v_cmd`.
Точное отслеживание команды скорости.

### 3.2 State и control

$$x = [dx_{err}, v, v_{rel}, F, v_\text{cmd}, e_\text{int}]^\top \in \mathbb R^6, \qquad u = a_\text{cmd}$$

| Компонент | Смысл |
|---|---|
| `dx_err, v, v_rel` | как у кинематики (измеряются) |
| `F` | actual drive force (state of inner loop) |
| `v_cmd` | внутренний интегратор: `v_cmd_dot = u` |
| `e_int = ∫ y_gap dt` | integral action на gap error |

**Ключевое отличие от кинематики**: `u` физически есть **производная
команды скорости** (jerk-equivalent на уровне команды), а не «желаемое
ускорение, которому привод следует с лагом». `v_cmd` накапливает `u·t_s`
и публикуется в plant.

### 3.3 Continuous → discrete

$$
\begin{aligned}
dx_{err,k+1} &= dx_{err,k} + t_s v_{rel,k} \\
v_{k+1}      &= (1 - t_s b/m) v_k + (t_s/m) F_k \\
v_{rel,k+1}  &= (t_s b/m) v_k + v_{rel,k} - (t_s/m) F_k \\
F_{k+1}      &= t_s \tfrac{b - \alpha}{\tau_F} v_k + (1 - t_s/\tau_F) F_k + t_s \tfrac{\alpha}{\tau_F} v_{cmd,k} \\
v_{cmd,k+1}  &= v_{cmd,k} + t_s u_k \\
e_{int,k+1}  &= t_s dx_{err,k} - t_s t_h v_k + e_{int,k}
\end{aligned}
$$

В матричной форме:

$$
A = \begin{pmatrix}
1 & 0 & t_s & 0 & 0 & 0 \\
0 & 1 - \tfrac{t_s b}{m} & 0 & \tfrac{t_s}{m} & 0 & 0 \\
0 & \tfrac{t_s b}{m} & 1 & -\tfrac{t_s}{m} & 0 & 0 \\
0 & \tfrac{t_s(b-\alpha)}{\tau_F} & 0 & 1 - \tfrac{t_s}{\tau_F} & \tfrac{t_s \alpha}{\tau_F} & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
t_s & -t_s t_h & 0 & 0 & 0 & 1
\end{pmatrix},
\quad
B = \begin{pmatrix} 0 \\ 0 \\ 0 \\ 0 \\ t_s \\ 0 \end{pmatrix}
$$

`B` имеет ненулевую запись только в строке `v_cmd` — `u` входит **только
через интегратор**. `F` отвечает на `v_cmd` через `A[3, 4] = t_s α/τ_F`.

**Путь от `u` до физической `v`** (3 уровня динамики):

$$u \xrightarrow{\int} v_\text{cmd} \xrightarrow{\text{lag } \tau_F} F \xrightarrow{1/m,\ −b/m} v$$

У кинематики только 2 уровня (`u → a → v`). Лишняя ступень требует
больше demping (s = 20 vs 3 у кинематики).

### 3.4 Output

$$y = [y_{gap}, v_{rel}, a, e_{int}]^\top$$

Третий выход — **физическое ускорение**, линейное по state:

$$a = \dot v = (F - bv)/m \quad\Rightarrow\quad C[2, :] = [\,0,\ -b/m,\ 0,\ 1/m,\ 0,\ 0\,]$$

В steady state `F = bv` ⟹ `a = 0`. Поэтому `q_a · a²` — clean damping
без offset. Это **математически чище**, чем штрафовать `F` напрямую
(`F = bv ≠ 0` в steady state создаёт offset). Результат структурно
идентичен `q_j · j²` у кинематики.

Полная C-матрица:

$$
C = \begin{pmatrix}
1 & -t_h & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 \\
0 & -b/m & 0 & 1/m & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{pmatrix}
$$

### 3.5 Cost веса

```yaml
phi_vals: [0.6, 0.95, 0.6, 0.9]
q_vals:   [10.0, 1.0, 0.0, 2.0]   # q_gap, q_vrel, q_a, q_int
s:         20.0
```

`q_int = 2` — критический вес. Без него каскадная установка
накапливает миллиметровый bias скорости в сантиметровый offset за
минуту (см. §5.2). Эмпирический sweep:

| `q_int` | r2 mean offset (cascade) |
|---|---|
| 0 | +27 cm |
| 1 | +10 cm |
| 2 | **+1 cm** ← выбрано |

`q_a = 0` — move suppression `s = 20` уже даёт нужный demping;
дополнительный штраф на `a²` на практике сужал коридор оптимума и
ухудшал ответ.

### 3.6 Constraints (после фикса)

- Input bound: `a_min ≤ u_m ≤ a_max` (`c` строк)
- Hard safety: `dx_{k+i} ≥ \text{gap}_\text{safe}` (`p` строк)

**Раньше была дополнительно строка** `v_\text{cmd,k+i} ∈ [v_\text{cmd,min}, v_\text{cmd,max}]`
над всем горизонтом. Удалена — см. §5.3 объяснение.

### 3.7 Anti-windup на v_cmd state

После advance предиктора:

```python
self._x_predicted = A @ x + B * u_opt + ex
self._x_predicted[4] = clip(self._x_predicted[4], [v_cmd_min, v_cmd_max])
v_cmd_published = self._x_predicted[4]
```

Это сохраняет internal v_cmd state синхронным с тем, что plant получает
через clipped Twist. Без этого clip'а predictor-овский `v_cmd_state`
мог уйти в saturation, при том что plant видит только clipped значение
→ MPC планирует на базе wrong v_cmd. Это **зеркало кинематического**
clip'а на `_v_cmd_published`.

---

## 4. Steady-state анализ

### 4.1 Условие offset-free для трекинга

Для линейной MPC без integrator action минимум `J` достигается на
**неединственном** equilibrium-семействе — есть свободный параметр
(constant disturbance state). Чтобы `y_gap → 0` строго, нужен один из
двух механизмов:

1. **Output structure без свободных параметров.** В кинематике состав
   `(dx_err, v, v_rel, a, j)` плюс output `(y_gap, v_rel, a, j)` имеет
   уникальное равновесие `dx_err = t_h v_peer, v = v_peer, остальные 0`,
   cost = 0. Достаточно при идеальной модели.
2. **Augmented integrator state.** Добавить `e_int = ∫ y_gap dt`. В
   linearized cost, штраф `q_int · e_int²` экспоненциально расходится
   при ненулевом mean(`y_gap`), вынуждая optimizer в среднем держать
   `y_gap = 0`. Стандартная техника **offset-free MPC** при наличии
   model mismatch или disturbance.

### 4.2 Почему Newton требует e_int, а кинематика — нет

Кинематика имеет 5 поломанных уровней; в идеальной модели cost = 0
достижим без integrator. На практике (model mismatch, измерительный
шум) кинематика тоже имеет малый residual offset, но малый.

Newton имеет 6 уровней — на один больше. Эта **лишняя ступень**
(internal `v_cmd` integrator) добавляет интегральную свободу: даже при
идеальной модели миллиметровый bias в `v_cmd_state` (который publishes
∫u·dt) накапливается в `v ≠ v_peer` и далее в `dx_err ≠ t_h v`. Без
integrator action `e_int`, optimizer не имеет «штрафной палки»,
заставляющей average y_gap = 0.

С `q_int = 2`: `e_int` integrator растёт при positive mean `y_gap`,
QP видит growing cost, активно компенсирует. Mean offset падает до 1 см.

### 4.3 Почему v_cmd state-bound над горизонтом был вреден

Старый constraint:

$$v_\text{cmd,min} \le v_\text{cmd,k+i} \le v_\text{cmd,max} \quad\forall i \in [1, p]$$

переписывается через `M_\text{free,v} x + D_v u`:

$$v_\text{cmd,min} - v_\text{cmd,free}_i \le (D_v u)_i \le v_\text{cmd,max} - v_\text{cmd,free}_i$$

Здесь `v_\text{cmd,free}_i = v_\text{cmd,est}` (постоянна по горизонту,
т.к. под `u = 0` интегратор стоит). При `v_\text{cmd,est} ≈ v_peer = 0.4`,
`v_\text{cmd,max} = 0.5`:

$$\text{headroom} = 0.5 - 0.4 = 0.1 \text{ m/s}$$

Этот 0.1 должен **вместить кумулятивный интеграл всех будущих `u`**
вплоть до шага `i`. На дальних шагах горизонта (`i = p = 20`), даже
небольшое `u > 0` накапливается → row binding → optimizer консервативно
урезает план. В реальности применяется только `u_0` (receding horizon)
→ ограничение **формально ложное**, душит authority зря.

После удаления constraint'а оптимизатор планирует свободно; saturation
ловится anti-windup'ом на v_cmd state (§3.7). Это **точно та же
структура**, что у кинематики.

---

## 5. Сравнение

| Аспект | Kinematic | Newton |
|---|---|---|
| State size | 5 | 6 |
| Plant params | 1 (`τ`) | 4 (`m, b, α, τ_F`) |
| `u` физически | желаемое ускорение | производная `v_cmd` |
| Путь `u → v` | 2 уровня | 3 уровня |
| Output | `[y_gap, v_rel, a, j]` | `[y_gap, v_rel, a, e_int]` |
| `q_vals` | `[10, 1, 1, 1]` | `[10, 1, 0, 2]` |
| `s` | 3 | 20 |
| QP constraints | u-bound + safety | u-bound + safety |
| Anti-windup | clip `_v_cmd_published` в node | clip `x[4]` в predictor'е |
| Идентификация plant | один step response | coast-down + step response |
| Cascade r2 mean err | `−0.0001 m` | `+0.012 m → ≤0.01 m` после фикса |
| Cascade r2 std err | `0.009 m` | `0.036 m → ≤0.015 m` после фикса |

### Когда какой использовать

- **Кинематика**: параметры робота неизвестны / меняются (большой
  robustness margin), нужен простой setup, допустимы model mismatch.
- **Newton**: параметры идентифицированы точно, нужна explicit hard
  safety (force-lag правильно предсказывает stopping distance),
  тонкая настройка через физические параметры.

В идеальном симуляторе после описанных фиксов обе формулировки дают
эквивалентный результат. На реальном железе ожидается, что Newton
выигрывает при правильной идентификации, кинематика — устойчивее при
неточных параметрах.

---

## 6. Идентификация Newton-параметров

### 6.1 `b` через coast-down

1. Разогнать робота до `v_0 ≈ 0.4` m/s
2. Отключить мотор (`cmd_vel = 0`)
3. Записать `v(t)` из одометрии
4. Фитнуть экспоненту `v(t) = v_0 · exp(-bt/m)` (`m` уже измерена)

### 6.2 `α, τ_F` через step response

1. Робот в покое, `v = 0`
2. Скачок `v_cmd = 0 → 0.3` m/s в момент `t = 0`
3. Записать `v(t)`, оценить `F(t) = m v̇(t) + b v(t)` численно
4. Фитнуть первый порядок `F(t) = F_∞ (1 − exp(−t/τ_F))`
5. `α = F_∞ / (v_cmd − v_∞)`, где `v_∞` — установившаяся `v` (≈ `v_cmd`)

### 6.3 Robustness тестирование

Для проверки чувствительности MPC к model mismatch ставим **разные**
значения `m, b, α, τ_F` в [params_simulator.yaml](../config/params_simulator.yaml)
и в [params_swarm_acc.yaml](../config/params_swarm_acc.yaml). Стандартные
сценарии:

| Параметр | Реальная неточность | Тест |
|---|---|---|
| `m` | ±20% | 1.6, 2.0, 2.4 |
| `b` | ±30% | 0.7, 1.0, 1.3 |
| `α` | ±30% | 2.8, 4.0, 5.2 |
| `τ_F` | ±50% | 0.1, 0.2, 0.3 |

---

## Appendix A. Move-suppression matrix

Для control horizon `c`, штраф `s · Σ_{m=0}^{c-1} (Δu_m)²` =
`uᵀ S u + 2 sᵀ u + const`, где

$$
S = s \begin{pmatrix}
2 & -1 & & \\
-1 & 2 & -1 & \\
& -1 & 2 & \ddots \\
& & \ddots & \ddots & -1 \\
& & & -1 & 1
\end{pmatrix}_{c \times c}
$$

Последний диагональный элемент = `s` (не `2s`), потому что нет
последующего `Δu_c`. Coupling с `u_prev` через дополнительный линейный
член `−s u_prev u_0` (вектор `M_2 = [s, 0, ..., 0]`).

## Appendix B. Reference shaping `Φᵏ`

Идея: вместо мгновенного reference `y_ref = 0` использовать
сглаженную траекторию `y_ref(k+i) = Φⁱ y_now`, где
`Φ = diag(phi_vals)`, `0 ≤ phi < 1`. На ближних шагах reference
близок к текущему output → нет резкого «прыжка», который вызывал бы
большой `u`. На дальних шагах reference → 0 → optimizer всё равно
давит к equilibrium.

Эффект: эквивалент soft setpoint filter, как в industrial PID. Делает
MPC менее агрессивным на больших ошибках.

## Appendix C. OSQP solver

Решаем стандартный QP:

$$\min_z \tfrac{1}{2} z^\top P z + q^\top z, \qquad l \le A z \le u$$

OSQP использует ADMM. Hessian `P` константа (рассчитывается один раз
в `__init__`), gradient `q` и bounds `l, u` обновляются каждый цикл
через `prob.update()`. Warm starting активирован после первого
solved-status.

Time limit: `0.8 · t_s = 40 ms`. При infeasibility (например, peer
тормозит резче чем мы можем отреагировать) fallback: `u = a_min`
(максимальное торможение), warm start сбрасывается.
