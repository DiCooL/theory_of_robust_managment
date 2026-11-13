# Управление углом крена БПЛА на гироскопическом стенде методом MFC

Репозиторий рабочего проекта ИПУ РАН: математическое моделирование системы «гироскопический стенд + квадрокоптер» и адаптация методов управления для БПЛА. Отправная точка — ВКР (МГТУ им. Н. Э. Баумана, 2026): управление креном методом обратной связи с линейной номинальной моделью (Model Feedback Control, MFC; Narikiyo & Izumi, 1991).

Основной язык проекта — Python (пакет `uav_mfc`). MATLAB-код сохранён в `matlab/` как эталон и архив.

## Структура

```
uav_mfc/                 Python-пакет
  params.py              параметры объекта, регулятора, фильтра, шума, моделирования
  trajectory.py          программные траектории α_d(t) с производными
  plant.py               нелинейная модель крена, шаг интегрирования, обратная линейная модель
  control.py             программное управление ū и корректирующий фильтр
  simulation.py          замкнутый контур (без внутреннего контура / обратная модель / MFC)
  tuning.py              перебор (ω_n, ζ) по функционалу J
  plotting.py            графики
experiments/
  run_experiment.py      один прогон с параметрами из командной строки
  thesis_experiments.py  воспроизведение экспериментов ВКР и сравнение с MATLAB
  tune_parameters.py     подбор ω_n, ζ
tests/                   pytest; сверка с MATLAB-кодом, прогнанным в Octave
matlab/
  thesis_2026/           MATLAB-код ВКР (exp1…exp3_1, param.m и функции)
  coursework_2025_*/     код курсовой 2025 (первый закон управления, a = −0.3)
  robot_manipulator/     воспроизведение примера Narikiyo для трёхзвенного манипулятора
docs/
  article_draft_2025.pdf черновик статьи 2025 (курсовой этап, неактуален)
```

Чужие работы в репозиторий не добавляются — только ссылки на них (раздел «Источники»).

## Установка (Windows, PyCharm)

```
git clone https://github.com/DiCooL/theory_of_robust_managment.git
cd theory_of_robust_managment
py -m venv .venv
.venv\Scripts\activate
pip install -e ".[dev]"
pytest
```

В PyCharm: File → Open → папка репозитория; интерпретатор — `.venv\Scripts\python.exe`. После `pip install -e .` пакет `uav_mfc` импортируется из любого скрипта, а правки в нём подхватываются без переустановки. Нужен Python 3.10+.

## Запуск

```
python experiments\thesis_experiments.py
python experiments\run_experiment.py --loop mfc --noise thesis --plot
python experiments\run_experiment.py --loop none --noise none --trajectory atan --plot
python experiments\run_experiment.py --noise thesis --noise-mode measurement
python experiments\tune_parameters.py --omega 10 100 10 --zeta 1.00 1.02 0.01
```

Ключи `run_experiment.py`: `--loop {none,inverse_model,mfc}`, `--noise {none,thesis,strong}`, `--noise-mode {state,measurement}`, `--trajectory {sin,atan,const}`, `--omega-n`, `--zeta`, `--T`, `--dt`, `--t-final`, `--seed`, `--plot`, `--save file.png`.

Из кода:

```python
from uav_mfc import simulate, SimConfig, THESIS_NOISE, PlantParams

res = simulate(noise=THESIS_NOISE, config=SimConfig(inner_loop="mfc", seed=2))
print(res.rmse_deg)

mismatch = simulate(plant=PlantParams(inertia=0.03), model=PlantParams(), config=SimConfig(inner_loop="mfc"))
```

`plant` — параметры «реального» объекта, `model` — параметры, которые знает регулятор (по умолчанию совпадают). Это позволяет исследовать рассогласование модели и объекта.

## Модель и закон управления

Канал крена стенда (Nikolaev, Ryakhimov, Yurchenkov, ICSC; уравнение (2), первая строка, β = 0):

$$\ddot\alpha = \frac{M_{bx} + 0.3\sin\alpha - 2\mu\dot\alpha}{0.02},\qquad \mu = 2.5\cdot10^{-3}.$$

Положение α = 0 — верхнее неустойчивое равновесие (член 0.3 sin α дестабилизирует).

Программное управление (ВКР, раздел 5):

$$\bar u = -(0.3\sin\alpha - 2\mu\dot\alpha) + 0.02\,\big(a\dot\alpha + b(\alpha - \alpha_d)\big),\qquad a = -2\zeta\omega_n,\ b = -\omega_n^2.$$

Внутренний контур MFC (на каждом шаге, после интегрирования объекта):

$$\tilde u = 2\mu\dot\alpha,\qquad u_{fil}[k{+}1] = u_{fil}[k] + dt\,T\,(M_{bx}[k] - u_{fil}[k]),\qquad \Delta u = \tilde u - u_{fil},\qquad M_{bx} = \bar u - \Delta u.$$

Номинальная модель $G_0(s) = 1/(2\mu s)$ и фильтр $f(s) = T/(s+T)$ выбраны по аналогии с примером Narikiyo & Izumi (1991), где $G_0(s) = \mathrm{diag}[1/(k_i s)]$. На вход фильтра подаётся момент $M_{bx}$.

Интегрирование объекта: $\dot\alpha_{k+1} = \dot\alpha_k + \ddot\alpha_k\,dt$, $\alpha_{k+1} = \alpha_k + \dot\alpha_k\,dt + \ddot\alpha_k\,dt^2/2$; $dt = 10^{-3}$ с.

Итоговые параметры ВКР: $T = 15$, $\omega_n = 100$, $\zeta = 1.01$ ($a = -202$, $b = -10^4$); траектория $\alpha_d = \tfrac{\pi}{6}\sin(\tfrac{\pi}{10}t - \tfrac{\pi}{2}) + \tfrac{\pi}{6}$; горизонт 20 с.

## Соответствие MATLAB → Python

| MATLAB (`matlab/thesis_2026`) | Python |
|---|---|
| `uav_model.m` | `plant.step` |
| `inverse_linear_model.m` | `plant.inverse_linear_model` |
| `controller.m` | `control.program_control` |
| `function_correction.m` | `control.CorrectionFilter` |
| `desired_trajectory.m` | `trajectory.Sinusoid`, `Arctan`, `Constant` |
| `exp1.m` / `exp1_1.m` | `inner_loop="none"`, без шума / `THESIS_NOISE` |
| `exp2.m` / `exp2_1.m` | `inner_loop="inverse_model"`, без шума / `STRONG_NOISE` |
| `exp3.m` / `exp3_1.m` | `inner_loop="mfc"`, без шума / `THESIS_NOISE` (= приложение А ВКР) |
| `param.m` | `tuning.grid_search` |

`exp1_1` и `exp3_1` используют шум 0.3°, 0.4°/с и 0.004; `exp2_1` — 2.6°, 2.8°/с и 0.04.

## Проверка

MATLAB-скрипты прогнаны в GNU Octave 8.4; `tests/test_matlab_equivalence.py` сравнивает траектории без шума с точностью ~1e-15.

| Эксперимент | Python, RMSE ° | MATLAB-код в Octave, RMSE ° |
|---|---|---|
| exp1 | 0.1346 | 0.1280 |
| exp2 | 0.1348 | 0.1281 |
| exp3 | 0.1423 | 0.1360 |
| exp1_1 | 1.077 ± 0.026 | 1.029 |
| exp2_1 | 9.27 ± 0.22 | 8.85 |
| exp3_1 | 1.059 ± 0.025 | 1.013 |

Для шумовых случаев — среднее ± СКО по 30 зёрнам: генераторы случайных чисел MATLAB, Octave и NumPy дают разные последовательности. В ВКР для exp3_1 (MATLAB, `rng(2)`) указано 1.002°.

Столбцы различаются способом подсчёта ошибки. MATLAB-код (и ВКР) записывает состояние после шага интегрирования, $\alpha(t_{k+1})$, рядом с заданием $\alpha_d(t_k)$, то есть ошибка считается со сдвигом на $dt$. В Python состояние и задание записываются в один момент $t_k$. Сами траектории совпадают: тест сравнивает `alpha[k+1]` Python с записью MATLAB на шаге $k$.
