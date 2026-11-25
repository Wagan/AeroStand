# AeroStand

Educational HIL test bench for flight controller firmware and control surface testing.

- **Автор:** Vagan Sarukhanov
- **Плата:** STM32F429I-DISC1 (STM32F429ZIT6)
- **Приводы:** 3× NEMA17 via MP8825 (STEP/DIR/EN)

## Тактирование

HSE 8 МГц (bypass, MCO от ST-Link) → PLL (M=4, N=168, P=2, Q=7):

| Шина | Частота |
|---|---|
| SYSCLK / HCLK | 168 МГц |
| APB1 | 42 МГц (таймеры 84 МГц) |
| APB2 | 84 МГц (таймеры 168 МГц) |
| USB | 48 МГц (PLLQ) |

> **Отклонение от T1:** вместо 180/45/90 МГц выбрано 168/42/84 МГц. На F429 тактирование
> USB берётся только из PLLQ; при SYSCLK 180 МГц (VCO 360 МГц) получить 48 МГц нельзя
> (Q=7 → 51.4, Q=8 → 45 МГц). Решение согласовано с автором.

## USB CDC (Virtual COM Port)

Используется периферия **USB_OTG_HS во встроенном Full-Speed PHY** (Device only, класс CDC),
пины PB14 (DM) / PB15 (DP) — к ним подключён разъём USB на DISC1.

> **Отклонение от T1:** вместо USB_OTG_FS. Пины PA11/PA12 (USB_OTG_FS) на DISC1 заняты LCD
> (LTDC_R4/R5) и не выведены на разъём.

## Таблица GPIO

STEP-пины — на каналах таймеров (PWM / прерывание по обновлению). Остальные пины
свободны на DISC1 (не пересекаются с LCD, SDRAM, гироскопом, USB, ST-Link).

| Ось | Сигнал | Пин | Функция |
|---|---|---|---|
| 0 | STEP0 | PB4 | TIM3_CH1 (AF2) |
| 0 | DIR0 | PD2 | GPIO Output, старт = 0 |
| 0 | EN0 | PD4 | GPIO Output, старт = 1 (драйвер выключен) |
| 1 | STEP1 | PB7 | TIM4_CH2 (AF2) |
| 1 | DIR1 | PD5 | GPIO Output, старт = 0 |
| 1 | EN1 | PD7 | GPIO Output, старт = 1 (драйвер выключен) |
| 2 | STEP2 | PE5 | TIM9_CH1 (AF3) |
| 2 | DIR2 | PG2 | GPIO Output, старт = 0 |
| 2 | EN2 | PG3 | GPIO Output, старт = 1 (драйвер выключен) |

Уровень EN=1 соответствует выключенному драйверу (активный уровень EN у MP8825 — низкий).
Таймеры TIM3/TIM4/TIM9 в режиме PWM; управление — модуль `stepper` (T2).

## Command Protocol

Текстовый протокол поверх USB CDC (Virtual COM Port). Строки завершаются `
`
(`
` тоже принимается), максимальная длина строки — 64 символа. Имя команды
нечувствительно к регистру, аргументы разделяются пробелами. Пустые строки игнорируются.

### Команды (ПК → МК)

| Команда | Формат | Описание |
|---|---|---|
| MOVE | `MOVE <axis> <steps> <dir> <freq>` | Запустить движение: axis=0..2, steps>0, dir=0/1, freq=1..5000 Гц |
| STOP | `STOP <axis>` | Аварийная остановка оси (драйвер остаётся включённым) |
| ENABLE | `ENABLE <axis>` | Включить драйвер оси (EN=LOW) |
| DISABLE | `DISABLE <axis>` | Остановить ось и выключить драйвер (EN=HIGH) |
| STATUS | `STATUS` | Состояние всех осей |
| PING | `PING` | Проверка связи |

### Ответы (МК → ПК)

| Ответ | Формат | Когда |
|---|---|---|
| OK | `OK` | Команда принята и выполнена/запущена |
| DONE | `DONE <axis>` | Все шаги движения оси отработаны (асинхронно, после `OK` на MOVE). После STOP/DISABLE не приходит |
| ERR | `ERR <текст>` | Ошибка команды (см. ниже) |
| STATUS | `STATUS <axis> <enabled> <busy> <pos>` ×3 | По строке на каждую ось 0, 1, 2 |
| PONG | `PONG` | Ответ на PING |

`pos` — знаковый счётчик шагов: dir=1 прибавляет, dir=0 убавляет; обнуляется при включении МК.

### Тексты ошибок

| Ответ | Причина |
|---|---|
| `ERR UNKNOWN COMMAND` | неизвестная команда |
| `ERR ARGS` | неверное число аргументов |
| `ERR AXIS` | axis не число или вне 0..2 |
| `ERR STEPS` | steps не число или ≤ 0 |
| `ERR DIR` | dir не 0 и не 1 |
| `ERR FREQ` | freq не число или вне 1..5000 |
| `ERR BUSY` | ось уже движется (сначала STOP) |
| `ERR DISABLED` | драйвер оси выключен (сначала ENABLE) |
| `ERR LINE TOO LONG` | строка длиннее 64 символов |
| `ERR RX OVERFLOW` | переполнен приёмный буфер (256 байт) |

### Пример

```
> PING
< PONG
> ENABLE 0
< OK
> MOVE 0 200 1 1000
< OK
< DONE 0          (через 200 мс)
> STATUS
< STATUS 0 1 0 200
< STATUS 1 0 0 0
< STATUS 2 0 0 0
```

### Реализация

- `Core/Src/stepper.c` — STEP формируется таймером в режиме PWM (скважность ~50%): один
  период = один импульс = один шаг. Импульсы считаются в прерывании Capture/Compare
  (спад STEP); после последнего шага канал отключается. TIM3/TIM4/TIM9, приоритет IRQ = 1
  (USB = 0).
- Частота: тик 1 МГц при freq ≥ 16 Гц, 10 кГц при freq < 16 Гц (счётчики 16-битные);
  период округляется до тика, например при 5000 Гц погрешность нулевая, при 3000 Гц ~0.1%.
- DIR выставляется за ~1 мкс до первого импульса.
- `Core/Src/cmd_parser.c` — приём через `CDC_Receive_HS` → кольцевой буфер → разбор по `
`
  в главном цикле (`cmd_poll()`); ответы идут через TX-буфер и `CDC_Transmit_HS`.
  Если USB не сконфигурирован хостом, ответы отбрасываются.

## Сборка

STM32CubeIDE 1.16.0, конфигурация `Debug`. Headless:

```
stm32cubeidec.exe -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild ^
  -data <workspace> -import <путь к репозиторию> -no-indexer -cleanBuild "AeroStand/Debug"
```

## Структура

- `AeroStand.ioc` — конфигурация CubeMX
- `Core/`, `Drivers/`, `Middlewares/`, `USB_DEVICE/` — стандартная генерация CubeMX
- `docs/` — TASK- и REPORT-файлы
