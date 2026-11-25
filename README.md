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
Таймеры TIM3/TIM4/TIM9 только инициализированы (PWM CHx, PSC=0, ARR=65535), не запущены.
Управление моторами — задача T2.

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
