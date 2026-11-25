# TASK_T1_PROJECT_SKELETON — AeroStand: скелет проекта

## Контекст

AeroStand — учебный HIL-стенд для отладки прошивок полётных контроллеров и оперения планеров.
Плата: STM32F429I-DISC1 (STM32F429ZIT6, 180 МГц, 2 МБ Flash, 256 КБ RAM, TFT 2.4").
Приводы: 3× NEMA17 (17HS08-1004S, 1.0A) через драйверы MP8825 на платах MP1510.
Интерфейс к драйверам: STEP + DIR + EN (3 GPIO на ось).

## Репозиторий

- GitHub: https://github.com/Wagan/AeroStand.git
- Локальная папка: C:\Wagan\AeroStand
- CubeIDE workspace: C:\Users\Sarukhanov\STM32CubeIDE\workspace_1.16.0

## Что сделать

1. Склонировать репозиторий в C:\Wagan\AeroStand
2. Создать STM32CubeIDE проект для STM32F429ZIT6 (Board: STM32F429I-DISC1) в этой папке
3. Настроить тактирование: HSE 8 МГц (от ST-Link), PLL → SYSCLK 180 МГц, APB1 45 МГц, APB2 90 МГц
4. Настроить USB CDC (USB_OTG_FS, Virtual COM Port) — для связи с ПК
5. Зарезервировать GPIO для 3 осей (конкретные пины назначить по удобству разводки на DISC1, избегая конфликтов с LCD и USB):
   - Ось 0: STEP0, DIR0, EN0
   - Ось 1: STEP1, DIR1, EN1
   - Ось 2: STEP2, DIR2, EN2
   - STEP-пины должны быть на каналах таймеров (для генерации импульсов по прерыванию или PWM)
6. Создать папку `docs/` для TASK- и REPORT-файлов
7. Добавить README.md:
   - Название: AeroStand
   - Назначение: Educational HIL test bench for flight controller firmware and control surface testing
   - Автор: Vagan Sarukhanov
   - Плата: STM32F429I-DISC1
   - Приводы: 3× NEMA17 via MP8825 (STEP/DIR/EN)
   - Таблица назначенных GPIO (заполнить по факту)
8. Собрать — проект должен компилироваться без ошибок
9. Первый коммит и push

## Чего НЕ делать

- Не писать код управления моторами — это T2
- Не трогать LCD — это позже
- Не менять структуру папок CubeIDE (стандартная генерация CubeMX)

## Критерий приёмки

- Проект компилируется чисто (0 errors, 0 warnings)
- USB CDC инициализирован
- GPIO для 3 осей назначены и задокументированы в README
- STEP-пины на таймерных каналах
- Коммит на GitHub
