/**
  * @file    cmd_parser.h
  * @brief   Текстовый командный протокол по USB CDC (строки завершаются '\n').
  *
  * Приём: CDC_Receive_HS -> cmd_rx_push() -> кольцевой буфер RX.
  * Разбор и ответы: cmd_poll() из главного цикла.
  * Передача: ответы копятся в кольцевом буфере TX и уходят через CDC_Transmit_HS.
  */
#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#include <stdint.h>

#define CMD_LINE_MAX    64U     /* максимальная длина строки без '\n' */

/** Сброс буферов. */
void cmd_init(void);

/** Положить принятые байты в кольцевой буфер RX (вызывается из USB-прерывания). */
void cmd_rx_push(const uint8_t *data, uint32_t len);

/**
  * Вызывать в главном цикле: разбирает принятые строки, отправляет
  * "DONE <axis>" по завершённым движениям, проталкивает TX в USB.
  */
void cmd_poll(void);

/**
  * Разобрать и выполнить одну строку (без '\n'; '\r' в конце допускается).
  * Ответ дописывается в буфер TX. Строка изменяется. Экспортирована для тестов.
  */
void cmd_execute(char *line);

#endif /* CMD_PARSER_H */
