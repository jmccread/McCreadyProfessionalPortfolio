#ifndef RBUFFER_H
#define RBUFFER_H
#include <stdint.h>

#define RBUFFER_BUFF_SIZE     512

typedef struct rbuffer_t rbuffer_t;

struct rbuffer_t {
  uint16_t  count;
  uint16_t  inP, outP;
  uint8_t   buff[RBUFFER_BUFF_SIZE];
};

extern void rbuffer_init(rbuffer_t *in_ptr);
extern void rbuffer_clear(rbuffer_t *in_ptr);
extern uint8_t  rbuffer_getchar(rbuffer_t *in_ptr);
extern uint8_t  rbuffer_putchar(rbuffer_t *in_ptr, uint8_t c);
extern uint16_t  rbuffer_putchars(rbuffer_t *in_ptr, uint8_t *in, uint16_t inLength);
extern uint16_t  rbuffer_count(rbuffer_t *in_ptr);
extern uint16_t  rbuffer_free(rbuffer_t *in_ptr);
extern void cmemcpy(char *dest, char *src, int start, int len);
#endif
