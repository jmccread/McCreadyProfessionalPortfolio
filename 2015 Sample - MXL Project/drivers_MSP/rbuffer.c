#include "rbuffer.h"

/******************************************************************************
****                                                                       ****
**                                                                           **
rbuffer_init()

Initializes all pointers to zero

Algorithm:  initialize pointers to zero 

Returns:    nothing

**                                                                           **
****                                                                       ****
******************************************************************************/
void rbuffer_init(rbuffer_t *in_ptr) {
  rbuffer_clear(in_ptr);
} /* rbuffer_init() */

/******************************************************************************
****                                                                       ****
**                                                                           **
rbuffer_clear()

Reset all pointers to zero

Algorithm:  set pointers to zero 

Returns:    nothing

**                                                                           **
****                                                                       ****
******************************************************************************/
void rbuffer_clear(rbuffer_t *in_ptr) {
  in_ptr->inP = 0;
  in_ptr->outP = 0;
  in_ptr->count = 0;
} /* rbuffer_init() */

/******************************************************************************
****                                                                       ****
**                                                                           **
rbuffer_getchar()

Removes a character (if present) from the
ring buffer and returns it as an int.

Algorithm:  if (char in ring buffer)
              get character from ring buffer
              wrap ring buffer's output ptr if req'd
              decrement ring buffer count
              return character
            else
              return 0  

Notes:     If/when changes to count are not atomic,
             protection is required. As long as it's a
             (16-bit) int, no protection is required.

Returns:    character if a character was present.
            0 if buffer was empty.

**                                                                           **
****                                                                       ****
******************************************************************************/
uint8_t  rbuffer_getchar(rbuffer_t *in_ptr) {
  uint8_t c;

  if (in_ptr->count) {
    c = in_ptr->buff[in_ptr->outP++];

    if (in_ptr->outP > RBUFFER_BUFF_SIZE - 1) {
        in_ptr->outP = 0;
    }
    
    in_ptr->count--;

    return c;
  } else {
    return 0;
  }
} /* rbuffer_getchar() */


/******************************************************************************
****                                                                       ****
**                                                                           **
rbuffer_putchar()

Puts the character into ring buffer
if room is available.

Algorithm:  if (room in ring buffer)
              put char in ring buffer
              wrap ring buffer's input ptr if req'd
              update buffer count
              return c
            else
              return EOF

Notes:     If/when changes to count are not atomic,
             protection is required. As long as it's a
             (16-bit) int, no protection is required.

Returns:    1 if there was room in buffer.
            EOF if buffer was full.

**                                                                           **
****                                                                       ****
******************************************************************************/
uint8_t  rbuffer_putchar(rbuffer_t *in_ptr, uint8_t c){

  if (in_ptr->count < RBUFFER_BUFF_SIZE) {
    in_ptr->buff[in_ptr->inP++] = c;

    if (in_ptr->inP > RBUFFER_BUFF_SIZE - 1) {
        in_ptr->inP = 0;
    }
    
    in_ptr->count++;
    
    return 1;
  } else {
    return 0;
  }

} /* rbuffer_putchar() */


uint16_t  rbuffer_putchars(rbuffer_t *in_ptr, uint8_t *in, uint16_t inLength) {
  int i=0;
  for (i = 0; i < inLength; i++) {
    if (rbuffer_putchar(in_ptr, in[i]) == 0) {
      break;
    }
  }
  return inLength - i;
} /* rbuffer_putchars() */

/******************************************************************************
****                                                                       ****
**                                                                           **
rbuffer_count()

Returns the number of bytes in the ring buffer

Algorithm:  return count

Returns:    count

**                                                                           **
****                                                                       ****
******************************************************************************/
uint16_t rbuffer_count(rbuffer_t *in_ptr) {
  return in_ptr->count;
} /* rbuffer_count() */

/******************************************************************************
****                                                                       ****
**                                                                           **
rbuffer_free()

Returns the number of bytes free in the ring buffer

Algorithm:  return freecount

Returns:    freecount

**                                                                           **
****                                                                       ****
******************************************************************************/
uint16_t rbuffer_free(rbuffer_t *in_ptr) {
  return RBUFFER_BUFF_SIZE - in_ptr->count;
} /* rbuffer_countr() */


/******************************************************************************
****                                                                       ****
**                                                                           **
cmemcpy

Copies src to dest starting at start going until length

Algorithm:  copy data from place to place


**                                                                           **
****                                                                       ****
******************************************************************************/
void cmemcpy(char *dest, char *src, int start, int len)
{
	int i=0;
	for(i=0; i<len;i++)
		dest[start+i]=src[i];
}