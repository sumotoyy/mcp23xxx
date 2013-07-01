#ifndef _maxBitUtility_H
#define _maxBitUtility_H 

/* a=target variable, b=bit number to act upon 0-n */
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1<<(b)))
#define BIT_CHECK(a,b) ((a) & (1<<(b)))

/* x=target variable, y=mask */
/* #define BITMASK_SET(x,y) ((x) |= (y))
#define BITMASK_CLEAR(x,y) ((x) &= (~(y)))
#define BITMASK_FLIP(x,y) ((x) ^= (y))
#define BITMASK_CHECK(x,y) ((x) & (y)) */
/*
#define BITS 8
#define BIT_SET(  p, n) (p[(n)/BITS] |=  (0x80>>((n)%BITS)))
#define BIT_CLEAR(p, n) (p[(n)/BITS] &= ~(0x80>>((n)%BITS)))
#define BIT_ISSET(p, n) (p[(n)/BITS] &   (0x80>>((n)%BITS)))
*/
#endif