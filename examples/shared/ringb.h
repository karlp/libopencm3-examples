/* 
 * File:   ringb.h
 * Author: karlp
 *
 * Created on April 18, 2016, 11:54 PM
 */

#ifndef RINGB_H
#define	RINGB_H

#ifdef	__cplusplus
extern "C" {
#endif

	
/*
 * If you're looking for elegant compact performance you've come to the wrong
 * place hombre.
 */

#include <stdbool.h>
#include <stdint.h>

struct ringb {
	volatile int idx_r;
	volatile int idx_w;
	uint8_t *buf;
	int buf_len;
};

/**
 * Load up a ring buffer. Always suceeds
 * @param ring struct saving state, provided by the user
 * @param buf where the data will be kept
 * @param len
 */
void ringb_init(struct ringb *ring, uint8_t *buf, int len);
/**
 * push data in
 * @param ring
 * @param c
 * @return true if space was available
 */
bool ringb_put(struct ringb *ring, uint8_t c);

/**
 * pull data out
 * @param ring
 * @return -1 for no data, uint8_t range for valid.
 */
int ringb_get(struct ringb *ring);

/**
 * Toss data and reset to empty
 * @param ring
 */
void ringb_flush(struct ringb *ring);


#ifdef	__cplusplus
}
#endif

#endif	/* RINGB_H */

