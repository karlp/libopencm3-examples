
#include "ringb.h"
void ringb_init(struct ringb *ring, uint8_t *buf, int len)
{
	ring->buf_len = len;
	ring->buf = buf;
	ring->depth = 0;
	ring->idx_r = 0;
	ring->idx_w = 0;
}

bool ringb_put(struct ringb *ring, uint8_t c) {
	if (ring->depth < ring->buf_len) {
		ring->buf[ring->idx_w++] = c;
		if (ring->idx_w >= ring->buf_len) {
			ring->idx_w = 0;
		}
		ring->depth++;
		return true;
	}
	return false;
	
}

int ringb_get(struct ringb *ring) {
	int rval;
	if (ring->depth) {
		rval = ring->buf[ring->idx_r++];
		if (ring->idx_r >= ring->buf_len) {
			ring->idx_r = 0;
		}
		ring->depth--;
		return rval;
	}
	return -1;
}

void ringb_flush(struct ringb *ring) {
	ring->idx_r = 0;
	ring->idx_w = 0;
	ring->depth = 0;
}