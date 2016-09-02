
#include "ringb.h"
void ringb_init(struct ringb *ring, uint8_t *buf, int len)
{
	ring->buf_len = len;
	ring->buf = buf;
	ring->idx_r = 0;
	ring->idx_w = 0;
}

bool ringb_put(struct ringb *ring, uint8_t c) {
	int next = (ring->idx_w + 1) % ring->buf_len;
	if (next != ring->idx_r) {
		ring->buf[ring->idx_w] = c;
		ring->idx_w = next;
		return true;
	}
	return false;
	
}

int ringb_get(struct ringb *ring) {
	if (ringb_empty(ring)) {
		return -1;
	}
	uint8_t rval = ring->buf[ring->idx_r];
	ring->idx_r = (ring->idx_r + 1) % ring->buf_len;
	return rval;
}

int ringb_depth(struct ringb *ring) {
	return ((unsigned int)(ring->buf_len + ring->idx_w - ring->idx_r) % ring->buf_len);
}
