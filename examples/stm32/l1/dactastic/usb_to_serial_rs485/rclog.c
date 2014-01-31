
#include <string.h>
#include <stdint.h>
#include "rclog.h"

static int uber_fake_timestamp = 0;
static struct rclog_obj_t *obj;

void rclog_init(struct rclog_obj_t *o) {
    obj = o;
    memset(o, 0, sizeof(struct rclog_obj_t));
    // For compatibility with stock rblog.py from zyp
    o->num_arguments = NUM_ARGUMENTS;
    o->num_entries = NUM_ENTRIES;
}

void rclog_log(const char *s, uint32_t a0, uint32_t a1) {
    obj->entries[obj->index].timestamp = uber_fake_timestamp++;
    obj->entries[obj->index].string = s;
    obj->entries[obj->index].arguments[0] = a0;
    obj->entries[obj->index].arguments[1] = a1;
    if (++obj->index >= NUM_ENTRIES) {
        obj->index = 0;
    }
}
