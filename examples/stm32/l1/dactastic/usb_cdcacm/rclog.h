/* 
 * File:   rclog.h
 * Author: karlp
 *
 * Created on August 29, 2013, 3:20 PM
 */

#ifndef RCLOG_H
#define	RCLOG_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif
    
    #define NUM_ENTRIES 64
    #define NUM_ARGUMENTS 2 //  You can't actually change this....

struct rclog_entry_t {
    uint32_t timestamp;
    const char* string;
    uint32_t arguments[NUM_ARGUMENTS]; // no soup for you!
};

struct rclog_obj_t {
        struct rclog_entry_t entries[NUM_ENTRIES];
        int index;
        // This makes it work with zyp's rblog.py unchanged
        int num_arguments;
        int num_entries;
};

void rclog_init(struct rclog_obj_t *o);

void rclog_log(const char *s, uint32_t a0, uint32_t a1);



#ifdef	__cplusplus
}
#endif

#endif	/* RCLOG_H */

