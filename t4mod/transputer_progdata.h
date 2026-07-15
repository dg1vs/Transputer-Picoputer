#ifndef HELPER_TRANSPUTER_PROGRAMS_H
#define HELPER_TRANSPUTER_PROGRAMS_H

#include <stddef.h>

/*
 * Boot/test program images used by transputer03.c.
 *
 * The arrays are defined in transputer_programs.c.  Do not define them in this
 * header, otherwise every C file including the header would get its own copy.
 */
extern const unsigned char progdata[];
extern const size_t progdata_len;

extern const unsigned char progdata1[];
extern const size_t progdata1_len;

#endif /* HELPER_TRANSPUTER_PROGRAMS_H */
