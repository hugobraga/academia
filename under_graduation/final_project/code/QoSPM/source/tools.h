/* 
 * File:   tools.h
 * Author: hugo
 *
 * Created on 15 de Março de 2009, 19:08
 */

#ifndef _TOOLS_H
#define	_TOOLS_H

#include <string.h>

#define MAX(a, b) ((a) > (b) ? (a) : (b));

#define MARSHALLING(buffer_ptr, data_ptr, size) \
        if (size) memcpy(buffer_ptr, data_ptr, size);\
        else memcpy(buffer_ptr, data_ptr, sizeof(*data_ptr));

#define UNMARSHALLING(data_ptr, buffer_ptr, size)\
        if (size) {\
            memset(data_ptr, 0, size);\
            memcpy(data_ptr, buffer_ptr, size);\
        } else {\
            memset(data_ptr, 0, sizeof(*data_ptr));\
            memcpy(data_ptr, buffer_ptr, sizeof(*data_ptr));}

#endif	/* _TOOLS_H */

