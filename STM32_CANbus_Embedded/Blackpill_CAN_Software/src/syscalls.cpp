#include <stddef.h>

extern "C" int _write(int, char *, int len) {
    return len;
}
