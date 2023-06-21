/* See COPYING file for copyright and license details. */

#include "ebur128.hpp"

void ebur128_get_version(int* major, int* minor, int* patch)
{
    *major = EBUR128_VERSION_MAJOR;
    *minor = EBUR128_VERSION_MINOR;
    *patch = EBUR128_VERSION_PATCH;
}
