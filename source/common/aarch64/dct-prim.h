#ifndef __DCT_PRIM_NEON_H__
#define __DCT_PRIM_NEON_H__


#include "common.h"
#include "primitives.h"
#include "contexts.h"   // costCoeffNxN_c
#include "threading.h"  // CLZ

namespace S265_NS
{
// s265 private namespace
void setupDCTPrimitives_neon(EncoderPrimitives &p);
};



#endif

