#pragma once
// This is a public header. Avoid references to cuda or other external references.

namespace cuNSearch
{
	typedef unsigned long long ulong;
	typedef unsigned short ushort;
	typedef unsigned int uint;
	typedef unsigned char byte;

    using Real = float;

/*#ifdef CUNSEARCH_USE_DOUBLE_PRECISION
	using Real = double;
#else
	using Real = float;
#endif*/
}