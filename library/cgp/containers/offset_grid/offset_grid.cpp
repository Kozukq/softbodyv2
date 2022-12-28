#include "cgp/base/base.hpp"
#include "offset_grid.hpp"
#include "cgp/base/base.hpp"


namespace cgp
{

	std::pair<int, int> index_grid_from_offset(int offset, int N1)
	{
		int const k1 = offset / N1;
		int const k2 = offset - k1 * N1;
		return { k1,k2 };
	}

	int offset_grid(int3 const& k, int N1, int N2)
	{
		return offset_grid(k.x, k.y, k.z, N1, N2);
	}
	int offset_grid(int2 const& k, int N1)
	{
		return offset_grid(k.x, k.y, N1);
	}
}