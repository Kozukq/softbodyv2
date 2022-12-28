#pragma once

#include <tuple>
#include <cstddef>

#include "cgp/containers/buffer_stack/buffer_stack.hpp"

namespace cgp
{

	// Compute the 1D offset corresponding to the index (k1,k2) in a 2D grid with static size
	template <int N1> size_t offset_grid_stack(int k1, int k2);
	template <int N1> std::pair<int, int> index_grid_from_offset_stack(int offset);


	inline int offset_grid(int k1, int k2, int N1);
	int offset_grid(int2 const& k, int N1);

	inline int offset_grid(int k0, int k1, int k2, int N0, int N1);
	int offset_grid(int3 const& k, int N0, int N1);
	

	std::pair<int, int> index_grid_from_offset(int offset, int N1);

}

namespace cgp
{

	template <int N1> size_t offset_grid_stack(int k1, int k2)
	{
		return k1 + N1 * k2;
	}

	template <int N1> std::pair<int, int> index_grid_from_offset_stack(int offset)
	{
		int const k1 = offset / N1;
		int const k2 = offset - k1 * N1;
		return { k1,k2 };
	}

	inline int offset_grid(int k1, int k2, int N1)
	{
		return k1 + N1 * k2;
	}
	inline int offset_grid(int k0, int k1, int k2, int N0, int N1)
	{
		return k0 + N0 * (k1 + N1 * k2);
	}
}