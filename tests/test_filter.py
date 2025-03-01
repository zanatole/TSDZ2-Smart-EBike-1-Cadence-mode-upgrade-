import pytest
from sim._tsdz2 import ffi, lib as ebike # module generated from c-code
import numpy as np
from hypothesis import given, assume, strategies as st


def test_filter():
	filter_result = 0
	new_value = 3
	alpha = 8
	loops = 3
	for _ in range(loops):
		filter_result = ebike.filter(new_value, filter_result, alpha)
		print(filter_result)

	assert filter_result == new_value, f'Expected filter_result {filter_result} == new_value {new_value} after {loops} loops'



# Run the tests
if __name__ == '__main__':
	pytest.main()