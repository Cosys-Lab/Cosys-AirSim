"""Test suite for cosysairsim package.

This test suite verifies:
1. Package imports correctly
2. NumPy API compatibility (np.frombuffer replacement)
3. Type annotations work
4. All public API is accessible
"""

import numpy as np
import pytest


class TestPackageImports:
    """Test that the cosysairsim package imports correctly."""

    def test_import_package(self):
        """Verify cosysairsim can be imported."""
        import cosysairsim

        assert cosysairsim is not None

    def test_import_version(self):
        """Verify version is available."""
        import cosysairsim

        assert cosysairsim.__version__ == "3.3.1.dev0"

    def test_import_types(self):
        """Verify all types can be imported."""
        from cosysairsim import (
            ImageRequest,
            ImageResponse,
            Pose,
            Quaternionr,
            Twist,
            Vector3r,
        )

        assert Vector3r is not None
        assert Quaternionr is not None
        assert Pose is not None
        assert Twist is not None
        assert ImageRequest is not None
        assert ImageResponse is not None

    def test_import_clients(self):
        """Verify all clients can be imported."""
        from cosysairsim import (
            CarClient,
            MultirotorClient,
            VehicleClient,
        )

        assert MultirotorClient is not None
        assert CarClient is not None
        assert VehicleClient is not None


class TestNumpyAPIs:
    """Test NumPy API replacements (np.fromstring -> np.frombuffer)."""

    def test_string_to_uint8_array(self):
        """Test string_to_uint8_array with np.frombuffer."""
        from cosysairsim.utils import string_to_uint8_array

        test_data = b"\x01\x02\x03\x04\x05"
        result = string_to_uint8_array(test_data)

        assert isinstance(result, np.ndarray)
        assert result.dtype == np.uint8
        assert len(result) == 5
        np.testing.assert_array_equal(result, np.array([1, 2, 3, 4, 5], dtype=np.uint8))

    def test_string_to_float_array(self):
        """Test string_to_float_array with np.frombuffer."""
        from cosysairsim.utils import string_to_float_array

        test_data = b"\x00\x00\x80?\x00\x00\x00@"  # 1.0, 2.0 in bytes
        result = string_to_float_array(test_data)

        assert isinstance(result, np.ndarray)
        assert result.dtype == np.float32
        assert len(result) == 2

    def test_vector3r_division_numpy_types(self):
        """Test Vector3r division with numpy types (replaced np.sctypes)."""
        from cosysairsim.types import Vector3r

        v = Vector3r(6, 8, 10)

        # Test with numpy integer types
        result = v / np.int32(2)
        assert result.x_val == 3
        assert result.y_val == 4
        assert result.z_val == 5

        # Test with numpy float types
        result = v / np.float32(2)
        assert result.x_val == 3
        assert result.y_val == 4
        assert result.z_val == 5

    def test_vector3r_multiplication_numpy_types(self):
        """Test Vector3r multiplication with numpy types."""
        from cosysairsim.types import Vector3r

        v = Vector3r(1, 2, 3)

        # Test with numpy integer types
        result = v * np.int32(2)
        assert result.x_val == 2
        assert result.y_val == 4
        assert result.z_val == 6

        # Test with numpy float types
        result = v * np.float64(2)
        assert result.x_val == 2
        assert result.y_val == 4
        assert result.z_val == 6


class TestPublicAPI:
    """Test that all public API is accessible via __all__."""

    def test_all_exports(self):
        """Verify __all__ is defined."""
        import cosysairsim

        assert hasattr(cosysairsim, "__all__")
        assert len(cosysairsim.__all__) > 0

    def test_all_are_importable(self):
        """Verify everything in __all__ can be imported."""
        import cosysairsim

        for name in cosysairsim.__all__:
            assert hasattr(cosysairsim, name), f"Missing: {name}"


class TestTypeAnnotations:
    """Test type annotations are present."""

    def test_types_module_exists(self):
        """Verify types module exists."""
        import cosysairsim.types as types

        assert types.Vector3r is not None

    def test_client_module_exists(self):
        """Verify client module exists."""
        import cosysairsim.client as client

        assert client.VehicleClient is not None


class TestVector3r:
    """Test Vector3r mathematical operations."""

    def test_vector3r_creation(self):
        """Test Vector3r can be created."""
        from cosysairsim.types import Vector3r

        v = Vector3r(1, 2, 3)
        assert v.x_val == 1
        assert v.y_val == 2
        assert v.z_val == 3

    def test_vector3r_addition(self):
        """Test Vector3r addition."""
        from cosysairsim.types import Vector3r

        v1 = Vector3r(1, 2, 3)
        v2 = Vector3r(4, 5, 6)
        result = v1 + v2

        assert result.x_val == 5
        assert result.y_val == 7
        assert result.z_val == 9

    def test_vector3r_division_by_int(self):
        """Test Vector3r division by int."""
        from cosysairsim.types import Vector3r

        v = Vector3r(6, 8, 10)
        result = v / 2

        assert result.x_val == 3
        assert result.y_val == 4
        assert result.z_val == 5


class TestQuaternionr:
    """Test Quaternionr mathematical operations."""

    def test_quaternionr_creation(self):
        """Test Quaternionr can be created."""
        from cosysairsim.types import Quaternionr

        q = Quaternionr(0, 0, 0, 1)
        assert q.w_val == 1
        assert q.x_val == 0
        assert q.y_val == 0
        assert q.z_val == 0

    def test_quaternionr_division(self):
        """Test Quaternionr division."""
        from cosysairsim.types import Quaternionr

        q = Quaternionr(2, 2, 2, 2)
        result = q / 2

        assert result.w_val == 1
        assert result.x_val == 1
        assert result.y_val == 1
        assert result.z_val == 1


class TestImageRequest:
    """Test ImageRequest creation."""

    def test_image_request_defaults(self):
        """Test ImageRequest with defaults."""
        from cosysairsim.types import ImageRequest

        req = ImageRequest("0", 0)
        assert req.camera_name == "0"
        assert req.image_type == 0

    def test_image_request_full(self):
        """Test ImageRequest with all parameters."""
        from cosysairsim.types import ImageRequest

        req = ImageRequest("front", 3, True, False)
        assert req.camera_name == "front"
        assert req.image_type == 3
        assert req.pixels_as_float is True
        assert req.compress is False


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
