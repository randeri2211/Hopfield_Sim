import pytest
import numpy as np
import sys
from pathlib import Path

# Add project root to Python path
sys.path.insert(0, str(Path(__file__).parent.parent))
from api.core.hopfield_control import SwarmHopfieldControl

@pytest.fixture
def sample_controller():
    return SwarmHopfieldControl(robot_positions=[[0,0], [1,1]], speed=0.2, angular_speed=0.1)

def test_velocity_patterns(sample_controller):
    """Test the updated velocity pattern generation with angular velocity encoding"""
    patterns = sample_controller.generate_velocity_patterns()
    
    # Verify pattern dimensions
    assert patterns.shape == (2, 8), "Should have 2 patterns (left/right) x 8 neurons (4 per robot)"
    
    # Test left turn pattern values
    expected_left = np.array([-1, 1, 1, -1] * 2)  # 2 robots
    assert np.array_equal(patterns[0], expected_left), "Left turn pattern mismatch"
    
    # Test right turn pattern values
    expected_right = np.array([1, -1, -1, 1] * 2)  # 2 robots
    assert np.array_equal(patterns[1], expected_right), "Right turn pattern mismatch"

def test_pattern_validation(sample_controller):
    """Test pattern validation logic catches invalid values"""
    with pytest.raises(ValueError):
        invalid_pattern = np.array([1, -1, 0, 1] * 2)  # 0 is invalid
        sample_controller.encode_patterns(invalid_pattern)
