"""
MAVLink Log-Based Fault Tolerance Wrapper
This module provides a drop-in replacement for pymavlink.mavutil.
"""

from .wrapper import MavlinkWrapper, mavlink_wrapper_connection
from . import constants

# The factory function that the mission program will call.
# We alias it to 'mavlink_connection' to match the pymavlink API.
mavlink_connection = mavlink_wrapper_connection

# Re-expose common MAVLink constants/types to avoid broken imports 
# in the mission script when switching from pymavlink.mavutil.
__all__ = [
    "mavlink_connection",
    "MavlinkWrapper",
    "constants"
]