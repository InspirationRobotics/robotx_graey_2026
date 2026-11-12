"""Canonical MAVLink identities for GRAEY.

The Cube and its onboard control components share vehicle system 3.  The
competition radio application is a separate companion endpoint at 43/201.
Keep this file dependency-free so every runtime, tool and simulator can import
the same identities before pymavlink or ROS is loaded.
"""

FCU_SYSTEM = 3
FCU_COMPONENT = 1
COMPANION_SYSTEM = 43
COMPANION_COMPONENT = 201

