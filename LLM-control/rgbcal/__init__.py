"""Real SO101 environment-camera calibration (laptop built-in RGB camera).

Separate from ``nexus_vision`` on purpose: that package renders a simulated
camera whose intrinsics and pose come from MuJoCo.  Nothing in here copies
a simulated focal length, camera position or table height; every number is
measured from the physical camera and written down with the imaging
configuration it was measured under.
"""
