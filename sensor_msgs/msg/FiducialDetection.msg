# Single fiducial detection from a planar fiducial detector

Header header             # timestamp in the header is the acquisition time
                          # of the first fiducial in the scan.

float32 angle_min         # start angle of the scan [rad]
float32 angle_max         # end angle of the scan [rad]

float32 range_min         # minimum range value [m]
float32 range_max         # maximum range value [m]
float32 range_max_id      # maximum range value to detect id [m]

# standard deviations of position (in spherical coordinate system)
float32 sigma_radial
float32 sigma_polar
float32 sigma_azimuthal

# standard deviations of orientation
float32 sigma_roll
float32 sigma_pitch
float32 sigma_yaw

Fiducial[] fiducials      # detected fiducials
