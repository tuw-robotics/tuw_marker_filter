# This expresses a fiducial in a 3D space.
# The message is a submessage of FiducialDetection and is not intended
# to be used separately.

int32 id                    # id of the detected fiducial (0 if id could not be identified)

geometry_msgs/Pose pose     # pose of the detected fiducial