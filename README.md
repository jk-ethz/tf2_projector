# tf2_projector

This package provides a node that projects the transform of a given tf2 frame (source) to another tf2 frame (target) under the same root, using an arbitrary (transitive) parent of the target frame as an attachment frame for the projection.

This is useful to project the output of a localization framework to the robot, without the need of parenting the robot to the exact frame that the localization framework uses. This allows also for arbitrary chains of odometry frames to be inbetween the robot and the root frame.
