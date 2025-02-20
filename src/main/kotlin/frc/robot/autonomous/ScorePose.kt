package frc.robot.autonomous

import edu.wpi.first.math.geometry.Pose2d

enum class ScorePose(val pose: () -> Pose2d) {
    `1L`({ ALIGNMENT_POSES["L1"]!! }),
    `1R`({ ALIGNMENT_POSES["R1"]!! }),
    `2L`({ ALIGNMENT_POSES["L2"]!! }),
    `2R`({ ALIGNMENT_POSES["R2"]!! }),
    `3L`({ ALIGNMENT_POSES["L3"]!! }),
    `3R`({ ALIGNMENT_POSES["R3"]!! }),
    `4L`({ ALIGNMENT_POSES["L4"]!! }),
    `4R`({ ALIGNMENT_POSES["R4"]!! }),
    `5L`({ ALIGNMENT_POSES["L5"]!! }),
    `5R`({ ALIGNMENT_POSES["R5"]!! }),
    `6L`({ ALIGNMENT_POSES["L6"]!! }),
    `6R`({ ALIGNMENT_POSES["R6"]!! }),
    S1({ ALIGNMENT_POSES["S1"]!! }),
    S2({ ALIGNMENT_POSES["S2"]!! })
}
