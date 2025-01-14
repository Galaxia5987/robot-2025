package frc.robot.subsystems.drive

object SwervePorts {
    const val FRONT_LEFT_DRIVE_MOTOR_ID: Int = 2
    const val FRONT_LEFT_ANGLE_MOTOR_ID: Int = 1
    const val FRONT_RIGHT_DRIVE_MOTOR_ID: Int = 4
    const val FRONT_RIGHT_ANGLE_MOTOR_ID: Int = 3
    const val REAR_LEFT_DRIVE_MOTOR_ID: Int = 6
    const val REAR_LEFT_ANGLE_MOTOR_ID: Int = 5
    const val REAR_RIGHT_DRIVE_MOTOR_ID: Int = 8
    const val REAR_RIGHT_ANGLE_MOTOR_ID: Int = 7

    val DRIVE_IDS: IntArray =
        intArrayOf(
            FRONT_LEFT_DRIVE_MOTOR_ID,
            FRONT_RIGHT_DRIVE_MOTOR_ID,
            REAR_LEFT_DRIVE_MOTOR_ID,
            REAR_RIGHT_DRIVE_MOTOR_ID
        )
    val ANGLE_IDS: IntArray =
        intArrayOf(
            FRONT_LEFT_ANGLE_MOTOR_ID,
            FRONT_RIGHT_ANGLE_MOTOR_ID,
            REAR_LEFT_ANGLE_MOTOR_ID,
            REAR_RIGHT_ANGLE_MOTOR_ID
        )

    const val FRONT_LEFT_ENCODER_ID: Int = 10
    const val FRONT_RIGHT_ENCODER_ID: Int = 20
    const val REAR_LEFT_ENCODER_ID: Int = 30
    const val REAR_RIGHT_ENCODER_ID: Int = 40

    val ENCODER_IDS: IntArray =
        intArrayOf(
            FRONT_LEFT_ENCODER_ID,
            FRONT_RIGHT_ENCODER_ID,
            REAR_LEFT_ENCODER_ID,
            REAR_RIGHT_ENCODER_ID
        )

    var FRONT_LEFT_DRIVE_INVERTED: Boolean = true
    var FRONT_LEFT_ANGLE_INVERTED: Boolean = true
    var FRONT_RIGHT_DRIVE_INVERTED: Boolean = true
    var FRONT_RIGHT_ANGLE_INVERTED: Boolean = true
    var REAR_LEFT_DRIVE_INVERTED: Boolean = true
    var REAR_LEFT_ANGLE_INVERTED: Boolean = true
    var REAR_RIGHT_DRIVE_INVERTED: Boolean = true
    val DRIVE_INVERTED: BooleanArray =
        booleanArrayOf(
            FRONT_LEFT_DRIVE_INVERTED,
            FRONT_RIGHT_DRIVE_INVERTED,
            REAR_LEFT_DRIVE_INVERTED,
            REAR_RIGHT_DRIVE_INVERTED
        )
    var REAR_RIGHT_ANGLE_INVERTED: Boolean = true
    val ANGLE_INVERTED: BooleanArray =
        booleanArrayOf(
            FRONT_LEFT_ANGLE_INVERTED,
            FRONT_RIGHT_ANGLE_INVERTED,
            REAR_LEFT_ANGLE_INVERTED,
            REAR_RIGHT_ANGLE_INVERTED
        )
}

object AltSwervePorts {
    const val FRONT_LEFT_DRIVE_MOTOR_ID: Int = 2
    const val FRONT_LEFT_ANGLE_MOTOR_ID: Int = 1
    const val FRONT_RIGHT_DRIVE_MOTOR_ID: Int = 4
    const val FRONT_RIGHT_ANGLE_MOTOR_ID: Int = 3
    const val REAR_LEFT_DRIVE_MOTOR_ID: Int = 6
    const val REAR_LEFT_ANGLE_MOTOR_ID: Int = 5
    const val REAR_RIGHT_DRIVE_MOTOR_ID: Int = 8
    const val REAR_RIGHT_ANGLE_MOTOR_ID: Int = 7

    val DRIVE_IDS: IntArray =
        intArrayOf(
            FRONT_LEFT_DRIVE_MOTOR_ID,
            FRONT_RIGHT_DRIVE_MOTOR_ID,
            REAR_LEFT_DRIVE_MOTOR_ID,
            REAR_RIGHT_DRIVE_MOTOR_ID
        )
    val ANGLE_IDS: IntArray =
        intArrayOf(
            FRONT_LEFT_ANGLE_MOTOR_ID,
            FRONT_RIGHT_ANGLE_MOTOR_ID,
            REAR_LEFT_ANGLE_MOTOR_ID,
            REAR_RIGHT_ANGLE_MOTOR_ID
        )

    const val FRONT_LEFT_ENCODER_ID: Int = 10
    const val FRONT_RIGHT_ENCODER_ID: Int = 20
    const val REAR_LEFT_ENCODER_ID: Int = 30
    const val REAR_RIGHT_ENCODER_ID: Int = 40

    val ENCODER_IDS: IntArray =
        intArrayOf(
            FRONT_LEFT_ENCODER_ID,
            FRONT_RIGHT_ENCODER_ID,
            REAR_LEFT_ENCODER_ID,
            REAR_RIGHT_ENCODER_ID
        )

    var FRONT_LEFT_DRIVE_INVERTED: Boolean = true
    var FRONT_LEFT_ANGLE_INVERTED: Boolean = true
    var FRONT_RIGHT_DRIVE_INVERTED: Boolean = true
    var FRONT_RIGHT_ANGLE_INVERTED: Boolean = true
    var REAR_LEFT_DRIVE_INVERTED: Boolean = true
    var REAR_LEFT_ANGLE_INVERTED: Boolean = true
    var REAR_RIGHT_DRIVE_INVERTED: Boolean = true
    val DRIVE_INVERTED: BooleanArray =
        booleanArrayOf(
            FRONT_LEFT_DRIVE_INVERTED,
            FRONT_RIGHT_DRIVE_INVERTED,
            REAR_LEFT_DRIVE_INVERTED,
            REAR_RIGHT_DRIVE_INVERTED
        )
    var REAR_RIGHT_ANGLE_INVERTED: Boolean = true
    val ANGLE_INVERTED: BooleanArray =
        booleanArrayOf(
            FRONT_LEFT_ANGLE_INVERTED,
            FRONT_RIGHT_ANGLE_INVERTED,
            REAR_LEFT_ANGLE_INVERTED,
            REAR_RIGHT_ANGLE_INVERTED
        )
}
