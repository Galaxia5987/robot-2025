package frc.robot.subsystems

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.*
import frc.robot.lib.getTranslation3d
import frc.robot.subsystems.drive.Drive

private val swerveModulePose: Array<Translation2d> =
    Drive.getModuleTranslations()

private val INITIAL_INTAKE_TRANSLATION =
    getTranslation3d(x = Meters.of(-0.32), z = Meters.of(0.35))
private val INITIAL_INTAKE_Roller_TRANSLATION =
    getTranslation3d(x = Meters.of(-0.32 + 0.62890), z = Meters.of(0.35))

private val INITIAL_WRIST_TRANSLATION =
    getTranslation3d(x = Meters.of(0.20715), z = Meters.of(0.995))

private val INITIAL_Elevator_1_TRANSLATION =
    getTranslation3d(x = Meters.of(0.11250), z = Meters.of(0.14345040))

private val INITIAL_Elevator_2_TRANSLATION =
    getTranslation3d(x = Meters.of(0.11250), z = Meters.of(0.20545040))

private val INITIAL_CLIMBER_TRANSLATION =
    getTranslation3d(x = Meters.of(-0.3), z = Meters.of(0.354))
private val kWheelRadius = Meters.of(0.0508)
private val CORAL_ROLLER_UP_C2C: Array<Double> =
    arrayOf(0.201364, 3.3) // arrayOf(C2C Distance, Angle (in rad))

private val WRIST_ANGLE_OFFSET = Degrees.of(90.0)
