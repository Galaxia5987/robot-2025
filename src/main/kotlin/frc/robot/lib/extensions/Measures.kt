package frc.robot.lib.extensions

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import kotlin.math.PI

fun LinearVelocity.toAngular(
    radius: Distance,
    gearRatio: Double,
): AngularVelocity =
    this.timesConversionFactor(
        Units.RotationsPerSecond.per(Units.MetersPerSecond)
            .of(1.0 / (radius.`in`(Units.Meters) * gearRatio * 2.0 * PI))
    )

fun Distance.toAngle(radius: Distance, gearRatio: Double): Angle =
    this.timesConversionFactor(
        Units.Rotations.per(Units.Meters)
            .of(1.0 / (radius.`in`(Units.Meters) * gearRatio * 2.0 * PI))
    )

fun Angle.toDistance(radius: Distance, gearRatio: Double): Distance =
    this.timesConversionFactor(
        Units.Meters.per(Units.Rotations)
            .of(radius.`in`(Units.Meters) * gearRatio * 2.0 * PI)
    )

fun AngularVelocity.toLinear(
    radius: Distance,
    gearRatio: Double,
): LinearVelocity =
    this.timesConversionFactor(
        Units.MetersPerSecond.per(Units.RotationsPerSecond)
            .of(radius.`in`(Units.Meters) * gearRatio * 2.0 * PI)
    )


operator fun Distance.div(time: TimeUnit): LinearVelocity = this / time.one()
operator fun Distance.div(divisor: Number): Distance = this / divisor.toDouble()
operator fun Voltage.div(time: TimeUnit): Velocity<VoltageUnit> = this / time.one()
