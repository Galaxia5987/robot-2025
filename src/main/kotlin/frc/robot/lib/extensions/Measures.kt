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


// Factories

// Helper function for conversion
inline fun <N : Number, R> N.toUnit(converter: (Double) -> R) = converter(this.toDouble())

// Distance
val Number.m: Distance get() = toUnit(Units.Meters::of)
val Number.cm: Distance get() = toUnit(Units.Centimeters::of)
val Number.mm: Distance get() = toUnit(Units.Millimeters::of)

// Linear velocity
val Number.mps: LinearVelocity get() = toUnit(Units.MetersPerSecond::of)

// Angle
val Number.deg: Angle get() = toUnit(Units.Degrees::of)
val Number.rotations: Angle get() = toUnit(Units.Rotations::of)
val Number.rad: Angle get() = toUnit(Units.Radians::of)

// Angular velocity
val Number.deg_ps: AngularVelocity get() = toUnit(Units.DegreesPerSecond::of)
val Number.rotations_ps: AngularVelocity get() = toUnit(Units.RotationsPerSecond::of)
val Number.rad_ps: AngularVelocity get() = toUnit(Units.RadiansPerSecond::of)

// Other
val Number.sec: Time get() = toUnit(Units.Seconds::of)
val Number.percent: Dimensionless get() = toUnit(Units.Percent::of)
val Number.amps: Current get() = toUnit(Units.Amps::of)
val Number.volts: Voltage get() = toUnit(Units.Volts::of)
val Number.kg2m: MomentOfInertia get() = toUnit(Units.KilogramSquareMeters::of)
