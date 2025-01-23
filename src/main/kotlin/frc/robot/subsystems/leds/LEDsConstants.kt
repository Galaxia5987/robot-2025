package frc.robot.subsystems.leds

import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color

val TRANSPARENT: LEDPattern = LEDPattern.kOff
val GRADIENT: LEDPattern =
    LEDPattern.gradient(
        LEDPattern.GradientType.kDiscontinuous,
        Color.kPurple,
        Color.kGreen
    )
val RED: LEDPattern = LEDPattern.solid(Color.kRed)
val BLUE: LEDPattern = LEDPattern.solid(Color.kBlue)
val YELLOW: LEDPattern = LEDPattern.solid(Color.kYellow)
val GREEN: LEDPattern = LEDPattern.solid(Color.kGreen)
val INTAKE_COLOR: LEDPattern = LEDPattern.solid(Color.kWhiteSmoke)
val RAINBOW: LEDPattern = LEDPattern.rainbow(255, 128)
val LED_SPACING: Distance = Meters.of(1 / 120.0)
const val STRIP_LENGTH = 100
val SCROLLING_SPEED: LinearVelocity = Units.MetersPerSecond.of(0.5)
val BLINKING_TIME: Time = Units.Second.one()
const val LED_STRIP_PORT = 1
