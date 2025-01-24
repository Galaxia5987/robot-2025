package frc.robot.subsystems.leds

import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color

val SCROLLING_SPEED: LinearVelocity = Units.MetersPerSecond.of(0.3)
val BLINKING_TIME: Time = Units.Second.of(2.0)
val LED_SPACING: Distance = Meters.of(1 / 120.0)
const val STRIP_LENGTH = 100
const val LED_STRIP_PORT = 1

val OFF: LEDPattern = LEDPattern.kOff

val INTAKE_COLOR: LEDPattern =
    LEDPattern.solid(Color.kWhiteSmoke).blink(BLINKING_TIME)
val RAINBOW: LEDPattern =
    LEDPattern.rainbow(255, 128)
        .scrollAtAbsoluteSpeed(SCROLLING_SPEED, LED_SPACING)
