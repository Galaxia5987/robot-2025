package frc.robot.subsystems.leds

import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.LEDPattern

interface LEDsIO {
    fun setColor(color: LEDPattern) {}
    fun clearStrip() {}
    fun blink(primary: LEDPattern, blinkTime: Time) {}
    fun setRainbow(scrollingSpeed: LinearVelocity) {}
    fun setPattern(pattern: LEDPattern, section: Array<Int>) {}
}
