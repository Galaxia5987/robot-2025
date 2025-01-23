package frc.robot.subsystems.leds

import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.LEDPattern

interface LedIO {
    fun setLedColor(color: LEDPattern) {}
    fun clearLedsStrip() {}
    fun setLedColorBlink(primary: LEDPattern, blinkTime: Time) {}
    fun setRainbow(scrollingSpeed: LinearVelocity) {}
}
