package frc.robot.subsystems.leds

import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.LEDPattern

class LedIOReal : LedIO {
    val ledStrip = AddressableLED(1).apply { setLength(STRIP_LENGTH) }
    val ledBuffer = AddressableLEDBuffer(STRIP_LENGTH)

    init {
        ledStrip.start()
    }

    override fun setRainbow(scrollingSpeed: LinearVelocity) {
        RAINBOW.scrollAtAbsoluteSpeed(scrollingSpeed, LED_SPACING)
            .applyTo(ledBuffer)
        ledStrip.setData(ledBuffer)
    }

    override fun setLedColor(color: LEDPattern) {
        color.applyTo(ledBuffer)
        ledStrip.setData(ledBuffer)
    }

    override fun clearLedsStrip() {
        TRANSPARENT.applyTo(ledBuffer)
        ledStrip.setData(ledBuffer)
    }

    override fun setStripBlink(primary: LEDPattern, blinkTime: Time) {
        (primary.blink(blinkTime)).applyTo(ledBuffer)
        ledStrip.setData(ledBuffer)
    }
}
