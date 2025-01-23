package frc.robot.subsystems.leds

import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.AddressableLEDBufferView
import edu.wpi.first.wpilibj.LEDPattern

class LEDsIOReal : LEDsIO {
    val ledStrip =
        AddressableLED(LED_STRIP_PORT).apply { setLength(STRIP_LENGTH) }
    val ledBuffer = AddressableLEDBuffer(STRIP_LENGTH)

    init {
        ledStrip.start()
    }

    override fun setRainbow(scrollingSpeed: LinearVelocity) {
        RAINBOW.scrollAtAbsoluteSpeed(scrollingSpeed, LED_SPACING)
            .applyTo(ledBuffer)
        ledStrip.setData(ledBuffer)
    }

    override fun setColor(color: LEDPattern) {
        color.applyTo(ledBuffer)
        ledStrip.setData(ledBuffer)
    }

    override fun clearStrip() {
        TRANSPARENT.applyTo(ledBuffer)
        ledStrip.setData(ledBuffer)
    }

    override fun blink(primary: LEDPattern, blinkTime: Time) {
        (primary.blink(blinkTime)).applyTo(ledBuffer)
        ledStrip.setData(ledBuffer)
    }

    override fun setPattern(pattern: LEDPattern, section: Array<Int>) {
        val sectionOfBuffer: AddressableLEDBufferView =
            ledBuffer.createView(section[0], section[1])
        pattern.applyTo(sectionOfBuffer)
        ledStrip.setData(ledBuffer)
    }
}
