package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.AddressableLEDBufferView
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class LEDs : SubsystemBase() {
    val ledStrip =
        AddressableLED(LED_STRIP_PORT).apply { setLength(STRIP_LENGTH) }
    val ledBuffer = AddressableLEDBuffer(STRIP_LENGTH)

    init {
        ledStrip.start()
    }

    private fun setColor(color: LEDPattern) {
        color.applyTo(ledBuffer)
    }

    private fun blink(primary: LEDPattern) {
        (primary.blink(BLINKING_TIME)).applyTo(ledBuffer)
    }

    private fun setPattern(
        primaryPattern: LEDPattern,
        secondaryPattern: LEDPattern,
        section: Array<Int>
    ) {
        val sectionOfBuffer: AddressableLEDBufferView =
            ledBuffer.createView(section[0], section[1])
        secondaryPattern.applyTo(ledBuffer)
        primaryPattern.applyTo(sectionOfBuffer)
        ledStrip.setData(ledBuffer)
    }

    fun intakeLED(): Command = run { blink(INTAKE_COLOR) }
    fun climbLED(): Command = run {
        RAINBOW.scrollAtAbsoluteSpeed(SCROLLING_SPEED, LED_SPACING)
            .applyTo(ledBuffer)
    }

    fun pattern(): Command = run { setPattern(RED, BLUE, arrayOf(5, 15)) }
    fun clearLED(): Command = run { TRANSPARENT.applyTo(ledBuffer) }

    override fun periodic() {
        ledStrip.setData(ledBuffer)
    }
}
