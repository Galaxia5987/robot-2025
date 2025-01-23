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

    private fun setLEDPattern(color: LEDPattern) {
        color.applyTo(ledBuffer)
    }

    private fun blink(primary: LEDPattern) {
        (primary.blink(BLINKING_TIME)).applyTo(ledBuffer)
    }

    private fun setSplitColor(left: LEDPattern, right: LEDPattern) {
        val leftBuffer = ledBuffer.createView(0, STRIP_LENGTH / 2)
        val rightBuffer =
            ledBuffer.createView(STRIP_LENGTH / 2, STRIP_LENGTH - 1)
        left.applyTo(leftBuffer)
        right.applyTo(rightBuffer)
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

    private fun setPattern(primaryPattern: LEDPattern, section: Array<Int>) {
        val sectionOfBuffer: AddressableLEDBufferView =
            ledBuffer.createView(section[0], section[1])
        primaryPattern.applyTo(sectionOfBuffer)
        ledStrip.setData(ledBuffer)
    }
    private fun setPattern(
        primaryPattern: LEDPattern,
    ) {
        primaryPattern.applyTo(ledBuffer)
        ledStrip.setData(ledBuffer)
    }

    private fun clear() = TRANSPARENT.applyTo(ledBuffer)

    fun intakeLED(): Command = run { blink(INTAKE_COLOR) }
    fun climbLED(): Command = run { RAINBOW.applyTo(ledBuffer) }
    fun setPattern(): Command = run { setSplitColor(RED, BLUE) } // don't commit

    override fun periodic() {
        ledStrip.setData(ledBuffer)
    }
}
