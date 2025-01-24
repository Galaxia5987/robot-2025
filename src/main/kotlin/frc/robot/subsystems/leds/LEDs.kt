package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.AddressableLEDBufferView
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class LEDs : SubsystemBase() {
    private val ledStrip =
        AddressableLED(LED_STRIP_PORT).apply { setLength(STRIP_LENGTH) }
    private val ledBuffer = AddressableLEDBuffer(STRIP_LENGTH)

    init {
        ledStrip.start()
    }

    private fun setLEDPattern(pattern: LEDPattern) {
        pattern.applyTo(ledBuffer)
    }

    private fun setSplitColor(left: LEDPattern, right: LEDPattern) {
        val leftBuffer = ledBuffer.createView(0, STRIP_LENGTH / 2)
        val rightBuffer =
            ledBuffer.createView(STRIP_LENGTH / 2, STRIP_LENGTH - 1)
        left.applyTo(leftBuffer)
        right.applyTo(rightBuffer)
    }
    private fun setPatternArea(
        primaryPattern: LEDPattern,
        secondaryPattern: LEDPattern? = null,
        section: Array<Int>
    ) {
        val sectionOfBuffer: AddressableLEDBufferView =
            ledBuffer.createView(section[0], section[1])
        secondaryPattern?.applyTo(ledBuffer)
        primaryPattern.applyTo(sectionOfBuffer)
        ledStrip.setData(ledBuffer)
    }

    private fun clear() = TRANSPARENT.applyTo(ledBuffer)
    private fun colorBlink(
        primaryColor: LEDPattern,
        secondaryPattern: LEDPattern
    ) {
        primaryColor.blink(BLINKING_TIME).applyTo(ledBuffer)
        if (ledBuffer.getLED(0).toHexString().equals("#000000")) {
            secondaryPattern.applyTo(ledBuffer)
        }
    }

    fun intakeLED(): Command = run { setLEDPattern(INTAKE_COLOR) }
    fun climbLED(): Command = run { RAINBOW.applyTo(ledBuffer) }
    override fun periodic() {
        ledStrip.setData(ledBuffer)
    }
}
