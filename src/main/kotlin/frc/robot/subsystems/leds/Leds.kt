package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Leds(private val io: LedIO) : SubsystemBase() {

    private fun rainbow(): Command = runOnce { io.setRainbow(SCROLLING_SPEED) }
    private fun solid(color: LEDPattern): Command = runOnce {
        io.setLedColor(color)
    }

    private fun blink(color: LEDPattern): Command = runOnce {
        io.setStripBlink(color, BLINKING_TIME)
    }
    private fun setLedPattern(pattern: LEDPattern, section: Array<Int>) {
        io.setLedPattern(pattern, section)
    }

    fun intakeLed(): Command = blink(INTAKE_COLOR)
    fun climbLed(): Command = rainbow()
}
