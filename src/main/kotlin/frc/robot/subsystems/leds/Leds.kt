package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Leds(private val io: LedIO) : SubsystemBase() {

    private fun rainbow(): Command = run { io.setRainbow(SCROLLING_SPEED) }
    private fun solid(color: LEDPattern): Command = run {
        io.setLedColor(color)
    }
    private fun blink(color: LEDPattern): Command = run {
        io.setStripBlink(color, BLINKING_TIME)
    }

    fun intakeLed(): Command = blink(INTAKE_COLOR)
    fun climbLed(): Command = rainbow()
}
