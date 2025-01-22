package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Leds(private val io: LedIO) : SubsystemBase() {

    private fun rainBow(): Command = run { io.setRainbow(SCROLLING_SPEED) }
    private fun solid(color: LEDPattern): Command = run { io.setLedStrip(color) }
    private fun blink(color: LEDPattern): Command = run { io.setLedStripBlink(color, BLINKING_TIME) }

    fun intakeLed(): Command = blink(INTAKE_COLOR)
    fun climbLed(): Command = rainBow()
}
