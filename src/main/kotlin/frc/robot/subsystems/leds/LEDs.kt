package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class LEDs(private val io: LEDsIO) : SubsystemBase() {

    private fun solid(color: LEDPattern): Command = runOnce {
        io.setColor(color)
    }

    private fun blink(color: LEDPattern): Command = runOnce {
        io.blink(color, BLINKING_TIME)
    }
    private fun setLedPattern(pattern: LEDPattern, section: Array<Int>) {
        io.setPattern(pattern, section)
    }

    override fun periodic() {
        ledStrip.setData(ledBuffer)
    }
}
