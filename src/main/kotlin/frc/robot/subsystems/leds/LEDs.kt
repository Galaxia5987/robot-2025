package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.AddressableLEDBufferView
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.IS_RED
import frc.robot.climber
import frc.robot.gripper

class LEDs : SubsystemBase() {
    private val ledStrip =
        AddressableLED(LED_STRIP_PORT).apply { setLength(STRIP_LENGTH) }
    private val ledBuffer = AddressableLEDBuffer(STRIP_LENGTH)
    private val leftBuffer = ledBuffer.createView(0, STRIP_LENGTH / 2)
    private val rightBuffer =
        ledBuffer.createView(STRIP_LENGTH / 2, STRIP_LENGTH - 1)

    val teamPattern: LEDPattern =
        when (IS_RED) {
            false ->
                LEDPattern.gradient(
                        LEDPattern.GradientType.kDiscontinuous,
                        Color.kAqua,
                        Color.kBlue
                    )
                    .scrollAtAbsoluteSpeed(
                        SCROLLING_SPEED_TEAM_PATTERN,
                        LED_SPACING
                    )
            true ->
                LEDPattern.gradient(
                        LEDPattern.GradientType.kDiscontinuous,
                        Color(255, 0, 148),
                        Color.kRed
                    )
                    .scrollAtAbsoluteSpeed(
                        SCROLLING_SPEED_TEAM_PATTERN,
                        LED_SPACING
                    )
        }

    init {
        ledStrip.start()
    }

    fun setPattern(
        left: LEDPattern? = null,
        right: LEDPattern? = null,
        all: LEDPattern? = null
    ): Command = run {
        left?.applyTo(leftBuffer)
        right?.applyTo(rightBuffer)
        all?.applyTo(ledBuffer)
    }

    fun setPatternArea(
        primaryPattern: LEDPattern,
        secondaryPattern: LEDPattern? = null,
        section: Array<Int>
    ): Command = run {
        val sectionOfBuffer: AddressableLEDBufferView =
            ledBuffer.createView(section[0], section[1])
        secondaryPattern?.applyTo(ledBuffer)
        primaryPattern.applyTo(sectionOfBuffer)
        ledStrip.setData(ledBuffer)
    }

    override fun periodic() {
        ledStrip.setData(ledBuffer)
    }

    private var runPattern: Trigger =
        climber.isClimbed
            .whileTrue(
                setPattern(
                    all =
                        LEDPattern.rainbow(255, 128)
                            .scrollAtAbsoluteSpeed(
                                SCROLLING_SPEED_RAINBOW,
                                LED_SPACING
                            )
                )
            )
            .or(
                gripper.hasCoral.whileTrue(
                    setPattern(all = LEDPattern.solid(Color.kWhiteSmoke))
                )
            )
            .negate()
            .whileTrue(setPattern(all = teamPattern))
}
