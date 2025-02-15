package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.AddressableLEDBufferView
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.gripper

class LEDs : SubsystemBase() {
    private val ledStrip =
        AddressableLED(LED_STRIP_PORT).apply { setLength(STRIP_LENGTH) }
    private val ledBuffer = AddressableLEDBuffer(STRIP_LENGTH)
    private val leftBuffer = ledBuffer.createView(0, STRIP_LENGTH / 2)
    private val rightBuffer =
        ledBuffer.createView(STRIP_LENGTH / 2, STRIP_LENGTH - 1)

    init {
        ledStrip.start()
        defaultCommand =
            setPattern(all = teamPattern).until {
                ledBuffer.getLED(1) != Color.kBlack
            }
    }

    fun setPattern(
        left: LEDPattern? = null,
        right: LEDPattern? = null,
        all: LEDPattern? = null
    ): Command =
        run {
                left?.applyTo(leftBuffer)
                right?.applyTo(rightBuffer)
                all?.applyTo(ledBuffer)
            }
            .ignoringDisable(true)

    fun setPatternArea(
        primaryPattern: LEDPattern,
        secondaryPattern: LEDPattern? = null,
        section: Array<Int>
    ): Command =
        run {
                val sectionOfBuffer: AddressableLEDBufferView =
                    ledBuffer.createView(section[0], section[1])
                secondaryPattern?.applyTo(ledBuffer)
                primaryPattern.applyTo(sectionOfBuffer)
                ledStrip.setData(ledBuffer)
            }
            .ignoringDisable(true)

    override fun periodic() {
        ledStrip.setData(ledBuffer)
    }

    private var climbPattern =
        Trigger {
                DriverStation.getMatchTime() < 5 &&
                    DriverStation.getMatchTime() > -1
            }
            .and(Trigger { DriverStation.isTeleop() })
            .onTrue(
                setPattern(
                    all =
                        LEDPattern.rainbow(255, 128)
                            .scrollAtAbsoluteSpeed(
                                SCROLLING_SPEED_RAINBOW,
                                LED_SPACING
                            )
                            .atBrightness(CLIMBER_PATTERN_BRIGHTNESS)
                )
            )

    private var gripperPattern =
        gripper.hasCoral
            .and(climbPattern.negate())
            .onTrue(
                setPattern(
                    all =
                        LEDPattern.solid(Color.kWhiteSmoke)
                            .blink(BLINKING_ON_TIME, BLINKING_OFF_TIME)
                            .atBrightness(GRIPPER_PATTERN_BRIGHTNESS)
                )
            )

    private var defaultPattern =
        climbPattern
            .or(gripper.hasCoral)
            .onFalse((setPattern(all = teamPattern)))
}
