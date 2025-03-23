package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.IS_RED
import frc.robot.autonomous.ableToNet
import frc.robot.autonomous.isAligning
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

    private val isEndGame =
        Trigger {
                DriverStation.getMatchTime() < 20 &&
                    DriverStation.getMatchTime() > -1
            }
            .and(RobotModeTriggers.teleop())
            .onTrue(setPattern(all = climbPattern))

    private val gripperTrigger: Trigger =
        gripper.hasCoral
            .and(isEndGame.negate())
            .onTrue(setPattern(all = gripperPattern))

    private val netLEDs: Trigger =
        ableToNet
            .and(isAligning.negate())
            .onTrue(setPattern(all = ableToNetPattern))
            .onFalse(setPattern(all = redTeamPattern))

    val blueDefaultPattern: Trigger =
        isEndGame
            .or(gripper.hasCoral)
            .or { IS_RED }
            .or(isAligning)
            .onFalse(setPattern(all = blueTeamPattern))

    val redDefaultPattern: Trigger =
        isEndGame
            .or(gripper.hasCoral)
            .or { !IS_RED }
            .or(isAligning)
            .onFalse(setPattern(all = redTeamPattern))
}
