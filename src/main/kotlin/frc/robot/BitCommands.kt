package frc.robot

import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.feeder
import frc.robot.subsystems.intake.intakeAlgae
import frc.robot.subsystems.l4

fun intakeBit(outtakeTrigger: Trigger): Command =
    sequence(
            intakeAlgae().until(outtakeTrigger),
            roller.outtake().withTimeout(Seconds.of(0.8)),
            extender.retract().withTimeout(0.5)
        )
        .withName("Bits/Intake")

fun feederL4Bit(outtakeTrigger: Trigger): Command =
    sequence(feeder(Trigger { true }), l4(outtakeTrigger))
        .withName("Bits/Feeder-->L4")
