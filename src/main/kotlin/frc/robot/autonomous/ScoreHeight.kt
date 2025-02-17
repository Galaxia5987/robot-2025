package frc.robot.autonomous

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.l1
import frc.robot.subsystems.l2
import frc.robot.subsystems.l3
import frc.robot.subsystems.l4

enum class ScoreHeight(val command: (Trigger) -> Command) {
    L1({ trigger -> l1(trigger) }),
    L2({ trigger -> l2(trigger) }),
    L3({ trigger -> l3(trigger) }),
    L4({ trigger -> l4(trigger) });
}
