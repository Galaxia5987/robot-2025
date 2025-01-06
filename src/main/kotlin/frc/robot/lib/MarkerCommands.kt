package frc.robot.lib

import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

const val ABNORMAL_EVENT_NAME = "PROBLEM!"
val alert = Alert(ABNORMAL_EVENT_NAME, Alert.AlertType.kWarning)

@AutoLogOutput
private var marked = false

private fun markEvent(eventName: String): Command = Commands.runOnce({
    DataLogManager.log(eventName)
    marked = true
}).withTimeout(5.0).andThen(Commands.runOnce({
    marked = false
}))

fun markAbnormalEvent(): Command = markEvent(ABNORMAL_EVENT_NAME)