package frc.robot.lib

import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

const val ABNORMAL_EVENT_NAME = "PROBLEM!"
val alert = Alert(ABNORMAL_EVENT_NAME, Alert.AlertType.kWarning)

fun markEvent(eventName: String): Command = Commands.runOnce({
    DataLogManager.log(eventName)
    alert.set(true)
}).withTimeout(5.0).andThen(Commands.runOnce({
    alert.set(false)
}))

fun markAbnormalEvent(): Command = markEvent(ABNORMAL_EVENT_NAME)