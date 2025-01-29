package frc.robot.auto.network_tables

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

class NetworkTableIOSim : NetworkTableIO {
    override var inputs: LoggedNetworkTableInputs = LoggedNetworkTableInputs()

    private val targetBranchPose =
        LoggedNetworkNumber("/Tuning/Dashboard/TargetBranchPose", 0.0)
    private val targetReefPose =
        LoggedNetworkNumber("/Tuning/Dashboard/TargetReefPose", 0.0)

    override fun updateInputs() {
        inputs.targetBranchPose = targetBranchPose.get().toInt()
        inputs.targetReefPose = targetReefPose.get().toInt()
    }
}
