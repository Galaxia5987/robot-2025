package frc.robot.lib.network_tables

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

class NetworkTableIOSim : NetworkTableIO {
     override var inputs: LoggedNetworkTableInputs = LoggedNetworkTableInputs()

     private val targetBranchPose = LoggedNetworkNumber("Tuning/Dashboard/TargetBranchPose")
     private val targetReefPose = LoggedNetworkNumber("Tuning/Dashboard/TargetReefPose")

     override fun updateInputs() {
        inputs.targetBranchPose = targetBranchPose.get().toInt()
        inputs.targetReefPose = targetReefPose.get().toInt()
     }
 }
