package frc.robot.auto.network_tables

import edu.wpi.first.networktables.NetworkTableInstance

class NetworkTableIOReal : NetworkTableIO {
    override var inputs: LoggedNetworkTableInputs = LoggedNetworkTableInputs()

    private fun getIntFromTopic(topicName: String): Int =
        NetworkTableInstance.getDefault()
            .getIntegerTopic(topicName)
            .getEntry(0)
            .asLong
            .toInt()

    override fun updateInputs() {
        inputs.targetBranchPose = getIntFromTopic("/Dashboard/TargetBranchPose")
        inputs.targetReefPose = getIntFromTopic("/Dashboard/TargetReefPose")
    }
}
