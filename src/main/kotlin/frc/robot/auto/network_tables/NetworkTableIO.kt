package frc.robot.auto.network_tables

import org.team9432.annotation.Logged

interface NetworkTableIO {
    var inputs: LoggedNetworkTableInputs

    fun updateInputs() {}

    @Logged
    open class NetworkTableInputs {
        var targetBranchPose: Int = 0
        var targetReefPose: Int = 0
    }
}
