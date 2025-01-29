package frc.robot.lib.network_tables

import edu.wpi.first.wpilibj2.command.SubsystemBase

class NetworkTable(private val io: NetworkTableIO) : SubsystemBase() {
    fun getTargetBranchPose(): Int = io.inputs.targetBranchPose

    fun getTargetReefPose(): Int = io.inputs.targetReefPose
    override fun periodic() {
        io.updateInputs()
    }
}
