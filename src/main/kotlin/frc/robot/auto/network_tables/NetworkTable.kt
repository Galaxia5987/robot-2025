package frc.robot.auto.network_tables

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class NetworkTable(private val io: NetworkTableIO) : SubsystemBase() {
    fun getTargetBranchPose(): Int = io.inputs.targetBranchPose

    fun getTargetReefPose(): Int = io.inputs.targetReefPose
    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, io.inputs)
    }
}
