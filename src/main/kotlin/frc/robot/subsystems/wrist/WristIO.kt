package frc.robot.subsystems.wrist

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.*
import org.team9432.annotation.Logged

interface WristIO {
    fun setAngle(angle: Angle) {}
    fun setPower(power: Double) {}
    fun resetAbsoluteEncoder() {}
    fun updateInputs() {}
}