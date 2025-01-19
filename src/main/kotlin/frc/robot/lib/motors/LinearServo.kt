package frc.robot.lib.motors

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj.Timer

class LinearServo(channel: Int, length: Int, speed: Int) : Servo(channel) {
    var m_speed: Double
    var m_length: Double
    var setPos = 0.0
    var curPos = 0.0
    var lastTime = 0.0

    /**
     * Miniature Linear Servo Actuators - User Guide (Rev 1) Page 10Table of
     * Contentswcproducts.com Run this method in any periodic function to update
     * the position estimation of your servo
     *
     * @param setpoint the target position of the servo [mm]
     */
    override fun setPosition(setpoint: Double) {
        setPos = MathUtil.clamp(setpoint, 0.0, m_length)
        speed = (setPos / m_length * 2.0) - 1.0
    }

    fun setPosition(setpoint: Distance) {
        setPosition(setpoint.`in`(Units.Millimeters))
    }

    /**
     * Parameters for L16-R Actuonix Linear Actuators
     *
     * @param channel PWM channel used to control the servo
     * @param length max length of the servo [mm]
     * @param speed max speed of the servo [mm/second]
     */
    init {
        setBoundsMicroseconds(2000, 0, 0, 0, 1000)
        m_length = length.toDouble()
        m_speed = speed.toDouble()
    }

    /**
     * Run this method in any periodic function to update the position
     * estimation of your servo
     */
    fun updateCurPos() {
        val dt = Timer.getFPGATimestamp() - lastTime
        if (curPos > setPos + m_speed * dt) {
            curPos -= m_speed * dt
        } else if (curPos < setPos - m_speed * dt) {
            curPos += m_speed * dt
        } else {
            curPos = setPos
        }
    }

    /**
     * Current position of the servo, must be calling
     * [updateCurPos()][.updateCurPos] periodically
     *
     * @return Servo Position [mm]
     */
    override fun getPosition(): Double {
        return curPos
    }

    /**
     * Checks if the servo is at its target position, must be calling
     * [updateCurPos()][.updateCurPos] periodically
     * @return true when servo is at its target
     */
    val isFinished: Boolean
        get() = curPos == setPos
}
