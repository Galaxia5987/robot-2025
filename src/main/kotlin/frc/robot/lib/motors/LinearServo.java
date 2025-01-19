package frc.robot.lib.motors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

public class LinearServo extends Servo {
    double m_speed;
    double m_length;
    double setPos;
    double curPos;

    /**
     * Parameters for L16-R Actuonix Linear Actuators
     *
     * @param channel PWM channel used to control the servo
     * @param length max length of the servo [mm]
     * @param speed max speed of the servo [mm/second]
     */
    public LinearServo(int channel, int length, int speed) {
        super(channel);
        setBoundsMicroseconds(2000000, 1800000, 1500000, 1200000, 1000000);
        m_length = length;
        m_speed = speed;
    }
    
    /**
     Miniature Linear Servo Actuators -
     User Guide (Rev 1)
     Page 10Table of Contentswcproducts.com
     * Run this method in any periodic function to update the position estimation of your
     servo
     *
     * @param setpoint the target position of the servo [mm]
     */
    public void setPosition(double setpoint){
        setPos = MathUtil.clamp(setpoint, 0, m_length);
        setSpeed( (setPos/m_length *2)-1);
    }
    double lastTime = 0;
    /**
     * Run this method in any periodic function to update the position estimation of your
     servo
     */
    public void updateCurPos(){
        double dt = Timer.getFPGATimestamp() - lastTime;
        if (curPos > setPos + m_speed *dt){
            curPos -= m_speed *dt;
        } else if(curPos < setPos - m_speed *dt){
            curPos += m_speed *dt;
        }else{
            curPos = setPos;
        }
    }
    /**
     * Current position of the servo, must be calling {@link #updateCurPos()
    updateCurPos()} periodically
     *
     * @return Servo Position [mm]
     */
    public double getPosition(){
        return curPos;
    }

    /**
     * Checks if the servo is at its target position, must be calling {@link #updateCurPos()
    updateCurPos()} periodically
     * @return true when servo is at its target
     */
    public boolean isFinished(){
        return curPos == setPos;
    }
}