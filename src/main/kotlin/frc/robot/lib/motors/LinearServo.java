package frc.robot.lib.motors;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

public class LinearServo extends Servo {
    double m_speed;
    double m_length;
    double setPos;
    double curPos;

    public LinearServo(int channel, int length, int speed) {
        super(channel);
//        setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        m_length = length;
        m_speed = speed;
    }

    public void setPosition(double setpoint) {
        setPos = MathUtil.clamp(setpoint, 0, m_length);
        setSpeed((setPos / m_length * 2) - 1);
    }

    double lastTime = 0;

    public void updateCurPos() {
        double dt = Timer.getFPGATimestamp() - lastTime;
        if (curPos > setPos + m_speed * dt) {
            curPos -= m_speed * dt;
        } else if (curPos < setPos - m_speed * dt) {
            curPos += m_speed * dt;
        } else {
            curPos = setPos;
        }
    }

    public double getPosition() {
        return curPos;
    }
}
