package frc.robot.lib.motors;

import com.ctre.phoenix6.controls.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.lib.math.differential.Derivative;
import frc.robot.lib.units.Units;

public class TalonFXSim extends SimMotor {

    private final Derivative acceleration = new Derivative();

    public TalonFXSim(
            LinearSystem<N2, N1, N2> model,
            int numMotors,
            double gearing,
            double conversionFactor) {
        super(model, DCMotor.getFalcon500(numMotors), gearing, conversionFactor);
    }

    public TalonFXSim(
            DCMotor motor, double gearing, double jKgMetersSquared, double conversionFactor) {
        super(motor, jKgMetersSquared, gearing, conversionFactor);
    }

    public TalonFXSim(
            int numMotors, double gearing, double jKgMetersSquared, double conversionFactor) {
        super(DCMotor.getFalcon500(numMotors), jKgMetersSquared, gearing, conversionFactor);
    }

    @Override
    public void update(double timestampSeconds) {
        super.update(timestampSeconds);

        acceleration.update(getVelocity(), timestampSeconds);
    }

    public void setControl(DutyCycleOut request) {
        setControl(new VoltageOut(request.Output * 12));
    }

    public void setControl(VoltageOut request) {
        voltageRequest = MotorSetpoint.simpleVoltage(request.Output);
    }

    public void setControl(PositionDutyCycle request) {
        setControl(new PositionVoltage(request.Position).withFeedForward(request.FeedForward * 12));
    }

    public void setControl(PositionVoltage request) {
        voltageRequest =
                () -> controller.calculate(getPosition(), request.Position) + request.FeedForward;
    }

    public void setControl(VelocityDutyCycle request) {
        setControl(new VelocityVoltage(request.Velocity).withFeedForward(request.FeedForward * 12));
    }

    public void setControl(VelocityVoltage request) {
        voltageRequest =
                () -> controller.calculate(getVelocity(), request.Velocity) + request.FeedForward;
    }

    public void setControl(MotionMagicDutyCycle request) {
        setControl(
                new MotionMagicVoltage(request.Position).withFeedForward(request.FeedForward * 12));
    }

    public void setControl(MotionMagicVoltage request) {
        voltageRequest =
                () ->
                        profiledController.calculate(getPosition(), request.Position)
                                + request.FeedForward;
    }

    public double getVelocity() {
        return Units.rpmToRps(motorSim.getAngularVelocityRPM()) * conversionFactor;
    }

    public double getPosition() {
        return motorSim.getAngularPositionRotations() * conversionFactor;
    }

    public double getAcceleration() {
        return acceleration.get();
    }

    public double getAppliedCurrent() {
        return motorSim.getCurrentDrawAmps();
    }

    public double getAppliedVoltage() {
        return voltageRequest.getAsDouble();
    }
}
