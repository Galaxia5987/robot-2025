package frc.robot.lib.motors;

import com.ctre.phoenix6.controls.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.math.differential.Derivative;

public class TalonFXSim extends SimMotor {

    private final Derivative acceleration = new Derivative();

    public TalonFXSim(
            LinearSystem<N2, N1, N2> model,
            int numMotors,
            double gearing,
            double conversionFactor,
            TalonType motorType) {
        super(model, TalonType.getDCMotor(motorType, numMotors), gearing, conversionFactor);
    }

    public TalonFXSim(
            DCMotor motor, double gearing, double jKgMetersSquared, double conversionFactor) {
        super(motor, jKgMetersSquared, gearing, conversionFactor);
    }

    public TalonFXSim(
            int numMotors,
            double gearing,
            double jKgMetersSquared,
            double conversionFactor,
            TalonType talonType) {
        super(
                TalonType.getDCMotor(talonType, numMotors),
                jKgMetersSquared,
                gearing,
                conversionFactor);
    }

    @Override
    public void update(double timestampSeconds) {
        super.update(timestampSeconds);

        acceleration.update(getVelocity().in(Units.RotationsPerSecond), timestampSeconds);
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
                () ->
                        controller.calculate(
                                        getVelocity().in(Units.RotationsPerSecond),
                                        request.Velocity)
                                + request.FeedForward;
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

    public AngularVelocity getVelocity() {
        return Units.Rotation.per(Units.Minutes)
                .of(motorSim.getAngularVelocityRPM())
                .times(conversionFactor);
    }

    public double getPosition() {
        return motorSim.getAngularPositionRotations() * conversionFactor;
    }

    public double getAcceleration() {
        return acceleration.get();
    }

    public Current getAppliedCurrent() {
        return Units.Amps.of(motorSim.getCurrentDrawAmps());
    }

    public Voltage getAppliedVoltage() {
        return Units.Volts.of(motorSim.getInputVoltage());
    }
}
