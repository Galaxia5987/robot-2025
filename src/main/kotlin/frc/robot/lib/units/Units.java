package frc.robot.lib.units;

public class Units {

    public static double rpmToRadsPerSec(double rpm) {
        return rpm * 2.0 * Math.PI / 60.0;
    }

    public static double radsPerSecToRpm(double radsPerSec) {
        return radsPerSec * 60.0 / (2.0 * Math.PI);
    }

    public static double rpsToRpm(double rps) {
        return rps * 60.0;
    }

    public static double rpmToRps(double rpm) {
        return rpm / 60.0;
    }

    public static double radsPerSecToRps(double radsPerSec) {
        return rpmToRps(radsPerSecToRpm(radsPerSec));
    }

    public static double rpsToRadsPerSec(double rps) {
        return rpmToRadsPerSec(rpsToRpm(rps));
    }

    public static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    public static double metersToInches(double meters) {
        return meters / 0.0254;
    }

    public static double feetToMeters(double feet) {
        return inchesToMeters(feet * 12.0);
    }

    public static double metersToFeet(double meters) {
        return metersToInches(meters) / 12.0;
    }

    public static double metersToRotations(double value, double radius) {
        return value / (2 * Math.PI * radius);
    }

    public static double rpsToMetersPerSecond(double value, double radius) {
        return value * (2 * Math.PI * radius);
    }
}
