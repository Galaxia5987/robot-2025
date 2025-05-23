// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.ConstantsKt.LOOP_TIME;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ConstantsKt;
import frc.robot.Mode;
import frc.robot.lib.LocalADStarAK;
import frc.robot.lib.math.GalacticSlewRateLimiter;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    // TunerConstants doesn't include these constants, so they are declared locally
    static final int ODOMETRY_FREQUENCY =
            new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 200 : 100;
    public static final double DRIVE_BASE_RADIUS =
            Math.max(
                    Math.max(
                            Math.hypot(
                                    TunerConstants.FrontLeft.LocationX,
                                    TunerConstants.FrontLeft.LocationY),
                            Math.hypot(
                                    TunerConstants.FrontRight.LocationX,
                                    TunerConstants.FrontRight.LocationY)),
                    Math.max(
                            Math.hypot(
                                    TunerConstants.BackLeft.LocationX,
                                    TunerConstants.BackLeft.LocationY),
                            Math.hypot(
                                    TunerConstants.BackRight.LocationX,
                                    TunerConstants.BackRight.LocationY)));

    // PathPlanner config constants
    private static final Mass ROBOT_MASS_KG = Kilograms.of(52.757);
    private static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(5.095);
    private static final double WHEEL_COF = 1.542;
    private static final RobotConfig PP_CONFIG =
            new RobotConfig(
                    ROBOT_MASS_KG,
                    ROBOT_MOI,
                    new ModuleConfig(
                            TunerConstants.FrontLeft.WheelRadius,
                            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                            WHEEL_COF,
                            DCMotor.getKrakenX60Foc(1)
                                    .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                            TunerConstants.FrontLeft.SlipCurrent,
                            1),
                    getModuleTranslations());

    public static final DriveTrainSimulationConfig mapleSimConfig =
            DriveTrainSimulationConfig.Default()
                    .withRobotMass(ROBOT_MASS_KG)
                    .withCustomModuleTranslations(getModuleTranslations())
                    .withGyro(COTS.ofNav2X())
                    .withSwerveModule(
                            new SwerveModuleSimulationConfig(
                                    DCMotor.getKrakenX60(1),
                                    DCMotor.getFalcon500(1),
                                    TunerConstants.FrontLeft.DriveMotorGearRatio,
                                    TunerConstants.FrontLeft.SteerMotorGearRatio,
                                    Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                                    Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                                    Meters.of(TunerConstants.FrontLeft.WheelRadius),
                                    KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                                    WHEEL_COF));

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    public Angle[] SwerveTurnAngle =
            new Angle[] {Radians.zero(), Radians.zero(), Radians.zero(), Radians.zero()};
    public Angle[] SwerveDriveAngle =
            new Angle[] {Radians.zero(), Radians.zero(), Radians.zero(), Radians.zero()};

    private Consumer<Pose2d> resetSimulationPoseCallBack = null;

    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final SysIdRoutine turnSysId;
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
    @AutoLogOutput private Rotation2d desiredHeading;

    private Rotation2d gyroOffset = Rotation2d.kZero;

    public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    public SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private final SwerveDrivePoseEstimator globalPoseEstimator =
            new SwerveDrivePoseEstimator(
                    kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    private final SwerveDrivePoseEstimator localPoseEstimator =
            new SwerveDrivePoseEstimator(
                    kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    private static final GalacticSlewRateLimiter slewRateLimiterX =
            new GalacticSlewRateLimiter(1.5);
    private static final GalacticSlewRateLimiter slewRateLimiterY =
            new GalacticSlewRateLimiter(1.5);

    public Drive(
            GyroIO gyroIO, ModuleIO[] moduleIOS, Optional<SwerveDriveSimulation> driveSimulation) {
        this(gyroIO, moduleIOS[0], moduleIOS[1], moduleIOS[2], moduleIOS[3], driveSimulation);
    }

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
            Optional<SwerveDriveSimulation> driveSimulation) {
        this.gyroIO = gyroIO;

        driveSimulation.ifPresent(
                swerveDriveSimulation ->
                        this.resetSimulationPoseCallBack =
                                swerveDriveSimulation::setSimulationWorldPose);

        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(
                tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();
        var scale = 3;
        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getChassisSpeeds,
                this::limitlessRunVelocity,
                new PPHolonomicDriveController(
                        new PIDConstants(3 * scale, 0.0, 0.0),
                        new PIDConstants(0.8 * scale, 0.0, 0.0)),
                PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory",
                            activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });

        // Configure SysId
        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) ->
                                        Logger.recordOutput("Drive/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
        turnSysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) ->
                                        Logger.recordOutput(
                                                "Drive/TurnSysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> runTurnCharacterization(voltage.in(Volts)),
                                null,
                                this));

        gyroIO.updateInputs(gyroInputs);
        desiredHeading = gyroInputs.yawPosition;
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();
        SwerveTurnAngle[0] = modules[0].getAngle().getMeasure();
        SwerveTurnAngle[1] = modules[1].getAngle().getMeasure();
        SwerveTurnAngle[2] = modules[2].getAngle().getMeasure();
        SwerveTurnAngle[3] = modules[3].getAngle().getMeasure();

        SwerveDriveAngle[0] = Radians.of(modules[0].getWheelRadiusCharacterizationPosition());
        SwerveDriveAngle[1] = Radians.of(modules[1].getWheelRadiusCharacterizationPosition());
        SwerveDriveAngle[2] = Radians.of(modules[2].getWheelRadiusCharacterizationPosition());
        SwerveDriveAngle[3] = Radians.of(modules[3].getWheelRadiusCharacterizationPosition());
        Logger.recordOutput("SwerveStates/Measured", getModuleStates());
        Logger.recordOutput("Odometry/Robot", getPose());

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps =
                modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] =
                        new SwerveModulePosition(
                                modulePositions[moduleIndex].distanceMeters
                                        - lastModulePositions[moduleIndex].distanceMeters,
                                modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            globalPoseEstimator.updateWithTime(
                    sampleTimestamps[i], rawGyroRotation, modulePositions);
            localPoseEstimator.updateWithTime(
                    sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(
                !gyroInputs.connected && ConstantsKt.getCURRENT_MODE() != Mode.SIM);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);
        Logger.recordOutput("SwerveChassisSpeeds/Measured", getChassisSpeeds());
        Logger.recordOutput("Drive/DesiredHeading", getDesiredHeading());

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void limitlessRunVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);
        Logger.recordOutput("SwerveChassisSpeeds/Measured", getChassisSpeeds());
        Logger.recordOutput("Drive/DesiredHeading", getDesiredHeading());

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void setAngle(Rotation2d angle) {
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(new SwerveModuleState(0.0, angle));
        }
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** NOTE: DO NOT USE WITH TorqueCurrentFOC */
    public void runTurnCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runTurnCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules
     * will return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction));
    }

    /**
     * Returns a command to run a quasistatic test in the specified direction on the turn modules.
     */
    public Command turnSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runTurnCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(turnSysId.quasistatic(direction));
    }

    public Command runAllTurnSysID() {
        return Commands.sequence(
                turnSysIdQuasistatic(SysIdRoutine.Direction.kForward),
                turnSysIdQuasistatic(SysIdRoutine.Direction.kReverse),
                turnSysIdDynamic(SysIdRoutine.Direction.kForward),
                turnSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    /** Returns a command to run a dynamic test in the specified direction on the turn modules. */
    public Command turnSysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runTurnCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(turnSysId.dynamic(direction));
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "SwerveChassisSpeeds/MeasuredFieldOriented")
    public ChassisSpeeds getFieldOrientedSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                kinematics.toChassisSpeeds(getModuleStates()), getRotation());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return globalPoseEstimator.getEstimatedPosition();
    }

    /** Returns the local estimated pose. */
    @AutoLogOutput(key = "Odometry/LocalEstimatedPose")
    public Pose2d getLocalEstimatedPose() {
        return localPoseEstimator.getEstimatedPosition();
    }

    public void resetLocalPoseEstimatorBasedOnGlobal() {
        localPoseEstimator.resetPose(globalPoseEstimator.getEstimatedPosition());
    }

    /** Returns the current gyro rotation or the estimated rotation if the gyro disconnects. */
    public Rotation2d getRotation() {
        return gyroInputs.connected
                ? gyroInputs.yawPosition.plus(gyroOffset)
                : getPose().getRotation();
    }

    public Rotation2d[] getGyroMeasurements() {
        return gyroInputs.odometryYawPositions;
    }

    public double[] getGyroTimestamps() {
        return gyroInputs.odometryYawTimestamps;
    }

    public Command updateDesiredHeading(DoubleSupplier omegaAxis) {
        return Commands.run(
                () -> {
                    double desiredDeltaOmega =
                            MathUtil.applyDeadband(omegaAxis.getAsDouble(), 0.15)
                                    * TunerConstants.kMaxOmegaVelocity.in(RadiansPerSecond)
                                    * LOOP_TIME;
                    desiredHeading = desiredHeading.plus(Rotation2d.fromRadians(desiredDeltaOmega));
                });
    }

    public Rotation2d getDesiredHeading() {
        return desiredHeading;
    }

    public Command setDesiredHeading(Rotation2d rotation) {
        return Commands.runOnce(() -> desiredHeading = rotation);
    }

    /** Resets the current odometry pose. */
    public void resetOdometry(Pose2d pose) {
        if (frc.robot.ConstantsKt.getUSE_MAPLE_SIM()) {
            resetSimulationPoseCallBack.accept(pose);
        }
        globalPoseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
        localPoseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    public void resetGyro(Rotation2d offset) {
        gyroIO.zeroGyro();
        gyroOffset = offset;
        desiredHeading = new Rotation2d();
    }

    public void resetGyroBasedOnAlliance(Rotation2d gyroOffset) {
        resetGyro(ConstantsKt.getIS_RED() ? gyroOffset : gyroOffset.minus(Rotation2d.k180deg));
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        globalPoseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public void addLocalVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        localPoseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(
                    TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(
                    TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(
                    TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }
}
