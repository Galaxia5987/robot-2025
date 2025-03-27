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

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.InitializerKt;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionConsumer globalConsumer;
    private final VisionConsumer localConsumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;

    public Vision(VisionConsumer globalConsumer, VisionConsumer localConsumer, VisionIO... io) {
        this.globalConsumer = globalConsumer;
        this.localConsumer = localConsumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                    new Alert(
                            "Vision camera " + io[i].getName() + " is disconnected.",
                            AlertType.kWarning);
        }
    }

    public int getBestTargetID(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.id();
    }

    public Supplier<Rotation2d> getYawToTarget(int cameraIndex) {
        return () -> inputs[cameraIndex].yawToTarget;
    }

    public Translation3d getTranslationToBestTarget(int cameraIndex) {
        return inputs[cameraIndex].translationToBestTarget;
    }

    public Transform3d getTransformToID(int cameraIndex, int id) {
        for (int i = 0; i < inputs[cameraIndex].trackedTargets.length; i++) {
            if (inputs[cameraIndex].trackedTargetsIDs[i] == id) {
                return inputs[cameraIndex].trackedTargets[i];
            }
        }
        return null;
    }

    public int getIdOfClosestTarget(int cameraIndex) {
        double minDistance = Integer.MAX_VALUE;
        int index = 0;
        for (int i = 0; i < inputs[cameraIndex].trackedTargets.length; i++) {
            if (inputs[cameraIndex].trackedTargets[i].getTranslation().getNorm() < minDistance) {
                minDistance = inputs[cameraIndex].trackedTargets[i].getTranslation().getNorm();
            }
        }
        return index;
    }

    private boolean isObservationValid(VisionIO.PoseObservation observation) {
        // Check whether to reject pose
        return !(observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                        && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                        > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth());
    }

    private Pair<Double, Double> calculateStddev(double avgTagDistance, int tagCount) {
        double stdFactor = Math.pow(avgTagDistance, 2.0) / tagCount;
        double linearStddev = linearStdDevBaseline * stdFactor;
        double angularStddev = angularStdDevBaseline * stdFactor;
        return new Pair(linearStddev, angularStddev);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + io[i].getName(), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                boolean shouldIgnoreFeederInAuto = io[cameraIndex].getName().equals(FeederOVName) && RobotState.isAutonomous();
                boolean rejectPose = !isObservationValid(observation) || shouldIgnoreFeederInAuto;

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                var stddevs =
                        calculateStddev(observation.averageTagDistance(), observation.tagCount());
                double linearStdDev = stddevs.getFirst();
                double angularStdDev = stddevs.getSecond();
                if (observation.type() == VisionIO.PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                // Send vision observation
                globalConsumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            boolean isWithinMaxDistanceFromGlobal =
                    InitializerKt.getSwerveDrive()
                                    .getPose()
                                    .minus(inputs[cameraIndex].localEstimatedPose.pose().toPose2d())
                                    .getTranslation()
                                    .getNorm()
                            < MAX_DELTA_BETWEEN_LOCAL_AND_GLOBAL.in(Units.Meters);
            if (isObservationValid(inputs[cameraIndex].localEstimatedPose)
                    && !io[cameraIndex].getName().equals(FeederOVName)
                    && isWithinMaxDistanceFromGlobal) {
                var stddevs =
                        calculateStddev(
                                inputs[cameraIndex].localEstimatedPose.averageTagDistance(),
                                inputs[cameraIndex].localEstimatedPose.tagCount());
                double linearStdDev = stddevs.getFirst();
                double angularStdDev = stddevs.getSecond();
                localConsumer.accept(
                        inputs[cameraIndex].localEstimatedPose.pose().toPose2d(),
                        inputs[cameraIndex].localEstimatedPose.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/" + io[cameraIndex].getName() + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/" + io[cameraIndex].getName() + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/" + io[cameraIndex].getName() + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/" + io[cameraIndex].getName() + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput(
                "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPoses",
                allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
