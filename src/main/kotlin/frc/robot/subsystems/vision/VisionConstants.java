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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashMap;
import java.util.Map;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String FrontOVName = "front";
    public static String RightOVName = "backRight";
    public static String LeftOVName = "backLeft";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToFrontOV =
            new Transform3d(
                    0.23246,
                    -0.18929,
                    0.54793,
                    new Rotation3d(0.0, Math.toRadians(30.0), Math.toRadians(20.0)));
    public static Transform3d robotToRightOV =
            new Transform3d(
                    -0.12547,
                    -0.23183,
                    0.55182,
                    new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(205.0)));
    public static Transform3d robotToLeftOV =
            new Transform3d(
                    -0.12635,
                    0.23345,
                    0.55208,
                    new Rotation3d(0.0, Math.toRadians(-15.0), Math.toRadians(155.0)));

    public static Map<String, Transform3d> OVNameToTransform =
            new HashMap<>() {
                {
                    put(FrontOVName, robotToFrontOV);
                    put(RightOVName, robotToRightOV);
                    put(LeftOVName, robotToLeftOV);
                }
            };

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.3;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
            new double[] {
                1.0, // OV1
                1.0, // OV2
                1.0 // OV3
            };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
            Double.POSITIVE_INFINITY; // No rotation data available
}
