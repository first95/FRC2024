// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Add your docs here. */
public class PoseLatency {
    public final Pose2d pose2d;
    public final Pose3d pose3d;
    public final double latency;

    public PoseLatency(Pose3d pose3d, double latency) {
        this.pose3d = pose3d;
        this.pose2d = pose3d.toPose2d();
        this.latency = latency;
    }

    public PoseLatency(double[] poseComponents) {
        pose3d = new Pose3d(
            poseComponents[0],
            poseComponents[1],
            poseComponents[2],
            new Rotation3d(
                Math.toRadians(poseComponents[3]),
                Math.toRadians(poseComponents[4]),
                Math.toRadians(poseComponents[5])
            )
        );
        pose2d = new Pose2d(
            poseComponents[0],
            poseComponents[1],
            Rotation2d.fromDegrees(poseComponents[5])
        );
        latency = poseComponents[6];
    }
}
