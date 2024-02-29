// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.util.LimelightHelpers.LimelightResults;

/** Packages useful information for AprilTag localization into an object created from json results */
public class CamData {
  public final Pose2d pose2d;
  public final Pose3d pose3d;
  public final double latency, ta;
  public final int numTargets, pipeline;
  public final boolean valid;

  public CamData() {
    pose3d = new Pose3d();
    pose2d = new Pose2d();
    latency = 0;
    ta = 0;
    numTargets = 0;
    valid = false;
    pipeline = 0;
  }

  public CamData(boolean valid, Pose3d pose3d, double latency, double ta, int numTargets, int pipeline) {
    this.pose3d = pose3d;
    this.pose2d = pose3d.toPose2d();
    this.latency = latency;
    this.ta = ta;
    this.numTargets = numTargets;
    this.valid = valid;
    this.pipeline = pipeline;
  }

  public CamData(LimelightResults results) {
    if (results.targetingResults.valid) {
      valid = true;
      pose3d = new Pose3d(
          new Translation3d(
              results.targetingResults.botpose_wpiblue[0],
              results.targetingResults.botpose_wpiblue[1],
              results.targetingResults.botpose_wpiblue[2]),
          new Rotation3d(
              Math.toRadians(results.targetingResults.botpose_wpiblue[3]),
              Math.toRadians(results.targetingResults.botpose_wpiblue[4]),
              Math.toRadians(results.targetingResults.botpose_wpiblue[5])));
      pose2d = pose3d.toPose2d();
      latency = results.targetingResults.latency_capture + results.targetingResults.latency_pipeline
          + results.targetingResults.latency_jsonParse;
      ta = results.targetingResults.targets_Fiducials[0].ta;
      numTargets = results.targetingResults.targets_Fiducials.length;
      pipeline = (int) results.targetingResults.pipelineID;
    } else {
      pose3d = new Pose3d();
      pose2d = new Pose2d();
      latency = 0;
      ta = 0;
      numTargets = 0;
      valid = false;
      pipeline = 0;
    }
  }
}
