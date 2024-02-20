// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Auton;
import frc.robot.Constants.Drivebase;
import frc.robot.subsystems.SwerveBase;

public class AlignToPose extends Command {
  private final Pose2d target;
  private final SwerveBase swerve;
  private final TrapezoidProfile driveProfile;
  private final PIDController xController, yController, thetaController;
  private final Timer timer;
  
  private Pose2d currentRelativePose;
  private double initialDistance, cosine, sine;
  private TrapezoidProfile.State goalState, initialState;

  /** Creates a new AlignToPose. */
  public AlignToPose(Pose2d pose, SwerveBase swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    target = pose;
    this.swerve = swerve;
    driveProfile = new TrapezoidProfile(Auton.DRIVE_CONSTRAINTS);
    xController = new PIDController(
      Auton.DRIVE_KP, Auton.DRIVE_KI, Auton.DRIVE_KD);
    yController = new PIDController(
      Auton.DRIVE_KP, Auton.DRIVE_KI, Auton.DRIVE_KD);
    thetaController = new PIDController(
      Drivebase.HEADING_KP, Drivebase.HEADING_KI, Drivebase.HEADING_KD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.setTolerance(Auton.DRIVE_POSITIONAL_TOLERANCE);
    yController.setTolerance(Auton.DRIVE_POSITIONAL_TOLERANCE);
    thetaController.setTolerance(Drivebase.HEADING_TOLERANCE);
    timer = new Timer();

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var initialPose = swerve.getPose();
    var deltaTranslation = target.getTranslation().minus(initialPose.getTranslation());
    var currentVelocity = swerve.getFieldVelocity();
    var driveAngle = deltaTranslation.getAngle();
    cosine = driveAngle.getCos();
    sine = driveAngle.getSin();
    initialDistance = deltaTranslation.getNorm();
    initialState = new TrapezoidProfile.State(0, Math.hypot(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond));
    goalState = new TrapezoidProfile.State(initialDistance, 0);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var currentPose = swerve.getPose();
    currentRelativePose = new Pose2d(
      target.getTranslation().minus(currentPose.getTranslation()),
      currentPose.getRotation()
    );
    var profileSetpoint = driveProfile.calculate(timer.get(), initialState, goalState);
    
    var translation = new Translation2d(
      xController.calculate(
        currentRelativePose.getX(),
        profileSetpoint.position * cosine
      ) + profileSetpoint.velocity * cosine,
      yController.calculate(
        currentRelativePose.getY(),
        profileSetpoint.position * sine
      ) + profileSetpoint.velocity * sine
    );

    var omega = thetaController.calculate(currentRelativePose.getRotation().getRadians(), target.getRotation().getRadians());
    omega = (Math.abs(omega) < Drivebase.HEADING_MIN_ANGULAR_CONTROL_EFFORT) ? 0 : omega;

    swerve.drive(translation, omega, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.setDriveBrake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}
