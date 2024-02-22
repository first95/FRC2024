// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Auton;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.NoteHandlerSpeeds;
import frc.robot.commands.autocommands.AlignToPose;
import frc.robot.commands.autocommands.FollowTrajectory;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveBase;

import java.util.Map;

import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command shootPreload(SwerveBase drive) {
    return new AutoShoot(drive).withTimeout(5);
  }

  public static Command twoNoteCenter(SwerveBase drive, Intake intake, Map<String, ChoreoTrajectory> trajMap) {
    return new AutoShoot(drive).withTimeout(2)
      .andThen(new AlignToPose("3/4NoteCenterStart", drive))
      .alongWith(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, IntakeConstants.INTAKE_MAX_SPEED)))
      .andThen(new FollowTrajectory(trajMap.get("3NoteCenterAmp.1.traj"), drive, true, true))
      .andThen(new AutoShoot(drive)).withTimeout(3)
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0)));
  }

  public static Command threeNoteCenterAmp(SwerveBase drive, Intake intake, Map<String, ChoreoTrajectory> trajMap) {
    return new AutoShoot(drive).withTimeout(2)
      .andThen(new AlignToPose("3/4NoteCenterStart", drive))
      .alongWith(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, IntakeConstants.INTAKE_MAX_SPEED)))
      .andThen(new FollowTrajectory(trajMap.get("3NoteCenterAmp.1.traj"), drive, true, true))
      .andThen(new AutoShoot(drive)).withTimeout(2)
      .andThen(new FollowTrajectory(trajMap.get("3NoteCenterAmp.2.traj"), drive, true, true))
      .andThen(new AutoShoot(drive)).withTimeout(2)
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0)));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
