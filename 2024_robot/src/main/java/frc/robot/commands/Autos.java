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
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command shootPreload(SwerveBase drive) {
    return new AutoShoot(drive).withTimeout(5);
  }

  public static Command twoNoteCenter(SwerveBase drive, Intake intake) {
    Command command = new AutoShoot(drive).withTimeout(2)
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, Auton.AUTO_INTAKE_SPEED)))
      .andThen(new AlignToPose("CenterNearNote", drive))
      .andThen(new AutoShoot(drive)).withTimeout(5)
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0)));
      command.end(SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0));
      return command;
  }

  public static Command threeNoteCenterAmp(SwerveBase drive, Intake intake, Map<String, ChoreoTrajectory> trajMap) {
    Command command = new AutoShoot(drive, 3)
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, Auton.AUTO_INTAKE_SPEED)))
      .andThen(new AlignToPose("CenterNearNote", drive))
      .andThen(new InstantCommand(() -> SmartDashboard.putString("AutoState", "DoneAlign")))
      .andThen(new AutoShoot(drive, 5))
      .andThen(new InstantCommand(() -> SmartDashboard.putString("AutoState", "DoneShot2")))
      .andThen(new FollowTrajectory(trajMap.get("3NoteCenterAmp.traj"), drive, false, true))
      .andThen(new InstantCommand(() -> SmartDashboard.putString("AutoState", "DonePath")))
      .andThen(new AutoShoot(drive, 5))
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0)));
    command.end(SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0));
    return command;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
