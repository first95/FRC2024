// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.Auton;
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
    Command command = new AutoShoot(drive).withTimeout(5);
    command.setName("Shoot Preload");
    return command;
  }

  public static Command twoNoteCenter(SwerveBase drive, Intake intake) {
    Command command = new AutoShoot(drive, 3)
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, Auton.AUTO_INTAKE_SPEED)))
      .andThen(new AlignToPose("CenterNearNote", drive))
      .andThen(new AutoShoot(drive, 5))
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0)));
      command.setName("Center");
      command.finallyDo(() -> {SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0);});
      return command;
  }

  public static Command twoNoteAmp(SwerveBase drive, Intake intake) {
    Command command = new AutoShoot(drive, 3)
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, Auton.AUTO_INTAKE_SPEED)))
      .andThen(new AlignToPose("AmpNote", drive))
      .andThen(new AutoShoot(drive, 5))
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0)));
      command.setName("Amp");
      command.finallyDo(() -> {SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0);});
      return command;
  }

  public static Command twoNotePodium(SwerveBase drive, Intake intake) {
    Command command = new AutoShoot(drive, 3)
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, Auton.AUTO_INTAKE_SPEED)))
      .andThen(new AlignToPose("PodiumNote", drive))
      .andThen(new AutoShoot(drive, 5))
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0)));
      command.setName("Podium");
      command.finallyDo(() -> {SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0);});
      return command;
  }
  

  public static Command threeNoteCenterAmp(SwerveBase drive, Intake intake, Map<String, ChoreoTrajectory> trajMap) {
    Command command = new AutoShoot(drive, 3)
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, Auton.AUTO_INTAKE_SPEED)))
      .andThen(new AlignToPose("CenterNearNote", drive))
      .andThen(new AutoShoot(drive, 2))
      .andThen(new FollowTrajectory(trajMap.get("3NoteCenterAmp"), drive, false, true))
      .andThen(new AutoShoot(drive, 5))
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0)));
    command.setName("Center Amp");
    command.finallyDo(() -> {SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0);});
    return command;
  }

  public static Command threeNoteCenterPodium(SwerveBase drive, Intake intake, Map<String, ChoreoTrajectory> trajMap) {
    Command command = new AutoShoot(drive, 3)
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, Auton.AUTO_INTAKE_SPEED)))
      .andThen(new AlignToPose("CenterNearNote", drive))
      .andThen(new AutoShoot(drive, 2))
      .andThen(new FollowTrajectory(trajMap.get("4NoteAmpFirst2"), drive, false, true))
      .andThen(new AutoShoot(drive, 5))
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0)));
    command.setName("Center Podium");
    command.finallyDo(() -> {SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0);});
    return command;
  }

  public static Command threeNoteAmpCenter(SwerveBase drive, Intake intake, Map<String, ChoreoTrajectory> trajMap) {
    Command command = new AutoShoot(drive, 3)
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, Auton.AUTO_INTAKE_SPEED)))
      .andThen(new AlignToPose("AmpNote", drive))
      .andThen(new AutoShoot(drive, 2))
      .andThen(new FollowTrajectory(trajMap.get("4NoteAmpFirst1"), drive, false, true))
      .andThen(new AutoShoot(drive, 5))
      .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0)));
    command.setName("Amp Center");
    command.finallyDo(() -> {SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0);});
    return command;
  }

  public static Command fourNearNotes(SwerveBase drive, Map<String, ChoreoTrajectory> trajMap) {
    Command command = new AutoShoot(drive, 2.5)
    .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, Auton.AUTO_INTAKE_SPEED)))
    .andThen(new AlignToPose("AmpNote", drive))
    .andThen(new AutoShoot(drive, 2.5))
    .andThen(new FollowTrajectory(trajMap.get("4NoteAmpFirst1"), drive, false, true))
    .andThen(new AutoShoot(drive, 2))
    .andThen(new FollowTrajectory(trajMap.get("4NoteAmpFirst2"), drive, false, true))
    .andThen(new AutoShoot(drive, 4))
    .andThen(new InstantCommand(() -> SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0)));
    command.setName("Four Note Auto");
    command.finallyDo(() -> {SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0);});
    return command;
  }

  public static Command midlineDisruptor(SwerveBase drive, Map<String, ChoreoTrajectory> trajMap) {
    Command command = new InstantCommand(() -> SmartDashboard.putBoolean(Auton.PURGE_MODE_KEY, true))
    .andThen(new FollowTrajectory(trajMap.get("CenterRocket"), drive, false, true))
    .alongWith(
      new WaitCommand(4.5)
      .andThen(new InstantCommand(() -> {
        SmartDashboard.putBoolean(Auton.PURGE_MODE_KEY, false);
        SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 1);
      }))
    )
    .andThen(new AutoShoot(drive, 5));
    command.setName("Centerline Disruptor");
    command.finallyDo(() -> {
      SmartDashboard.putBoolean(Auton.PURGE_MODE_KEY, false);
      SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0);
    });
    return command;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
