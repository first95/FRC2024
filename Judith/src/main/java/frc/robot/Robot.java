// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Auton;
import frc.robot.Constants.NoteHandlerSpeeds;
import frc.robot.Constants.ShooterConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private boolean sentAlliance = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    initializeDashboard();

    // Grab the build computer, branchname, git commit ID and build timestamp from the Jar manifest
    // and toss the on the smart dashboard
    
    String hostbranch = Robot.class.getPackage().getImplementationTitle();
    if ( hostbranch != null && ! hostbranch.isEmpty()) {
      SmartDashboard.putString("BuildHost-BranchName", hostbranch);
    } else {
      SmartDashboard.putString("BuildHost-BranchName","No JAR Manifest in simulation");
    }
    
    String CommitIDtime = Robot.class.getPackage().getImplementationVersion();
    if (CommitIDtime != null && ! CommitIDtime.isEmpty()) {
      SmartDashboard.putString("GitCommitID-BuildTimestamp", CommitIDtime);
    } else {
      SmartDashboard.putString("GitCommitID-BuildTimestamp", "No JAR Manifest in simulation");
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if (!sentAlliance && DriverStation.isDSAttached()) {
      m_robotContainer.sendAlliance();
      sentAlliance = true;
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    initializeDashboard();
    m_robotContainer.stopDrive();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void initializeDashboard() {
    SmartDashboard.putNumber(Auton.ARM_ANGLE_KEY, ArmConstants.ARM_LOWER_LIMIT.getRadians());
    SmartDashboard.putBoolean(Auton.AUTO_SHOOTING_KEY, false);
    SmartDashboard.putBoolean(Auton.ON_TARGET_KEY, false);
    SmartDashboard.putBoolean(Auton.AUTO_AMP_ALIGN_KEY, false);
    SmartDashboard.putBoolean(Auton.AUTO_AMP_SCORE_KEY, false);
    SmartDashboard.putNumber(Auton.AUTO_INTAKE_SPEED_KEY, 0);
  }
}
