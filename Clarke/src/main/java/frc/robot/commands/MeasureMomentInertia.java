// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveBase;

import java.io.FileWriter;
import java.io.PrintWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MeasureMomentInertia extends Command {
  private final SwerveBase swerve;
  private final Timer time;
  private double lastOmega, lastTime, alpha, omega, currentTime;
  private PrintWriter logger;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MeasureMomentInertia(SwerveBase swerve) {
    this.swerve = swerve;
    time = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    try {
      FileWriter fileWriter = new FileWriter("/U/accelData/log.txt");
      logger = new PrintWriter(fileWriter);
    } catch (Exception e) {
      DriverStation.reportError("Log File Cannot Be Opened!", true);
    }
    if (SmartDashboard.getNumber("Amps", 123456789) == 123456789) {
      SmartDashboard.putNumber("Amps", 0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double amps = SmartDashboard.getNumber("Amps", 0);
    swerve.constantForceSpin(amps);
    time.reset();
    time.start();
    lastTime = time.get();
    lastOmega = swerve.getAngVelocity().getRadians();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    omega = swerve.getAngVelocity().getRadians();
    currentTime = time.get();
    alpha = (omega - lastOmega) / (currentTime - lastTime);
    logger.print(Double.toString(alpha) + "\\n");
    lastOmega = omega;
    lastTime = currentTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    logger.close();
    time.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
