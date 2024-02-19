// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class NoteHandlerCommand extends Command {
  private final Shooter shooter;
  private final Intake intake;
  private final DoubleSupplier intakeSpeedAxis;
  private final BooleanSupplier shooterButtonSupplier;

  // Testing
  private Rotation2d ang;
  private BooleanSupplier upSup, downSup;
  private boolean lastUp, lastDown;

  private enum State {
    SHOOTING, IDLE, HOLDING, SPOOLING
  }

  // Real
  private State currentState;
  private double intakeSpeed, portShootingSpeed, starboardShootingSpeed, loaderSpeed, commandedIntakeSpeed, portSpeed, starboardSpeed;
  private boolean sensorvalue, shooterbutton, shooterAtSpeed;

  public NoteHandlerCommand(Shooter shooter, Intake intake, DoubleSupplier intakeSpeedAxis,
      BooleanSupplier shooterButtonSupplier, BooleanSupplier upSup, BooleanSupplier downSup) {
    this.shooter = shooter;
    this.intake = intake;
    this.intakeSpeedAxis = intakeSpeedAxis;
    this.shooterButtonSupplier = shooterButtonSupplier;

    // Testing
    this.upSup = upSup;
    this.downSup = downSup;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = State.IDLE;

    // For testing
    ang = shooter.getArmAngle();
    portSpeed = ShooterConstants.PORT_SHOOTER_SPEED;
    starboardSpeed = ShooterConstants.STARBOARD_SHOOTER_SPEED;
    SmartDashboard.putNumber("PortSpeed", portSpeed);
    SmartDashboard.putNumber("StarboardSpeed", starboardSpeed);
    shooter.setArmAngle(ang);
    lastDown = false;
    lastUp = false;
    shooter.setShooterSpeed(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Read inputs
    sensorvalue = shooter.getNoteSensor();
    shooterbutton = shooterButtonSupplier.getAsBoolean();
    commandedIntakeSpeed = intakeSpeedAxis.getAsDouble();
    shooterAtSpeed = (Math.abs(portSpeed - shooter.getPortShooterSpeed()) <= ShooterConstants.SHOOTER_SPEED_TOLERANCE)
        &&
        (Math.abs(starboardSpeed - shooter.getStarboardShooterSpeed()) <= ShooterConstants.SHOOTER_SPEED_TOLERANCE);
    portSpeed = SmartDashboard.getNumber("PortSpeed", portSpeed);
    starboardSpeed = SmartDashboard.getNumber("StarboardSpeed", starboardSpeed);

    // This is for testing
    if (upSup.getAsBoolean() && !lastUp && (ang.getRadians() <= ShooterConstants.ARM_UPPER_LIMIT.getRadians())) {
      ang = ang.plus(Rotation2d.fromDegrees(5));
      shooter.setArmAngle(ang);
    } else if (downSup.getAsBoolean() && !lastDown) {
      ang = ang.minus(Rotation2d.fromDegrees(5));
      shooter.setArmAngle(ang);
    }

    // Execute statemachine logic
    switch (currentState) {
      case IDLE:
        // Set desired outputs in this state
        intakeSpeed = commandedIntakeSpeed;
        loaderSpeed = (commandedIntakeSpeed > 0) ? ShooterConstants.LOADER_INTAKE_SPEED : 0;
        portShootingSpeed = 0;
        starboardShootingSpeed = 0;

        // Determine if we neeed to change state
        if (shooterbutton) {
          currentState = State.SPOOLING;
        } else if (sensorvalue) {
          currentState = State.HOLDING;
        }

        // Don't forget this-- things get real weird when every successive case can execute
        break;

      case SPOOLING:
        portShootingSpeed = portSpeed;
        starboardShootingSpeed = starboardSpeed;
        intakeSpeed = commandedIntakeSpeed < 0 ? commandedIntakeSpeed : 0;
        loaderSpeed = commandedIntakeSpeed < 0 ? -ShooterConstants.LOADER_INTAKE_SPEED : 0;

        if (!shooterbutton) {
          currentState = sensorvalue ? State.HOLDING : State.IDLE;
        }
        if (shooterbutton && shooterAtSpeed) {
          currentState = State.SHOOTING;
        }

        break;

      case SHOOTING:
        intakeSpeed = commandedIntakeSpeed < 0 ? commandedIntakeSpeed : 0;
        loaderSpeed = ShooterConstants.LOADER_FIRING_SPEED;
        portShootingSpeed = portSpeed;
        starboardShootingSpeed = starboardSpeed;

        // This could (should) have a special case to go back to holding if the sensor
        // is still tripped, but 1) that's really rare, and 2) it doesn't matter if that
        // happens; it will get fixed on the next cycle.
        if (!shooterbutton) {
          currentState = State.IDLE;
        }

        break;

      case HOLDING:
        intakeSpeed = commandedIntakeSpeed < 0 ? commandedIntakeSpeed : 0;
        loaderSpeed = commandedIntakeSpeed < 0 ? -ShooterConstants.LOADER_INTAKE_SPEED : 0;
        portShootingSpeed = 0;
        starboardShootingSpeed = 0;

        if (shooterbutton) {
          currentState = State.SPOOLING;
        }

        break;
    }

    // Apply outputs
    intake.runRollers(intakeSpeed);
    shooter.setShooterSpeed(portShootingSpeed, starboardShootingSpeed);
    shooter.runLoader(loaderSpeed);

    // This is also for testing
    lastDown = downSup.getAsBoolean();
    lastUp = upSup.getAsBoolean();

    SmartDashboard.putString("AngleCommand", ang.toString());

    // These are real
    SmartDashboard.putString("currentState", currentState.toString());
    SmartDashboard.putNumber("intakeSpeed", intakeSpeed);
    SmartDashboard.putNumber("shooterSpeed", portShootingSpeed);
    SmartDashboard.putNumber("loaderSpeed", loaderSpeed);
    SmartDashboard.putBoolean("sensor", sensorvalue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
