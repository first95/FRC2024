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
  private double portSpeed, starboardSpeed;
  private BooleanSupplier upSup, downSup, portUpSup, portDownSup, sboardUpSup, sboardDownSup;
  private boolean lastUp, lastDown, lastPortUp, lastPortDown, lastSboardUp, lastSboardDown;
  private enum State {
    SHOOTING, IDLE, HOLDING, SPOOLING
  }

  // Real
  private State currentState;
  private double intakeSpeed, portShootingSpeed, starboardShootingSpeed, loaderSpeed, commandedIntakeSpeed;
  private boolean sensorvalue, shooterbutton, shooterAtSpeed;
  
  public NoteHandlerCommand(Shooter shooter, Intake intake, DoubleSupplier intakeSpeedAxis, BooleanSupplier shooterButtonSupplier, BooleanSupplier upSup, BooleanSupplier downSup, BooleanSupplier portUpSup, BooleanSupplier portDownSup, BooleanSupplier sboardUpSup, BooleanSupplier sboardDownSup) {
    this.shooter = shooter;
    this.intake = intake;
    this.intakeSpeedAxis = intakeSpeedAxis;
    this.shooterButtonSupplier = shooterButtonSupplier;

    // Testing
    this.upSup = upSup;
    this.downSup = downSup;
    this.portDownSup = portDownSup;
    this.portUpSup = portUpSup;
    this.sboardDownSup = sboardDownSup;
    this.sboardUpSup = sboardUpSup;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = State.IDLE;

    // For testing
    ang = shooter.getArmAngle();
    portSpeed = ShooterConstants.PORT_SHOOTER_SPEED;
    starboardSpeed = ShooterConstants.STARBOARD_SHOOTER_SPEED;
    shooter.setArmAngle(ang);
    lastDown = false;
    lastUp = false;
    lastPortDown = false;
    lastPortUp = false;
    lastSboardDown = false;
    lastSboardUp = false;
    shooter.setShooterSpeed(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sensorvalue = shooter.getNoteSensor();
    shooterbutton = shooterButtonSupplier.getAsBoolean();
    commandedIntakeSpeed = intakeSpeedAxis.getAsDouble();
    shooterAtSpeed = (Math.abs(portSpeed - shooter.getPortShooterSpeed())
    <= ShooterConstants.SHOOTER_SPEED_TOLERANCE) &&
    (Math.abs(starboardSpeed - shooter.getStarboardShooterSpeed())
    <= ShooterConstants.SHOOTER_SPEED_TOLERANCE);

    // This is for testing
    if (upSup.getAsBoolean() && !lastUp && (ang.getRadians() <= ShooterConstants.ARM_UPPER_LIMIT.getRadians())) {
      ang = ang.plus(Rotation2d.fromDegrees(5));
      shooter.setArmAngle(ang);
    } else if (downSup.getAsBoolean() && !lastDown) {
      ang = ang.minus(Rotation2d.fromDegrees(5));
      shooter.setArmAngle(ang);
    }

    if (portUpSup.getAsBoolean() && !lastPortUp) {
      portSpeed += 100;
      shooter.setShooterSpeed(portSpeed, starboardSpeed);
    } else if (portDownSup.getAsBoolean() && !lastPortDown && portSpeed > 0) {
      portSpeed -= 100;
      shooter.setShooterSpeed(portSpeed, starboardSpeed);
    }

    if (sboardUpSup.getAsBoolean() && !lastSboardUp) {
      starboardSpeed += 100;
      shooter.setShooterSpeed(portSpeed, starboardSpeed);
    } else if (sboardDownSup.getAsBoolean() && !lastSboardDown && starboardSpeed > 0) {
      starboardSpeed -= 100;
      shooter.setShooterSpeed(portSpeed, starboardSpeed);
    }

    // This is the FSM
    switch(currentState){
      case IDLE:
      intakeSpeed = commandedIntakeSpeed;
      loaderSpeed = (commandedIntakeSpeed > 0) ? ShooterConstants.LOADER_INTAKE_SPEED : 0;
      portShootingSpeed = 0;
      starboardShootingSpeed = 0;
      if (shooterbutton) {
        currentState = State.SPOOLING;
      } else if (sensorvalue) {
        currentState = State.HOLDING;
      }
      break;

      case SPOOLING:
        portShootingSpeed = portSpeed;
        starboardShootingSpeed = starboardSpeed;
        intakeSpeed = commandedIntakeSpeed < 0 ? commandedIntakeSpeed : 0;
        loaderSpeed = commandedIntakeSpeed < 0 ? -ShooterConstants.LOADER_INTAKE_SPEED : 0;

        if (!shooterbutton){
          currentState = State.HOLDING;
        }
        if (shooterbutton && shooterAtSpeed){
          currentState = State.SHOOTING;
        }
      break;

      case SHOOTING:
        intakeSpeed = commandedIntakeSpeed < 0 ? commandedIntakeSpeed : 0;
        loaderSpeed = ShooterConstants.LOADER_FIRING_SPEED;
        portShootingSpeed = portSpeed;
        starboardShootingSpeed = starboardSpeed;

        if (!shooterbutton){
          currentState = State.IDLE;
        }
      break;
      
      case HOLDING:
        intakeSpeed = commandedIntakeSpeed < 0 ? commandedIntakeSpeed : 0;
        loaderSpeed = commandedIntakeSpeed < 0 ? -ShooterConstants.LOADER_INTAKE_SPEED : 0;
        portShootingSpeed = 0;
        starboardShootingSpeed = 0;
        if (shooterbutton){
          currentState = State.SPOOLING;
        }
      break;
    }
    
    intake.runRollers(intakeSpeed);
    shooter.setShooterSpeed(portShootingSpeed, starboardShootingSpeed);
    shooter.runLoader(loaderSpeed);

    // This is also for testing
    lastDown = downSup.getAsBoolean();
    lastUp = upSup.getAsBoolean();
    lastPortDown = portDownSup.getAsBoolean();
    lastPortUp = portUpSup.getAsBoolean();
    lastSboardDown = sboardDownSup.getAsBoolean();
    lastSboardUp = sboardUpSup.getAsBoolean();

    SmartDashboard.putNumber("PortSpeed", portSpeed);
    SmartDashboard.putNumber("StarboardSpeed", starboardSpeed);
    SmartDashboard.putString("AngleCommand", ang.toString());
    
    // These are real
    SmartDashboard.putString("currentState",currentState.toString());
    SmartDashboard.putNumber("intakeSpeed", intakeSpeed);
    SmartDashboard.putNumber("shooterSpeed", portShootingSpeed);
    SmartDashboard.putNumber("loaderSpeed", loaderSpeed);
    SmartDashboard.putBoolean("sensor", sensorvalue);
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
