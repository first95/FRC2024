// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Method;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NoteHandlerConstants;
public class NoteHandler extends SubsystemBase {

  private final CANSparkMax Intake_Roller,shooter,loader;
  private final SparkPIDController shooterPID;
  private final SimpleMotorFeedforward feedforward;
  private final DigitalInput loaderSensor;
  private final RelativeEncoder shooterEncoder;
  public NoteHandler() {
    Intake_Roller = new CANSparkMax(NoteHandlerConstants.INTAKE_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
    shooter = new CANSparkMax(NoteHandlerConstants.SHOOTER_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
    loader = new CANSparkMax(NoteHandlerConstants.LOADER_MOTOR_CONTROLLER_ID,  MotorType.kBrushless);
    shooterPID = shooter.getPIDController();
    feedforward = new SimpleMotorFeedforward(NoteHandlerConstants.SHOOTER_KS, NoteHandlerConstants.SHOOTER_KV, NoteHandlerConstants.SHOOTER_KA);
    loaderSensor = new DigitalInput(NoteHandlerConstants.LOADER_SENSOR_ID);
    shooterEncoder = shooter.getEncoder();

    Intake_Roller.restoreFactoryDefaults();
    shooter.restoreFactoryDefaults();
    loader.restoreFactoryDefaults();
    
    Intake_Roller.setInverted(NoteHandlerConstants.INVERT_INTAKE_ROLLER);
    shooter.setInverted(NoteHandlerConstants.INVERT_SHOOTER);
    shooterPID.setP(NoteHandlerConstants.SHOOTER_KP);
    shooterPID.setI(NoteHandlerConstants.SHOOTER_KI);
    shooterPID.setD(NoteHandlerConstants.SHOOTER_KD);
    shooterPID.setFF(NoteHandlerConstants.SHOOTER_KFF);
    loader.setInverted(NoteHandlerConstants.INVERT_LOADER);

   
    Intake_Roller.setIdleMode(IdleMode.kBrake);
    shooter.setIdleMode(IdleMode.kCoast);
    loader.setIdleMode(IdleMode.kBrake);

    Intake_Roller.burnFlash();
    shooter.burnFlash();
    loader.burnFlash();

  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ShooterVolts", shooter.get() * shooter.getBusVoltage());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void setIntakeSpeed(double speed){
    Intake_Roller.set(speed);
  }
  public void setLoaderSpeed(double speed){
    loader.set(speed);
  }
  public void setShooterRPM(double RPM){
    shooterPID.setReference(RPM, ControlType.kVelocity , 0, feedforward.calculate(RPM));
  }
  public void setShooterSpeed(double speed) {
    shooter.set(speed);
  }
  public boolean getLoaderSensor(){
    return loaderSensor.get();
  }
  public double getShooterRPM(){
    return shooterEncoder.getVelocity();
  }
}
