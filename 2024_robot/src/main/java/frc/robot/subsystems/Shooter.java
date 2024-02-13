// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  
  private final CANSparkFlex portShooter, starboardShooter;
  private final CANSparkMax loader, shoulder, shoulder2;
  private final RelativeEncoder portShooterEncoder, starboardShooterEncoder, shoulderEncoder;
  private final SparkPIDController portShooterPID, starboardShooterPID, shoulderPID;
  private final TrapezoidProfile shoulderProfile;
  private final SparkLimitSwitch bottomLimitSwitch;
  private final DigitalInput noteSensor;
  private final SimpleMotorFeedforward flywheelFeedforward;
  private final ArmFeedforward shoulderFeedforward;

  private Rotation2d armGoal;
  private final Timer timer;
  private TrapezoidProfile.State armSetpoint;
  /** Creates a new ExampleSubsystem. */
  public Shooter() {
    portShooter = new CANSparkFlex(ShooterConstants.PORT_SHOOTER_ID, MotorType.kBrushless);
    starboardShooter = new CANSparkFlex(ShooterConstants.STARBOARD_SHOOTER_ID, MotorType.kBrushless);
    loader = new CANSparkMax(ShooterConstants.LOADER_ID, MotorType.kBrushless);
    shoulder = new CANSparkMax(ShooterConstants.SHOULDER_ID, MotorType.kBrushless);
    shoulder2 = new CANSparkMax(ShooterConstants.SHOULDER_2_ID, MotorType.kBrushless);

    portShooter.restoreFactoryDefaults();
    starboardShooter.restoreFactoryDefaults();
    loader.restoreFactoryDefaults();
    shoulder.restoreFactoryDefaults();
    shoulder2.restoreFactoryDefaults();

    shoulder2.follow(shoulder, true);

    portShooter.setInverted(ShooterConstants.INVERT_PORT_SHOOTER);
    starboardShooter.setInverted(ShooterConstants.INVERT_STARBOARD_SHOOTER);
    loader.setInverted(ShooterConstants.INVERT_LOADER);
    shoulder.setInverted(ShooterConstants.INVERT_SHOULDER);

    portShooter.setIdleMode(IdleMode.kCoast);
    starboardShooter.setIdleMode(IdleMode.kCoast);
    loader.setIdleMode(IdleMode.kBrake);
    shoulder.setIdleMode(IdleMode.kBrake);
    shoulder2.setIdleMode(IdleMode.kBrake);

    portShooter.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
    starboardShooter.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
    loader.setSmartCurrentLimit(ShooterConstants.LOADER_CURRENT_LIMIT);
    shoulder.setSmartCurrentLimit(ShooterConstants.SHOULDER_CURRENT_LIMIT);
    shoulder2.setSmartCurrentLimit(ShooterConstants.SHOULDER_CURRENT_LIMIT);

    portShooter.setOpenLoopRampRate(ShooterConstants.SHOOTER_RAMP_RATE);
    starboardShooter.setOpenLoopRampRate(ShooterConstants.SHOOTER_RAMP_RATE);
    portShooter.setClosedLoopRampRate(ShooterConstants.SHOOTER_RAMP_RATE);
    starboardShooter.setClosedLoopRampRate(ShooterConstants.SHOOTER_RAMP_RATE);

    portShooterEncoder = portShooter.getEncoder();
    starboardShooterEncoder = starboardShooter.getEncoder();
    shoulderEncoder = shoulder.getEncoder();

    shoulderEncoder.setPositionConversionFactor(ShooterConstants.ARM_RADIANS_PER_MOTOR_ROTATION);
    shoulderEncoder.setVelocityConversionFactor(ShooterConstants.ARM_RADIANS_PER_MOTOR_ROTATION / 60);

    portShooterPID = portShooter.getPIDController();
    starboardShooterPID = starboardShooter.getPIDController();
    shoulderPID = shoulder.getPIDController();

    portShooterPID.setP(ShooterConstants.FLYWHEEL_KP);
    portShooterPID.setI(ShooterConstants.FLYWHEEL_KI);
    portShooterPID.setD(ShooterConstants.FLYWHEEL_KD);
    portShooterPID.setFF(ShooterConstants.FLYWHEEL_KF);

    starboardShooterPID.setP(ShooterConstants.FLYWHEEL_KP);
    starboardShooterPID.setI(ShooterConstants.FLYWHEEL_KI);
    starboardShooterPID.setD(ShooterConstants.FLYWHEEL_KD);
    starboardShooterPID.setFF(ShooterConstants.FLYWHEEL_KF);

    shoulderPID.setP(ShooterConstants.SHOULDER_KP);
    shoulderPID.setI(ShooterConstants.SHOULDER_KI);
    shoulderPID.setD(ShooterConstants.SHOULDER_KD);
    shoulderPID.setFF(ShooterConstants.SHOULDER_KF);

    bottomLimitSwitch = shoulder.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    bottomLimitSwitch.enableLimitSwitch(true);

    shoulderProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        ShooterConstants.MAX_SPEED,
        ShooterConstants.MAX_ACCELERATION)
    );
    armGoal = ShooterConstants.ARM_LOWER_LIMIT;

    shoulderFeedforward = new ArmFeedforward(
      ShooterConstants.SHOULDER_KS,
      ShooterConstants.SHOULDER_KG,
      ShooterConstants.SHOULDER_KV,
      ShooterConstants.SHOULDER_KA
    );
    
    flywheelFeedforward = new SimpleMotorFeedforward(
      ShooterConstants.FLYWHEEL_KS,
      ShooterConstants.FLYWHEEL_KV,
      ShooterConstants.FLYWHEEL_KA
    );

    noteSensor = new DigitalInput(ShooterConstants.NOTE_SENSOR_ID);

    portShooter.burnFlash();
    starboardShooter.burnFlash();
    loader.burnFlash();
    shoulder.burnFlash();
    shoulder2.burnFlash();

    timer = new Timer();
    timer.start();
  }

  public void runLoader(double speed) {
    loader.set(speed);
  }

  public void setShooterSpeed(double portRPM, double starboardRPM) {
    portShooterPID.setReference(portRPM, ControlType.kVelocity, 0, flywheelFeedforward.calculate(portRPM), ArbFFUnits.kVoltage);
    starboardShooterPID.setReference(starboardRPM, ControlType.kVelocity, 0, flywheelFeedforward.calculate(starboardRPM), ArbFFUnits.kVoltage);
  }

  public void setPortShooterRaw(double speed) {
    portShooter.set(speed);
  }

  public void setStarboardShooterRaw(double speed) {
    starboardShooter.set(speed);
  }

  public void setArmAngle(Rotation2d angle) {
    armGoal = angle;
    timer.reset();
  }

  public Rotation2d getArmAngle() {
    return Rotation2d.fromRadians(shoulderEncoder.getPosition());
  }

  public double getPortShooterSpeed() {
    return portShooterEncoder.getVelocity();
  }

  public double getStarboardShooterSpeed() {
    return starboardShooterEncoder.getVelocity();
  }

  public boolean getNoteSensor() {
    return noteSensor.get();
  }

  @Override
  public void periodic() {
    if (bottomLimitSwitch.isPressed()) {
      shoulderEncoder.setPosition(ShooterConstants.ARM_LOWER_LIMIT.getRadians());
    }
    if (armGoal.getRadians() >= ShooterConstants.ARM_UPPER_LIMIT.getRadians()) {
      armGoal = ShooterConstants.ARM_UPPER_LIMIT;
    }
    armSetpoint = shoulderProfile.calculate(
      timer.get(),
      new TrapezoidProfile.State(shoulderEncoder.getPosition(), shoulderEncoder.getVelocity()),
      new TrapezoidProfile.State(armGoal.getRadians(), 0)
    );
    // This really shouldn't be necessary
    armSetpoint = (armSetpoint.position >= ShooterConstants.ARM_UPPER_LIMIT.getRadians())
      ? new TrapezoidProfile.State(ShooterConstants.ARM_UPPER_LIMIT.getRadians(), 0)
      : armSetpoint;

    /*shoulderPID.setReference(
      armSetpoint.position,
      ControlType.kPosition,
      0,
      shoulderFeedforward.calculate(armSetpoint.position, armSetpoint.velocity),
      ArbFFUnits.kVoltage
    );*/

    SmartDashboard.putNumber("ShooterShoulderGoal", armGoal.getDegrees());
    SmartDashboard.putNumber("ShooterShoulderSetpoint", armSetpoint.position);
    SmartDashboard.putNumber("ShooterShoulderSetpointVel", armSetpoint.velocity);
    SmartDashboard.putNumber("PortVolts", portShooter.getAppliedOutput() * portShooter.getBusVoltage());
    SmartDashboard.putNumber("StarboardVolts", starboardShooter.getAppliedOutput() * starboardShooter.getBusVoltage());
    SmartDashboard.putNumber("PortRPM", portShooterEncoder.getVelocity());
    SmartDashboard.putNumber("StarboardRPM", starboardShooterEncoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
