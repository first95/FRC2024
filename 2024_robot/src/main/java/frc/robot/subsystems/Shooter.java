// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final CANSparkFlex leftShooter, rightShooter;
  private final CANSparkMax loader, shoulder, shoulder2;
  private final RelativeEncoder leftShooterEncoder, rightShooterEncoder, shoulderEncoder;
  private final SparkPIDController leftShooterPID, rightShooterPID, shoulderPID;
  private final TrapezoidProfile shoulderProfile;
  private final SparkLimitSwitch bottomLimitSwitch;
  private final DigitalInput noteSensor;
  private final SimpleMotorFeedforward flywheelFeedforward;
  private final ArmFeedforward shoulderFeedforward;
  /** Creates a new ExampleSubsystem. */
  public Shooter() {
    leftShooter = new CANSparkFlex(ShooterConstants.LEFT_SHOOTER_ID, MotorType.kBrushless);
    rightShooter = new CANSparkFlex(ShooterConstants.RIGHT_SHOOTER_ID, MotorType.kBrushless);
    loader = new CANSparkMax(ShooterConstants.LOADER_ID, MotorType.kBrushless);
    shoulder = new CANSparkMax(ShooterConstants.SHOULDER_ID, MotorType.kBrushless);
    shoulder2 = new CANSparkMax(ShooterConstants.LEFT_SHOOTER_ID, MotorType.kBrushless);

    leftShooter.restoreFactoryDefaults();
    rightShooter.restoreFactoryDefaults();
    loader.restoreFactoryDefaults();
    shoulder.restoreFactoryDefaults();
    shoulder2.restoreFactoryDefaults();

    shoulder2.follow(shoulder, true);

    leftShooter.setInverted(ShooterConstants.INVERT_LEFT_SHOOTER);
    rightShooter.setInverted(ShooterConstants.INVERT_RIGHT_SHOOTER);
    loader.setInverted(ShooterConstants.INVERT_LOADER);
    shoulder.setInverted(ShooterConstants.INVERT_SHOULDER);

    leftShooter.setIdleMode(IdleMode.kCoast);
    rightShooter.setIdleMode(IdleMode.kCoast);
    loader.setIdleMode(IdleMode.kBrake);
    shoulder.setIdleMode(IdleMode.kBrake);
    shoulder2.setIdleMode(IdleMode.kBrake);

    leftShooterEncoder = leftShooter.getEncoder();
    rightShooterEncoder = rightShooter.getEncoder();
    shoulderEncoder = shoulder.getEncoder();

    shoulderEncoder.setPositionConversionFactor(ShooterConstants.ARM_RADIANS_PER_MOTOR_ROTATION);
    shoulderEncoder.setVelocityConversionFactor(ShooterConstants.ARM_RADIANS_PER_MOTOR_ROTATION / 60);

    leftShooterPID = leftShooter.getPIDController();
    rightShooterPID = rightShooter.getPIDController();
    shoulderPID = shoulder.getPIDController();

    leftShooterPID.setP(ShooterConstants.FLYWHEEL_KP);
    leftShooterPID.setI(ShooterConstants.FLYWHEEL_KI);
    leftShooterPID.setD(ShooterConstants.FLYWHEEL_KD);
    leftShooterPID.setFF(ShooterConstants.FLYWHEEL_KF);

    rightShooterPID.setP(ShooterConstants.FLYWHEEL_KP);
    rightShooterPID.setI(ShooterConstants.FLYWHEEL_KI);
    rightShooterPID.setD(ShooterConstants.FLYWHEEL_KD);
    rightShooterPID.setFF(ShooterConstants.FLYWHEEL_KF);

    shoulderPID.setP(ShooterConstants.SHOULDER_KP);
    shoulderPID.setI(ShooterConstants.SHOULDER_KI);
    shoulderPID.setD(ShooterConstants.SHOULDER_KD);
    shoulderPID.setFF(ShooterConstants.SHOULDER_KF);

    bottomLimitSwitch = shoulder.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    shoulderProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        ShooterConstants.MAX_SPEED,
        ShooterConstants.MAX_ACCELERATION)
    );

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

    leftShooter.burnFlash();
    rightShooter.burnFlash();
    loader.burnFlash();
    shoulder.burnFlash();
    shoulder2.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
