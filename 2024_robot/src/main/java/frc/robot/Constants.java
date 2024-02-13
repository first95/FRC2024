// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.BetterSwerveKinematics;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double NEO_FREE_SPEED = 5676; // RPM
  public static final double NEO_STALL_TORQUE = 3.75; // N * m
  public static final double NEO_550_FREE_SPEED = 11000; // RPM
  public static final double SPARK_MAX_RESPONSE_LOOP = 0.13; // 130ms
  public static final double VORTEX_FREE_SPEED = 6784; // RPM
  public static final double VORTEX_STALL_TORQUE = 3.6; // N * m
  
  public static final double GRAVITY = 9.81; // m/s/s

  public static final double LOOP_CYCLE = 0.02; // 20ms
  
  public static final class Drivebase {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static final int SWERVE_MODULE_CURRENT_LIMIT = 60;

    public static final double HEADING_TOLERANCE = Math.toRadians(1);

    // Motor and encoder inversions
    public static final boolean ABSOLUTE_ENCODER_INVERT = true;
    public static final boolean DRIVE_MOTOR_INVERT = true;
    public static final boolean ANGLE_MOTOR_INVERT = false;
    public static final boolean INVERT_GYRO = false;

    // Module locations, in meters, as distances to the center of the robot.
    // Positive x is torwards the robot front, and +y is torwards robot left.
    public static final double FRONT_LEFT_X = Units.inchesToMeters(7.875);
    public static final double FRONT_LEFT_Y = Units.inchesToMeters(12.25);
    public static final double FRONT_RIGHT_X = Units.inchesToMeters(7.875);
    public static final double FRONT_RIGHT_Y = Units.inchesToMeters(-12.25);
    public static final double BACK_LEFT_X = Units.inchesToMeters(-14.625);
    public static final double BACK_LEFT_Y = Units.inchesToMeters(12.25);
    public static final double BACK_RIGHT_X = Units.inchesToMeters(-14.625);
    public static final double BACK_RIGHT_Y = Units.inchesToMeters(-12.25);

    public static final Translation2d[] MODULE_LOCATIONS = {
        new Translation2d(Drivebase.FRONT_LEFT_X, Drivebase.FRONT_LEFT_Y),
        new Translation2d(Drivebase.FRONT_RIGHT_X, Drivebase.FRONT_RIGHT_Y),
        new Translation2d(Drivebase.BACK_LEFT_X, Drivebase.BACK_LEFT_Y),
        new Translation2d(Drivebase.BACK_RIGHT_X, Drivebase.BACK_RIGHT_Y)
    };

    // IMU Mounting. CCW Positive
    public static final double IMU_MOUNT_YAW = 0;
    public static final double IMU_MOUNT_PITCH = 0;
    public static final double IMU_MOUNT_ROLL = 0;

    // Drivetrain limitations
    public static final double MAX_SPEED = (VORTEX_FREE_SPEED * Units.inchesToMeters(3 * Math.PI)) / (60 * 4.71); // meters per second NOT A LIMIT!!! DO NOT TOUCH!!!
    public static final double MAX_ANGULAR_VELOCITY = MAX_SPEED / Math.hypot(FRONT_LEFT_X, FRONT_LEFT_Y); // rad/s
    // Theoretical max acceleration should be as follows:
    // (NEO stall torque * module gearing * number of modules) / (wheel radius * robot mass) = m/s/s
    // (2.6 * 6.75 * 4) / (Units.inchesToMeters(2) * ROBOT_MASS)
    // However, the drive is traction-limited, so the max accelration is actually (wheel coefficient of friction * gravity)
    public static final double MAX_ACCELERATION = 1 * GRAVITY; // COF is 1.1 but careful
    public static final double MAX_ANGULAR_ACCELERATION = MAX_ACCELERATION / Math.hypot(FRONT_LEFT_X, FRONT_LEFT_Y);
    // max speed (RPM) / gear ratio, convert to deg/min, divide by 60 for deg/s
    public static final double MAX_MODULE_ANGULAR_SPEED = Units.rotationsToDegrees(NEO_550_FREE_SPEED * 7 / 372) / 60; // deg/s

    // Robot heading control gains
    public static final double HEADING_KP = 0.4 * (MAX_ANGULAR_VELOCITY / Math.PI);
    public static final double HEADING_KI = 0;
    public static final double HEADING_KD = 0.01 * (MAX_ANGULAR_VELOCITY / Math.PI);
    
    // Swerve base kinematics object
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_LOCATIONS);

    // Module PIDF gains
    public static final double MODULE_KP = 0.01;
    public static final double MODULE_KI = 0;
    public static final double MODULE_KD = 0;
    public static final double MODULE_IZ = 0;
    public static final double MODULE_KF = 0;
    // Volt * seconds / degree.  Equal to (maxVolts) / ((degreesPerRotation) * (maxMotorSpeedRPM / gearRatio) * (minutesPerSecond))
    public static final double MODULE_KV = 12 / MAX_MODULE_ANGULAR_SPEED;

    public static final double VELOCITY_KP = 0.2; // kp from SysId, eventually
    public static final double VELOCITY_KI = 0; // Leave all of these zero to disable them
    public static final double VELOCITY_KD = 0;
    public static final double VELOCITY_IZ = 0;
    public static final double VELOCITY_KF = 0;

    public static final double CURRENT_KP = 0;
    public static final double CURRENT_KI = 0.2;
    public static final double CURRENT_KD = 0;
    public static final double CURRENT_MOVING_AVERAGE_SAMPLES = 1;

    public static final int NUM_MODULES = 4;

    // Drive feedforward gains
    public static final double KS = 0.12; // Volts
    public static final double KV = 12 / MAX_SPEED; // Volt-seconds per meter (max voltage divided by max speed)
    public static final double KA = (12 / MAX_ACCELERATION) / NUM_MODULES; // Volt-seconds^2 per meter (max voltage divided by max accel)
    public static final double KG = (KA / KV);

    // Encoder conversion values.  Drive converts motor rotations to linear wheel distance
    // and steering converts motor rotations to module azimuth
    public static final double METERS_PER_MOTOR_ROTATION = (Math.PI * Units.inchesToMeters(3)) / 4.71;
        // Calculation: 3in diameter wheels * pi [circumfrence] / gear ratio
    public static final double DEGREES_PER_STEERING_ROTATION = 360;
        // degrees per rotation / gear ratio between module and motor

        // Module specific constants
    public static final class Mod0 {
        public static final int DRIVE_MOTOR_ID = 3;
        public static final int ANGLE_MOTOR_ID = 2;
        public static final double ANGLE_OFFSET = 360 - 20.3;
        public static final SwerveModuleConstants CONSTANTS =
            new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET, FRONT_LEFT_X, FRONT_LEFT_Y);
    }
    public static final class Mod1 {
        public static final int DRIVE_MOTOR_ID = 5;
        public static final int ANGLE_MOTOR_ID = 4;
        public static final double ANGLE_OFFSET = 360 - 128.5;
        public static final SwerveModuleConstants CONSTANTS =
            new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET, FRONT_RIGHT_X, FRONT_RIGHT_Y);
    }
    public static final class Mod2 {
        public static final int DRIVE_MOTOR_ID = 7;
        public static final int ANGLE_MOTOR_ID = 6;
        public static final double ANGLE_OFFSET = 360 - 58.6;
        public static final SwerveModuleConstants CONSTANTS =
            new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET, BACK_LEFT_X, BACK_LEFT_Y);
    }
    public static final class Mod3 {
        public static final int DRIVE_MOTOR_ID = 9;
        public static final int ANGLE_MOTOR_ID = 8;
        public static final double ANGLE_OFFSET = 360 - 198.4;
        public static final SwerveModuleConstants CONSTANTS =
            new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, ANGLE_OFFSET, BACK_RIGHT_X, BACK_RIGHT_Y);
    }

    public static final int PIGEON = 30;
}
    public static class Vision {
        public static final int APRILTAG_PIPELINE_NUMBER = 0;
        public static final String PORT_LIMELIGHT_NAME = "port";
        public static final String STARBOARD_LIMELIGHT_NAME = "sboard";
        public static final double POSE_ERROR_TOLERANCE = 0.5;
        public static final double ANGULAR_ERROR_TOLERANCE = Math.toRadians(2);
}
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
