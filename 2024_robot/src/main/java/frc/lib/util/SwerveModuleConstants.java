package frc.lib.util;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final double angleOffset, xPos, yPos;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, double angleOffset, double xPos, double yPos) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.angleOffset = angleOffset;
        this.xPos = xPos;
        this.yPos = yPos;
    }
}
