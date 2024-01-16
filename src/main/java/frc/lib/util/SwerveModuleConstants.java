package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int absoluteEncoderPort;
    public final Rotation2d angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param absoluteEncoderPort
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int absoluteEncoderPort, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.absoluteEncoderPort = absoluteEncoderPort;
        this.angleOffset = angleOffset;
    }
}
