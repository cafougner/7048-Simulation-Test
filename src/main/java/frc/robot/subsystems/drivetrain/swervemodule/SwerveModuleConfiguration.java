package frc.robot.subsystems.drivetrain.swervemodule;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A simple data class for a swerve module configuration, containing the module's position and relevant CAN IDs.
*/
public final class SwerveModuleConfiguration {
    private final int m_driveMotorID;
    private final int m_azimuthMotorID;
    private final int m_azimuthEncoderID;
    private final Translation2d m_moduleTranslation;

    /**
     * Constructs a new SwerveModuleConfiguration with the given configurations.
     * 
     * @param driveMotorID The CAN ID of the drive motor.
     * @param azimuthMotorID The CAN ID of the azimuth motor.
     * @param azimuthEncoderID The CAN ID of the azimuth encoder.
     * @param moduleTranslation The robot-center relative translation of the swerve module.
    */
    public SwerveModuleConfiguration(
        int driveMotorID,
        int azimuthMotorID,
        int azimuthEncoderID,
        Translation2d moduleTranslation
    ) {
        m_driveMotorID = driveMotorID;
        m_azimuthMotorID = azimuthMotorID;
        m_azimuthEncoderID = azimuthEncoderID;
        m_moduleTranslation = moduleTranslation;
    }

    public final int getDriveMotorID() {
        return m_driveMotorID;
    }

    public final int getAzimuthMotorID() {
        return m_azimuthMotorID;
    }

    public final int getAzimuthEncoderID() {
        return m_azimuthEncoderID;
    }

    public final Translation2d getModuleTranslation() {
        return m_moduleTranslation;
    }
}
