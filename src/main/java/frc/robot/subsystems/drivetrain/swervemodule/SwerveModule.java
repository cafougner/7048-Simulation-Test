package frc.robot.subsystems.drivetrain.swervemodule;

import static edu.wpi.first.units.Units.*;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleIO;
import frc.robot.subsystems.drivetrain.swervemodule.io.SwerveModuleInputsAutoLogged;

/**
 * A pseudo-subsystem for a swerve module that manages its own loggable inputs.
*/
public final class SwerveModule {
    private final String m_swerveModuleKey;
    private final SwerveModuleIO m_swerveModuleIO;
    private final SwerveModuleInputsAutoLogged m_swerveModuleInputs = new SwerveModuleInputsAutoLogged();

    private final SwerveModulePosition m_swerveModulePosition = new SwerveModulePosition();
    private final SwerveModuleState m_swerveModuleState = new SwerveModuleState();

    /**
     * Constructs a new SwerveModule with the supplied IO interface and name.
     * 
     * @param swerveModuleIO An implementation of the SwerveModuleIO interface.
     * @param swerveModuleName The name of the swerve module, used for logging and replay.
    */
    public SwerveModule(SwerveModuleIO swerveModuleIO, String swerveModuleName) {
        m_swerveModuleIO = swerveModuleIO;
        m_swerveModuleKey = "DrivetrainSubsystem/SwerveModule" + swerveModuleName;
    }

    /**
     * Updates and processes the swerve module's inputs for logging and replay.
     * <p> This should be called periodically, before any other class methods are called.
    */
    public final void periodic() {
        m_swerveModuleIO.updateInputs(m_swerveModuleInputs);
        Logger.processInputs(m_swerveModuleKey, m_swerveModuleInputs);
    }

    /**
     * Sets the swerve module's motor's onboard closed-loop setpoints from a desired state.
     * 
     * @param desiredState The unoptimized, desired state of the swerve module.
    */
    public final void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentWheelRotation = getWheelRotation();

        desiredState.optimize(currentWheelRotation);
        desiredState.cosineScale(currentWheelRotation);

        m_swerveModuleIO.setDriveVelocity(MetersPerSecond.of(desiredState.speedMetersPerSecond));
        m_swerveModuleIO.setDriveAngle(Radians.of(desiredState.angle.getRadians()));
    }

    /**
     * Sets the drive motor's onboard closed-loop velocity setpoint from a given drive velocity.
     * 
     * @param linearVelocity The desired linear velocity of the wheel.
    */
    public final void setDriveVelocity(LinearVelocity linearVelocity) {
        m_swerveModuleIO.setDriveVelocity(linearVelocity);
    }

    /**
     * Sets the azimuth motor's onboard closed-loop position setpoint from a given drive angle.
     * 
     * @param angle The desired angle of the wheel relative to its zero.
    */
    public final void setDriveAngle(Angle angle) {
        m_swerveModuleIO.setDriveAngle(angle);
    }

    /**
     * Directly applies a voltage to the drive motor.
     * <p> This will cancel any onboard closed-loop control.
     * 
     * @param voltage The voltage to apply to the drive motor.
    */
    public final void setDriveMotorVoltage(Voltage voltage) {
        m_swerveModuleIO.setDriveMotorVoltage(voltage);
    }

    /**
     * Directly applies a voltage to the azimuth motor.
     * <p> This will cancel any onboard closed-loop control.
     * 
     * @param voltage The voltage to apply to the azimuth motor.
    */
    public final void setAzimuthMotorVoltage(Voltage voltage) {
        m_swerveModuleIO.setAzimuthMotorVoltage(voltage);
    }

    /** Updates and returns the swerve module's current position. */
    public final SwerveModulePosition getSwerveModulePosition() {
        m_swerveModulePosition.angle = getWheelRotation();
        m_swerveModulePosition.distanceMeters = getWheelPosition().in(Meters);
        return m_swerveModulePosition;
    }

    /** Updates and returns the swerve module's current state. */
    public final SwerveModuleState getSwerveModuleState() {
        m_swerveModuleState.angle = getWheelRotation();
        m_swerveModuleState.speedMetersPerSecond = getWheelVelocity().in(MetersPerSecond);
        return m_swerveModuleState;
    }

    /** Returns the current position of the swerve module's wheel. */
    public final Distance getWheelPosition() {
        double driveMotorPosition = m_swerveModuleInputs.driveMotorPosition.in(Rotations);
        return Meters.of(driveMotorPosition * kDriveMotorToWheelFactor);
    }

    /** Returns the current velocity of the swerve module's wheel. */
    public final LinearVelocity getWheelVelocity() {
        double driveMotorVelocity = m_swerveModuleInputs.driveMotorVelocity.in(RotationsPerSecond);
        return MetersPerSecond.of(driveMotorVelocity * kDriveMotorToWheelFactor);
    }

    /** Returns the current angle of the swerve module's wheel relative to its zero. */
    public final Angle getWheelAngle() {
        return m_swerveModuleInputs.azimuthMotorPosition;
    }

    /**
     * Returns the current rotation of the swerve module's wheel relative to its zero.
     * <p> This is shorthand for calling {@code Rotation2d.fromRadians(getWheelAngle().in(Radians))}.
    */
    public final Rotation2d getWheelRotation() {
        return Rotation2d.fromRadians(getWheelAngle().in(Radians));
    }
}
