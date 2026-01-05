package frc.robot.subsystems.drivetrain.gyro;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.drivetrain.gyro.io.GyroIO;
import frc.robot.subsystems.drivetrain.gyro.io.GyroInputsAutoLogged;

public final class Gyro {
    private final GyroIO m_gyroIO;
    private final GyroInputsAutoLogged m_gyroInputs;

    public Gyro(GyroIO gyroIO) {
        m_gyroIO = gyroIO;
        m_gyroInputs = new GyroInputsAutoLogged();
    }

    public final void periodic() {
        m_gyroIO.updateInputs(m_gyroInputs);
        Logger.processInputs("DrivetrainSubsystem/Gyro", m_gyroInputs);
    }

    public final Angle getAngle() {
        return m_gyroInputs.yawAngle;
    }

    public final AngularVelocity getAngularVelocity() {
        return m_gyroInputs.yawVelocity;
    }

    public final Rotation2d getRotation() {
        return Rotation2d.fromRadians(getAngle().in(Radians));
    }
}
