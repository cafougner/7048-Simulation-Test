package frc.robot.subsystems.drivetrain.gyro.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public abstract class GyroIOBase implements GyroIO {
    protected final Pigeon2 m_gyro;

    private final StatusSignal<Angle> m_gyroPitchAngle;
    private final StatusSignal<Angle> m_gyroRollAngle;
    private final StatusSignal<Angle> m_gyroYawAngle;
    private final StatusSignal<AngularVelocity> m_gyroPitchVelocity;
    private final StatusSignal<AngularVelocity> m_gyroRollVelocity;
    private final StatusSignal<AngularVelocity> m_gyroYawVelocity;
    private final StatusSignal<Voltage> m_gyroSuppliedVoltage;

    public GyroIOBase(int gyroID) {
        m_gyro = new Pigeon2(gyroID);

        m_gyroPitchAngle = m_gyro.getPitch(false);
        m_gyroRollAngle = m_gyro.getRoll(false);
        m_gyroYawAngle = m_gyro.getYaw(false);
        m_gyroPitchVelocity = m_gyro.getAngularVelocityXWorld(false);
        m_gyroRollVelocity = m_gyro.getAngularVelocityYWorld(false);
        m_gyroYawVelocity = m_gyro.getAngularVelocityZWorld(false);
        m_gyroSuppliedVoltage = m_gyro.getSupplyVoltage(false);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            m_gyroPitchAngle,
            m_gyroRollAngle,
            m_gyroYawAngle,
            m_gyroPitchVelocity,
            m_gyroRollVelocity,
            m_gyroYawVelocity,
            m_gyroSuppliedVoltage
        );

        m_gyro.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroInputs loggableInputs) {
        loggableInputs.isConnected = BaseStatusSignal.refreshAll(
            m_gyroPitchAngle,
            m_gyroRollAngle,
            m_gyroYawAngle,
            m_gyroPitchVelocity,
            m_gyroRollVelocity,
            m_gyroYawVelocity,
            m_gyroSuppliedVoltage
        ).equals(StatusCode.OK);

        loggableInputs.pitchAngle = m_gyroPitchAngle.getValue();
        loggableInputs.rollAngle = m_gyroRollAngle.getValue();
        loggableInputs.yawAngle = m_gyroYawAngle.getValue();
        loggableInputs.pitchVelocity = m_gyroPitchVelocity.getValue();
        loggableInputs.rollVelocity = m_gyroRollVelocity.getValue();
        loggableInputs.yawVelocity = m_gyroYawVelocity.getValue();
        loggableInputs.suppliedVoltage = m_gyroSuppliedVoltage.getValue();
    }
}
