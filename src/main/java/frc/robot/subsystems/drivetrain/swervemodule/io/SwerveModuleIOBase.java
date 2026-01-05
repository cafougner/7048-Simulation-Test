package frc.robot.subsystems.drivetrain.swervemodule.io;

import static edu.wpi.first.units.Units.*;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleConfiguration;

public abstract class SwerveModuleIOBase implements SwerveModuleIO {
    protected final TalonFX m_driveMotor;
    protected final TalonFX m_azimuthMotor;
    protected final CANcoder m_azimuthEncoder;

    private final StatusSignal<Angle> m_driveMotorPosition;
    private final StatusSignal<AngularVelocity> m_driveMotorVelocity;
    private final StatusSignal<AngularAcceleration> m_driveMotorAcceleration;
    private final StatusSignal<Voltage> m_driveMotorAppliedVoltage;
    private final StatusSignal<Current> m_driveMotorStatorCurrent;

    private final StatusSignal<Angle> m_azimuthMotorPosition;
    private final StatusSignal<AngularVelocity> m_azimuthMotorVelocity;
    private final StatusSignal<AngularAcceleration> m_azimuthMotorAcceleration;
    private final StatusSignal<Voltage> m_azimuthMotorAppliedVoltage;
    private final StatusSignal<Current> m_azimuthMotorStatorCurrent;

    private final StatusSignal<Angle> m_azimuthEncoderPosition;
    private final StatusSignal<AngularVelocity> m_azimuthEncoderVelocity;

    private final MotionMagicVelocityVoltage m_driveMotorControlRequest = new MotionMagicVelocityVoltage(0.0);
    private final MotionMagicVoltage m_azimuthMotorControlRequest = new MotionMagicVoltage(0.0);

    public SwerveModuleIOBase(SwerveModuleConfiguration configuration) {
        m_driveMotor = new TalonFX(configuration.getDriveMotorID());
        m_azimuthMotor = new TalonFX(configuration.getAzimuthMotorID());
        m_azimuthEncoder = new CANcoder(configuration.getAzimuthEncoderID());

        m_driveMotor.getConfigurator().apply(kDriveMotorConfiguration);
        m_azimuthMotor.getConfigurator().apply(kAzimuthMotorConfiguration.withFeedback(
            new FeedbackConfigs().withRemoteCANcoder(m_azimuthEncoder)
        ));

        m_driveMotorPosition = m_driveMotor.getPosition(false);
        m_driveMotorVelocity = m_driveMotor.getVelocity(false);
        m_driveMotorAcceleration = m_driveMotor.getAcceleration(false);
        m_driveMotorAppliedVoltage = m_driveMotor.getMotorVoltage(false);
        m_driveMotorStatorCurrent = m_driveMotor.getStatorCurrent(false);

        m_azimuthMotorPosition = m_azimuthMotor.getPosition(false);
        m_azimuthMotorVelocity = m_azimuthMotor.getVelocity(false);
        m_azimuthMotorAcceleration = m_azimuthMotor.getAcceleration(false);
        m_azimuthMotorAppliedVoltage = m_azimuthMotor.getMotorVoltage(false);
        m_azimuthMotorStatorCurrent = m_azimuthMotor.getStatorCurrent(false);

        m_azimuthEncoderPosition = m_azimuthEncoder.getPosition(false);
        m_azimuthEncoderVelocity = m_azimuthEncoder.getVelocity(false);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            m_driveMotorPosition,
            m_driveMotorVelocity,
            m_driveMotorAcceleration,
            m_driveMotorAppliedVoltage,
            m_driveMotorStatorCurrent,

            m_azimuthMotorPosition,
            m_azimuthMotorVelocity,
            m_azimuthMotorAcceleration,
            m_azimuthMotorAppliedVoltage,
            m_azimuthMotorStatorCurrent
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            m_azimuthEncoderPosition,
            m_azimuthEncoderVelocity
        );

        ParentDevice.optimizeBusUtilizationForAll(m_driveMotor, m_azimuthMotor, m_azimuthEncoder);
    }

    @Override
    public final void updateInputs(SwerveModuleInputs loggableInputs) {
        loggableInputs.isDriveMotorConnected = BaseStatusSignal.refreshAll(
            m_driveMotorPosition,
            m_driveMotorVelocity,
            m_driveMotorAcceleration,
            m_driveMotorAppliedVoltage,
            m_driveMotorStatorCurrent
        ).equals(StatusCode.OK);

        loggableInputs.driveMotorPosition = m_driveMotorPosition.getValue();
        loggableInputs.driveMotorVelocity = m_driveMotorVelocity.getValue();
        loggableInputs.driveMotorAcceleration = m_driveMotorAcceleration.getValue();
        loggableInputs.driveMotorAppliedVoltage = m_driveMotorAppliedVoltage.getValue();
        loggableInputs.driveMotorStatorCurrent = m_driveMotorStatorCurrent.getValue();

        loggableInputs.isAzimuthMotorConnected = BaseStatusSignal.refreshAll(
            m_azimuthMotorPosition,
            m_azimuthMotorVelocity,
            m_azimuthMotorAcceleration,
            m_azimuthMotorAppliedVoltage,
            m_azimuthMotorStatorCurrent
        ).equals(StatusCode.OK);

        loggableInputs.azimuthMotorPosition = m_azimuthMotorPosition.getValue();
        loggableInputs.azimuthMotorVelocity = m_azimuthMotorVelocity.getValue();
        loggableInputs.azimuthMotorAcceleration = m_azimuthMotorAcceleration.getValue();
        loggableInputs.azimuthMotorAppliedVoltage = m_azimuthMotorAppliedVoltage.getValue();
        loggableInputs.azimuthMotorStatorCurrent = m_azimuthMotorStatorCurrent.getValue();

        loggableInputs.isAzimuthEncoderConnected = BaseStatusSignal.refreshAll(
            m_azimuthEncoderPosition,
            m_azimuthEncoderVelocity
        ).equals(StatusCode.OK);

        loggableInputs.azimuthEncoderPosition = m_azimuthEncoderPosition.getValue();
        loggableInputs.azimuthEncoderVelocity = m_azimuthEncoderVelocity.getValue();
    }

    @Override
    public final void setDriveVelocity(LinearVelocity linearVelocity) {
        AngularVelocity motorVelocity = RotationsPerSecond.of(
            linearVelocity.in(MetersPerSecond) / kDriveMotorToWheelFactor
        );

        m_driveMotor.setControl(
            m_driveMotorControlRequest.withVelocity(motorVelocity)
        );
    }

    @Override
    public final void setDriveAngle(Angle angle) {
        m_azimuthMotor.setControl(
            m_azimuthMotorControlRequest.withPosition(angle)
        );
    }

    @Override
    public final void setDriveMotorVoltage(Voltage voltage) {
        m_driveMotor.setVoltage(voltage.in(Volts));
    }

    @Override
    public final void setAzimuthMotorVoltage(Voltage voltage) {
        m_azimuthMotor.setVoltage(voltage.in(Volts));
    }
}
