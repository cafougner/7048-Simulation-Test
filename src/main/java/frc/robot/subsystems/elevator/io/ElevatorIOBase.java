package frc.robot.subsystems.elevator.io;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.*;

public abstract class ElevatorIOBase implements ElevatorIO {
    protected final TalonFX m_elevatorMotor;

    private final StatusSignal<Angle> m_elevatorMotorPosition;
    private final StatusSignal<AngularVelocity> m_elevatorMotorVelocity;
    private final StatusSignal<AngularAcceleration> m_elevatorMotorAcceleration;
    private final StatusSignal<Voltage> m_elevatorMotorAppliedVoltage;
    private final StatusSignal<Current> m_elevatorMotorStatorCurrent;

    private final MotionMagicVoltage m_elevatorMotorControlRequest = new MotionMagicVoltage(0.0);

    public ElevatorIOBase(int elevatorMotorID) {
        m_elevatorMotor = new TalonFX(elevatorMotorID);

        m_elevatorMotor.getConfigurator().apply(kElevatorMotorConfiguration);

        m_elevatorMotorPosition = m_elevatorMotor.getPosition(false);
        m_elevatorMotorVelocity = m_elevatorMotor.getVelocity(false);
        m_elevatorMotorAcceleration = m_elevatorMotor.getAcceleration(false);
        m_elevatorMotorAppliedVoltage = m_elevatorMotor.getMotorVoltage(false);
        m_elevatorMotorStatorCurrent = m_elevatorMotor.getStatorCurrent(false);

        BaseStatusSignal.setUpdateFrequencyForAll(
            Hertz.of(50.0),
            m_elevatorMotorPosition,
            m_elevatorMotorVelocity,
            m_elevatorMotorAcceleration,
            m_elevatorMotorAppliedVoltage,
            m_elevatorMotorStatorCurrent
        );

        m_elevatorMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorInputs loggableInputs) {
        loggableInputs.isElevatorMotorConnected = BaseStatusSignal.refreshAll(
            m_elevatorMotorPosition,
            m_elevatorMotorVelocity,
            m_elevatorMotorAcceleration,
            m_elevatorMotorAppliedVoltage,
            m_elevatorMotorStatorCurrent
        ).equals(StatusCode.OK);

        loggableInputs.elevatorMotorPosition = m_elevatorMotorPosition.getValue();
        loggableInputs.elevatorMotorVelocity = m_elevatorMotorVelocity.getValue();
        loggableInputs.elevatorMotorAcceleration = m_elevatorMotorAcceleration.getValue();
        loggableInputs.elevatorMotorAppliedVoltage = m_elevatorMotorAppliedVoltage.getValue();
        loggableInputs.elevatorMotorStatorCurrent = m_elevatorMotorStatorCurrent.getValue();
    }

    @Override
    public void setElevatorHeight(Distance height) {
        Angle motorAngle = Rotations.of(height.in(Meters) * kElevatorMetersToRotationsFactor);

        m_elevatorMotor.setControl(
            m_elevatorMotorControlRequest.withPosition(motorAngle)
        );
    }

    @Override
    public void setElevatorMotorVoltage(Voltage voltage) {
        m_elevatorMotor.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setElevatorMotorPosition(Angle position) {
        m_elevatorMotor.setPosition(position);
    }
}
