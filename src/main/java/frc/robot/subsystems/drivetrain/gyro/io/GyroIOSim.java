package frc.robot.subsystems.drivetrain.gyro.io;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

import com.ctre.phoenix6.sim.Pigeon2SimState;

public final class GyroIOSim extends GyroIOBase {
    private final GyroSimulation m_gyroSimulation;
    private final Pigeon2SimState m_pigeonSimState;

    public GyroIOSim(GyroSimulation gyroSimulation, int gyroID) {
        super(gyroID);
        m_gyroSimulation = gyroSimulation;
        m_pigeonSimState = m_gyro.getSimState();
    }

    @Override
    public final void updateInputs(GyroInputs loggableInputs) {
        m_pigeonSimState.setRawYaw(m_gyroSimulation.getGyroReading().getDegrees());
        m_pigeonSimState.setAngularVelocityZ(m_gyroSimulation.getMeasuredAngularVelocity());
        m_pigeonSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());

        super.updateInputs(loggableInputs);
    }
}
