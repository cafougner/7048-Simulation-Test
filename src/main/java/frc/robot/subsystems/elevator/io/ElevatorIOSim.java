package frc.robot.subsystems.elevator.io;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import org.ironmaple.simulation.motorsims.SimulatedBattery;

import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot;

public final class ElevatorIOSim extends ElevatorIOBase {
    private final TalonFXSimState m_elevatorMotorSimState;

    // private final DCMotorSim test = new DCMotorSim(
    //     LinearSystemId.createElevatorSystem(
    //         DCMotor.getKrakenX60(1),
    //         Units.lbsToKilograms(0.005),
    //         kElevatorDrumRadius.in(Meters),
    //         1.0 / kElevatorGearReduction
    //     ),

    //     DCMotor.getKrakenX60(1)
    // );

    private final ElevatorSim m_elevatorSimulation = new ElevatorSim(
        DCMotor.getKrakenX60(1),
        1.0 / kElevatorGearReduction,
        Units.lbsToKilograms(0.005),
        kElevatorDrumRadius.in(Meters),
        0.0,
        kElevatorMaxHeight.in(Meters),
        true,
        0.0
    );

    public ElevatorIOSim(int elevatorMotorID) {
        super(elevatorMotorID);

        m_elevatorMotorSimState = m_elevatorMotor.getSimState();
        SimulatedBattery.addElectricalAppliances(() -> Amps.of(m_elevatorSimulation.getCurrentDrawAmps()));
        //SimulatedBattery.addElectricalAppliances(() -> Amps.of(test.getCurrentDrawAmps()));
    }

    @Override
    public final void updateInputs(ElevatorInputs loggableInputs) {
        m_elevatorMotorSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
        //test.setInputVoltage(m_elevatorMotorSimState.getMotorVoltage());
        //test.update(Robot.defaultPeriodSecs);
        m_elevatorSimulation.setInputVoltage(m_elevatorMotorSimState.getMotorVoltage());
        m_elevatorSimulation.update(Robot.defaultPeriodSecs);

        m_elevatorMotorSimState.setRawRotorPosition(m_elevatorSimulation.getPositionMeters() * kElevatorMetersToRotationsFactor);
        m_elevatorMotorSimState.setRotorVelocity(m_elevatorSimulation.getVelocityMetersPerSecond() * kElevatorMetersToRotationsFactor);
        ///m_elevatorMotorSimState.setRawRotorPosition(test.getAngularPosition());
        ///m_elevatorMotorSimState.setRotorVelocity(test.getAngularVelocity());
        ///m_elevatorMotorSimState.setRotorAcceleration(test.getAngularAcceleration());

        super.updateInputs(loggableInputs);
    }
}
