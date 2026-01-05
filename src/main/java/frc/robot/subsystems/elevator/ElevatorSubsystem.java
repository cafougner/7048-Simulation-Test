package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.elevator.io.ElevatorIO;
import frc.robot.subsystems.elevator.io.ElevatorInputsAutoLogged;
import frc.robot.utils.RobotVisualizer;

public final class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO m_elevatorIO;
    private final ElevatorInputsAutoLogged m_elevatorInputs = new ElevatorInputsAutoLogged();

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        m_elevatorIO = elevatorIO;
    }

    @Override
    public final void periodic() {
        m_elevatorIO.updateInputs(m_elevatorInputs);
        Logger.processInputs("ElevatorSubsystem", m_elevatorInputs);

        RobotVisualizer.setFirstStageTransform(new Transform3d(0.0, 0.0, getHeight().in(Meters) / 2.0, Rotation3d.kZero));
        RobotVisualizer.setSecondStageTransform(new Transform3d(0.0, 0.0, getHeight().in(Meters) / 2.0 * 0.73490813648, Rotation3d.kZero));
    }

    public final Distance getHeight() {
        return Meters.of(
            m_elevatorInputs.elevatorMotorPosition.in(Rotations) * kElevatorRotationsToMetersFactor
        );
    }

    public final Command goToHeight(Distance height, boolean wait) {
        if (wait) {
            return Commands.sequence(
                goToHeight(height, false),
                Commands.waitUntil(() -> getHeight().isNear(height, kElevatorHeightTolerance))
            ).ignoringDisable(true);
        }

        return Commands.runOnce(() -> m_elevatorIO.setElevatorHeight(height)).ignoringDisable(true);
    }

    public final Command goToSetpoint(ElevatorSetpoints setpoint, boolean wait) {
        return goToHeight(setpoint.getValue(), wait);
    }
}
