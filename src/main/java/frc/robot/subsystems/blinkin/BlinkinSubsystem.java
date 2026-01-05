package frc.robot.subsystems.blinkin;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.blinkin.BlinkinConstants.BlinkinPattern;
import frc.robot.subsystems.blinkin.io.BlinkinIO;
import frc.robot.subsystems.blinkin.io.BlinkinInputsAutoLogged;

public final class BlinkinSubsystem extends SubsystemBase {
    private final BlinkinIO m_blinkinIO;
    private final BlinkinInputsAutoLogged m_blinkinInputs = new BlinkinInputsAutoLogged();

    public BlinkinSubsystem(BlinkinIO blinkinIO) {
        m_blinkinIO = blinkinIO;
    }

    @Override
    public final void periodic() {
        m_blinkinIO.updateInputs(m_blinkinInputs);
        Logger.processInputs("BlinkinSubsystem", m_blinkinInputs);
    }

    public final Command getSetColorAllianceCommand() {
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red) {
                m_blinkinIO.setPattern(BlinkinPattern.SOLID_DARK_RED.getDutyCycle());
            } else {
                m_blinkinIO.setPattern(BlinkinPattern.SOLID_DARK_BLUE.getDutyCycle());
            }
        });
    }

    public final Command getSetColorGreenCommand() {
        return Commands.runOnce(() -> {
            m_blinkinIO.setPattern(BlinkinPattern.SOLID_DARK_GREEN.getDutyCycle());
        });
    }
}
