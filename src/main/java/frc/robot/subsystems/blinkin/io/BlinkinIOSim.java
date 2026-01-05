package frc.robot.subsystems.blinkin.io;

public final class BlinkinIOSim implements BlinkinIO {
    private double m_currentDutyCycle = 0.0;

    public BlinkinIOSim() {}

    @Override
    public final void updateInputs(BlinkinInputs loggableInputs) {
        loggableInputs.isConnected = true;
        loggableInputs.currentDutyCycle = m_currentDutyCycle;
    }

    @Override
    public final void setPattern(double dutyCycle) {
        m_currentDutyCycle = dutyCycle;
    }
}
