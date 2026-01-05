package frc.robot.subsystems.blinkin.io;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public final class BlinkinIOReal implements BlinkinIO {
    private final Spark m_blinkin;

    public BlinkinIOReal(int blinkinPWM) {
        m_blinkin = new Spark(blinkinPWM);
    }

    @Override
    public final void updateInputs(BlinkinInputs loggableInputs) {
        loggableInputs.isConnected = m_blinkin.isAlive();
        loggableInputs.currentDutyCycle = m_blinkin.get();
    }

    @Override
    public final void setPattern(double dutyCycle) {
        m_blinkin.set(dutyCycle);
    }
}
