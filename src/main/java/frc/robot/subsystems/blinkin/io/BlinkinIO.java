package frc.robot.subsystems.blinkin.io;

import org.littletonrobotics.junction.AutoLog;

/**
 * The generic REV Blinkin IO interface (with support for AdvantageKit).
 * 
 * @see frc.robot.subsystems.template.io.TemplateIO The IO interface template
 * @see https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/
*/
public abstract interface BlinkinIO {
    @AutoLog
    public static abstract class BlinkinInputs {
        public boolean isConnected = false;
        public double currentDutyCycle = 0.0;
    }

    /**
     * Updates the supplied inputs with values determined by the implementation.
     * <p> After updating, the inputs should be logged via {@code Logger.processInputs()}
     * 
     * @param loggableInputs The loggable inputs to update.
    */
    public default void updateInputs(BlinkinInputs loggableInputs) {}

    /**
     * Sets the color and/or pattern of the connected LEDs from a given duty cycle.
     * 
     * @param dutyCycle The corresponding duty cycle of the color and/or pattern.
     * @see https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf#page=14
    */
    public default void setPattern(double dutyCycle) {}
}
