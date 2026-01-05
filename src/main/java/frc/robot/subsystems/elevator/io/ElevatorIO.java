package frc.robot.subsystems.elevator.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

/**
 * The generic elevator IO interface (with support for AdvantageKit).
 * 
 * @see frc.robot.subsystems.template.io.TemplateIO The IO interface template
 * @see https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/
*/
public abstract interface ElevatorIO {
    @AutoLog
    public static abstract class ElevatorInputs {
        public boolean isElevatorMotorConnected = false;
        public Angle elevatorMotorPosition = Radians.of(0.0);
        public AngularVelocity elevatorMotorVelocity = RadiansPerSecond.of(0.0);
        public AngularAcceleration elevatorMotorAcceleration = RadiansPerSecondPerSecond.of(0.0);
        public Voltage elevatorMotorAppliedVoltage = Volts.of(0.0);
        public Current elevatorMotorStatorCurrent = Amps.of(0.0);
    }

    /**
     * Updates the supplied loggable inputs with values determined by the implementation.
     * <p> After updating, the inputs should be logged via {@code Logger.processInputs()}.
     * 
     * @param loggableInputs The loggable inputs to update.
    */
    public default void updateInputs(ElevatorInputs loggableInputs) {}

    /**
     * Sets the elevator motor's onboard closed-loop position setpoint from a given height.
     * 
     * @param height The desired height of the elevator relative to its zero.
    */
    public default void setElevatorHeight(Distance height) {}

    /**
     * Directly applies a voltage to the elevator motor.
     * <p> This will cancel any onboard closed-loop control.
     * 
     * @param voltage The voltage to apply to the elevator motor.
    */
    public default void setElevatorMotorVoltage(Voltage voltage) {}

    /**
     * Updates the position of the elevator motor's encoder.
     * 
     * @param position The new position of the elevator motor's encoder.
    */
    public default void setElevatorMotorPosition(Angle position) {}
}
