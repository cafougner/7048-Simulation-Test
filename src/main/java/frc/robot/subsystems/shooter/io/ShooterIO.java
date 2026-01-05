package frc.robot.subsystems.shooter.io;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;
/**
 * The generic shooter IO interface (with support for AdvantageKit).
 * 
 * @see frc.robot.subsystems.template.io.TemplateIO The IO interface template
 * @see https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/
*/
public abstract interface ShooterIO {
    @AutoLog
    public static abstract class ShooterInputs {
        public boolean isShooterMotorConnected = false;
        public Angle shooterMotorPosition = Radians.of(0.0);
        public AngularVelocity shooterMotorVelocity = RadiansPerSecond.of(0.0);
        public AngularAcceleration shooterMotorAcceleration = RadiansPerSecondPerSecond.of(0.0);
        public Voltage shooterMotorAppliedVoltage = Volts.of(0.0);
        public Current shooterMotorStatorCurrent = Amps.of(0.0);
    }

    /**
     * Updates the supplied loggable inputs with values determined by the implementation.
     * <p> After updating, the inputs should be logged via {@code Logger.processInputs()}.
     * 
     * @param loggableInputs The loggable inputs to update.
    */
    public default void updateInputs(ShooterInputs loggableInputs) {}
}
