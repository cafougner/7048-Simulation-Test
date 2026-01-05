package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;

public final class ElevatorConstants {
    public static final Distance kElevatorDrumRadius = Inches.of(1.5);
    public static final double kElevatorGearReduction = 0.71439715284;
    public static final Distance kElevatorMaxHeight = Inches.of(60.0);

    public static final Distance kElevatorHeightTolerance = Inches.of(0.25);

    public static enum ElevatorSetpoints {
        L4      (Inches.of(60.0)),
        L3      (Inches.of(34.0)),
        L2      (Inches.of(19.25)),
        L1      (Inches.of(5.0)),

        CLIMB   (Inches.of(8.0)),
        INTAKE  (Inches.of(0.0)),
        IDLE    (Inches.of(0.0));

        private final Distance m_elevatorSetpoint;

        ElevatorSetpoints(Distance elevatorSetpoint) {
            m_elevatorSetpoint = elevatorSetpoint;
        }

        public Distance getValue() {return m_elevatorSetpoint;}
    }

    public static final double kElevatorMetersToRotationsFactor =
        kElevatorGearReduction / (2.0 * Math.PI * kElevatorDrumRadius.in(Meters));

    public static final double kElevatorRotationsToMetersFactor = 1.0 / kElevatorMetersToRotationsFactor;

    public static final TalonFXConfiguration kElevatorMotorConfiguration = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(60.0))
            .withSupplyCurrentLimit(Amps.of(60.0))
        ).withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
        ).withSlot0(new Slot0Configs()
            .withKP(5.0)
        ).withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicAcceleration(2.54 * kElevatorMetersToRotationsFactor)
            .withMotionMagicCruiseVelocity(3.556 * kElevatorMetersToRotationsFactor)
        );

    private ElevatorConstants() {}
}
