package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Transform3d;

public final class RobotVisualizer {
    private static Transform3d m_firstStageTransform = new Transform3d();
    private static Transform3d m_secondStageTransform = new Transform3d();
    private static Transform3d m_shooterTransform = new Transform3d();

    public static final void updateComponents() {
        // For now, components cannot be "linked" in AdvantageScope; the transform
        // of each component needs to be manually added to all connected components.
        Logger.recordOutput(
            "RobotComponents",
            m_firstStageTransform,
            m_firstStageTransform.plus(m_secondStageTransform),
            m_firstStageTransform.plus(m_secondStageTransform).plus(m_shooterTransform)
        );
    }

    public static final void setFirstStageTransform(Transform3d firstStageTransform) {
        m_firstStageTransform = firstStageTransform;
    }

    public static final void setSecondStageTransform(Transform3d secondStageTransform) {
        m_secondStageTransform = secondStageTransform;
    }

    public static final void setShooterTransform(Transform3d shooterTransform) {
        m_shooterTransform = shooterTransform;
    }
}
