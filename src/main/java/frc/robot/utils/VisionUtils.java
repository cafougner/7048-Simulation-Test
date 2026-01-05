package frc.robot.utils;

import org.photonvision.simulation.VisionSystemSim;

public final class VisionUtils {
    private static VisionSystemSim m_visionSimulation;

    public static final VisionSystemSim getVisionSimulation() {
        if (m_visionSimulation == null) {
            m_visionSimulation = new VisionSystemSim("PhotonVision");
        }

        return m_visionSimulation;
    }
}
