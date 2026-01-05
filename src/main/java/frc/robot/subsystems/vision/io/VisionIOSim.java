package frc.robot.subsystems.vision.io;

import static frc.robot.RobotConstants.kRobotBehavior;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import frc.robot.RobotConstants.RobotBehavior;
import frc.robot.utils.FieldUtils;

public final class VisionIOSim extends VisionIOBase {
    private final PhotonCameraSim m_cameraSimulation;
    private final Supplier<Pose2d> m_simulationPoseSupplier;
    private static final VisionSystemSim m_visionSimulation;

    static {
        if (kRobotBehavior == RobotBehavior.SIMULATION) {
            m_visionSimulation = new VisionSystemSim("PhotonVision");
            m_visionSimulation.addAprilTags(FieldUtils.getFieldAprilTags());
        } else {
            m_visionSimulation = null;
        }
    }

    public VisionIOSim(
        String cameraName,
        Transform3d cameraTransform,
        SimCameraProperties cameraProperties,
        Supplier<Pose2d> simulationPoseSupplier
    ) {
        super(cameraName, cameraTransform);
        m_cameraSimulation = new PhotonCameraSim(m_camera, cameraProperties);
        m_simulationPoseSupplier = simulationPoseSupplier;

        m_visionSimulation.addCamera(m_cameraSimulation, cameraTransform);
    }

    @Override
    public final void updateInputs(VisionInputs loggableInputs, Pose2d estimatedPose) {
        m_visionSimulation.update(m_simulationPoseSupplier.get());
        super.updateInputs(loggableInputs, estimatedPose);
    }
}
