package frc.robot.utilities;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.interfaces.IAprilTagProvider;
import frc.robot.interfaces.IVisualPoseProvider;

public class PhotonVision implements IAprilTagProvider,IVisualPoseProvider {

    private final PhotonCamera camera;

    private final PhotonPoseEstimator photonPoseEstimator;

    private final double cameraHeightMeters;
    private final double cameraPitchRadians;

    public PhotonVision(String cameraName, Transform3d robotToCam, double cameraHeightMeters, double cameraPitchRadians) throws IOException {
        camera = new PhotonCamera(cameraName);

        photonPoseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2024Crescendo.m_resourceFile
            ), 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            camera,
            robotToCam
        );

        this.cameraHeightMeters = cameraHeightMeters;
        this.cameraPitchRadians = cameraPitchRadians;
    }

    @Override
    public Optional<VisualPose> getVisualPose() {
        Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();

        if (pose.isEmpty()) {
            return Optional.empty();
        }

        return Optional.of(
            new VisualPose(
                pose.get().estimatedPose.toPose2d(), 
                pose.get().timestampSeconds
            )
        );
    }

    @Override
    public OptionalDouble getTagDistanceFromCameraByID(int id, double targetHeightMeters) {
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (!result.hasTargets()) {
            return OptionalDouble.empty();
        }

        Optional<PhotonTrackedTarget> desiredTarget = getTargetFromList(result.getTargets(), id);

        if (desiredTarget.isEmpty()) {
            return OptionalDouble.empty();
        }

        return OptionalDouble.of(
            PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeightMeters, 
                targetHeightMeters, 
                cameraPitchRadians, 
                Units.degreesToRadians(desiredTarget.get().getPitch()))
        );
    }

    @Override
    public OptionalDouble getTagPitchByID(int id) {
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (!result.hasTargets()) {
            return OptionalDouble.empty();
        }

        Optional<PhotonTrackedTarget> desiredTarget = getTargetFromList(result.getTargets(), id);

        if (desiredTarget.isEmpty()) {
            return OptionalDouble.empty();
        }

        return OptionalDouble.of(
            desiredTarget.get().getPitch()
        );
    }

    @Override
    public OptionalDouble getTagYawByID(int id) {
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (!result.hasTargets()) {
            return OptionalDouble.empty();
        }

        Optional<PhotonTrackedTarget> desiredTarget = getTargetFromList(result.getTargets(), id);

        if (desiredTarget.isEmpty()) {
            return OptionalDouble.empty();
        }

        return OptionalDouble.of(
            desiredTarget.get().getYaw()
        );
    }

    private Optional<PhotonTrackedTarget> getTargetFromList(List<PhotonTrackedTarget> targets, int id) {
        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() == id) {
                return Optional.of(target);
            }
        }

        return Optional.empty();
    }
    
}
