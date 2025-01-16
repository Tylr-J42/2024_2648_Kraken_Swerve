package frc.robot.interfaces;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * An interface which ensures a class' ability to provide visual pose information
 * in a consistent way
 */
public interface IVisualPoseProvider {
    /**
     * A record that can contain the two elements necessary for a WPILIB
     * pose estimator to use the information from a vision system as part of a full
     * robot pose estimation
     */
    public record VisualPose(Pose2d visualPose, double timestamp) {}

    /**
     * Return a VisualPose or null if an empty Optional if none is available.
     * Implementation should provide an empty response if it's unable to provide
     * a reliable pose, or any pose at all.
     * 
     * @return An Optional containing a VisualPose, or empty if no VisualPose can reliably be provided
     */
    public Optional<VisualPose> getVisualPose();
}
