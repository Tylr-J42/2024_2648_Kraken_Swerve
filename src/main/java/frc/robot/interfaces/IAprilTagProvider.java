package frc.robot.interfaces;

import java.util.OptionalDouble;

/**
 * An interface which ensures a class can provide common AprilTag oriented
 * information from various sources in a consistent way.
 */
public interface IAprilTagProvider {
    /**
     * A method to get the distance from <i>the camera</i> to the AprilTag specified
     * 
     * @param id The ID of the AprilTag to give a distance to
     * @param targetHeightMeters The height of the AprilTag off the ground, in meters
     * @return The distance, in meters, to the target, or OptionalDouble.empty() if the tag is not present in the camera's view
     */
    public OptionalDouble getTagDistanceFromCameraByID(int id, double targetHeightMeters);

    /**
     * A method to get the pitch from the center of the image of a particular AprilTag
     * 
     * @param id The ID of the AprilTag to get the pitch of
     * @return The pitch, in degrees, of the target, or OptionalDouble.empty() if the tag is not present in the camera's view
     */
    public OptionalDouble getTagPitchByID(int id);

    /**
     * A method to get the yaw from the center of the image of a particular AprilTag
     * 
     * @param id The ID of the AprilTag to get the yaw of
     * @return The yaw, in degrees, of the target, or OptionalDouble.empty() if the tag is not present in the camera's view
     */
    public OptionalDouble getTagYawByID(int id);
}
