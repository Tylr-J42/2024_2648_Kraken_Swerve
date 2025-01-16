package frc.robot.constants;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class PhotonConstants {
    public static final String kCameraName = "Camera_Module_v1";

    // TODO Set this to something real if visual pose estimation is used
    public static final Transform3d kCameraTransform = new Transform3d();

    // TODO The camera will be moved, this is an old value that needs to update
    public static final double kCameraHeightMeters = .517525;

    // TODO The camera will be moved, this is an old value that needs to update
    public static final double kCameraPitchRadians = Units.degreesToRadians(15);
}
