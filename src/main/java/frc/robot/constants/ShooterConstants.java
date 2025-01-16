package frc.robot.constants;

public class ShooterConstants {
    
    public static final int kTopShooterID = 9;
    public static final int kBottomShooterID = 11;
    public static final int kShooterPivotID = 16;

    public static final double kShooterP = 0.0;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;

    public static final double kShooterFF = 0.0;

    public static final double kShooterPivotP = 3.0;
    public static final double kShooterPivotI = 0.0;
    public static final double kShooterPivotD = 0.0;

    public static final double kShooterLoadAngle = Math.toRadians(-20.0);
    public static final double kShooterAmpAngle = Math.toRadians(105.0);

    public static final double kPivotConversion = 1/(40.0*(28.0/15.0));

    public static final double kSShooterPivotFF = 0.0;
    public static final double kGShooterPivotFF = 0.33;
    public static final double kVShooterPivotFF = 1.44;

    public static final double kMaxPivotSpeed = 0.0;
    public static final double kMaxPivotAcceleration = 0.0;

    public static final int kShooterEncoderChannelA = 0;
    public static final int kShooterEncoderChannelB = 1;
    public static final int kShooterBeamBreakChannel = 3;
}
