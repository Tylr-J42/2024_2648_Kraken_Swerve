package frc.robot.constants;

public class IntakeConstants {
    public static final int kIntakePivotID = 10;
    public static final int kIntakeRollerID = 12;

    public static final double kIntakePivotConversionFactor = (20.0*(28.0/15.0));

    public static final int kPivotCurrentLimit = 40;

    public static final double kPIntake = 3;//4; //2.5;
    public static final double kIIntake = 0;
    public static final double kDIntake = 0.01;

    public static final double kSFeedForward = 0;
    public static final double kGFeedForward = 1;//1.11;
    public static final double kVFeedForward = .5;//0.73;

    public static final double kStartingAngle = Math.toRadians(95.0);
    public static final double kUpAngle = Math.toRadians(90.0);
    public static final double kDownAngle = Math.toRadians(-34.0);

}
