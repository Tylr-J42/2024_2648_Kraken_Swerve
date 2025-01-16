package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    private CANSparkMax topShooter;
    private CANSparkMax bottomShooter;

    private CANSparkMax shooterPivot;

    private Encoder pivotEncoder;

    private PIDController shooterPivotPID;
    private ArmFeedforward shooterPivotFF;

    private DigitalInput shooterBeamBreak;

    private boolean indexerBeamBreak;

    public Shooter(BooleanSupplier indexerBeamBreak){
        topShooter = new CANSparkMax(ShooterConstants.kTopShooterID, MotorType.kBrushless);
        topShooter.setInverted(true);
        bottomShooter = new CANSparkMax(ShooterConstants.kBottomShooterID, MotorType.kBrushless);
        bottomShooter.setInverted(true);

        shooterPivot = new CANSparkMax(ShooterConstants.kShooterPivotID, MotorType.kBrushless);
        shooterPivot.setInverted(true);

        /* 
        topPID = topShooter.getPIDController();
        topPID.setFeedbackDevice(topEncoder);
        bottomPID = bottomShooter.getPIDController();
        bottomPID.setFeedbackDevice(bottomEncoder);
        */

        shooterPivot.setSmartCurrentLimit(50);
        topShooter.setSmartCurrentLimit(40);
        bottomShooter.setSmartCurrentLimit(40);

        pivotEncoder = new Encoder(0, 1);
        pivotEncoder.setDistancePerPulse(ShooterConstants.kPivotConversion);

        shooterBeamBreak = new DigitalInput(ShooterConstants.kShooterBeamBreakChannel);

        topShooter.setIdleMode(IdleMode.kCoast);
        bottomShooter.setIdleMode(IdleMode.kCoast);

        bottomShooter.burnFlash();
        shooterPivot.burnFlash();
        topShooter.burnFlash();

        shooterPivotPID = new PIDController(
            ShooterConstants.kShooterPivotP,
            ShooterConstants.kShooterPivotI,
            ShooterConstants.kShooterPivotD
        );
        shooterPivotFF = new ArmFeedforward(ShooterConstants.kSShooterPivotFF, ShooterConstants.kGShooterPivotFF, ShooterConstants.kVShooterPivotFF);

    }

    /*
    public Command angleSpeedsSetpoints(double setpointAngle, double topRPM, double bottomRPM){
        return run(()-> {
            angleAndSpeedControl(setpointAngle, topRPM, bottomRPM);
        });
    }*/

    public Command angleSpeedsSetpoints(DoubleSupplier setpointAngle, DoubleSupplier speed){
        return run(()-> {
            angleAndSpeedControl(setpointAngle.getAsDouble(), speed.getAsDouble(), speed.getAsDouble());
        });
    }

    public Command angleSpeedsSetpoints(DoubleSupplier setpointAngle, double topRPM, double bottomRPM){
        return run(()-> {
            angleAndSpeedControl(setpointAngle.getAsDouble(), topRPM, bottomRPM);
        });
    }

    private void angleAndSpeedControl(double setpointAngle, double topRPM, double bottomRPM){
        shooterPivot.setIdleMode(IdleMode.kBrake);

            shooterPivot.setVoltage(
                shooterPivotPID.calculate(getShooterAngle(), setpointAngle) + 
                shooterPivotFF.calculate(setpointAngle, 0.0));

            //topPID.setReference(topRPM, CANSparkMax.ControlType.kVelocity);
            //bottomPID.setReference(bottomRPM, CANSparkMax.ControlType.kVelocity);
            topShooter.set(topRPM);
            bottomShooter.set(bottomRPM);
    }

    public Command ampHandoff(DoubleSupplier shootNoteSpeed, BooleanSupplier ampAngleOrNah){
        return run(() -> {
            
            shooterPivot.setIdleMode(IdleMode.kBrake);

            shooterPivot.setVoltage(
                shooterPivotPID.calculate(getShooterAngle(), ShooterConstants.kShooterLoadAngle) + 
                shooterPivotFF.calculate(ShooterConstants.kShooterLoadAngle, 0.0));
            
            if(shooterBeamBreak.get() || indexerBeamBreak){
                angleAndSpeedControl(ShooterConstants.kShooterLoadAngle, 0.25, 0.25);
            }else{
                angleAndSpeedControl(ShooterConstants.kShooterLoadAngle, shootNoteSpeed.getAsDouble(), shootNoteSpeed.getAsDouble());
            }
        });
    }

    public Command climbState(){
        return run(() -> {
            shooterPivot.setIdleMode(IdleMode.kCoast);
            shooterPivot.set(0.0);
        });
    }

    public double getShooterAngle(){
        return pivotEncoder.getDistance() + ShooterConstants.kShooterLoadAngle;
    }

    public Command zeroEncoder(){
        return run(() -> {
            pivotEncoder.reset();
        });
    }

    public Boolean getBeamBreak(){
        return shooterBeamBreak.get();
    }

    public Command manualPivot(DoubleSupplier pivotPower, DoubleSupplier spinny){
        return run(() ->{
            shooterPivot.setIdleMode(IdleMode.kBrake);
            if(getShooterAngle() >= ShooterConstants.kShooterLoadAngle && getShooterAngle() <= ShooterConstants.kShooterAmpAngle){
                shooterPivot.set(pivotPower.getAsDouble());
            }else if(getShooterAngle() > ShooterConstants.kShooterAmpAngle){
                shooterPivot.set(MathUtil.clamp(pivotPower.getAsDouble(), -1.0, 0.0));
            }else if(getShooterAngle() < ShooterConstants.kShooterLoadAngle){
                shooterPivot.set(MathUtil.clamp(pivotPower.getAsDouble(), 0.0, 1.0));
            }

            topShooter.set(spinny.getAsDouble());
            bottomShooter.set(spinny.getAsDouble());
            
        });
    }


}
