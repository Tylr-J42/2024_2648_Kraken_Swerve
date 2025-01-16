package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase{
    private PIDController intakeAnglePID;

    private CANSparkMax intakeRoller;
    private CANSparkMax intakePivot;

    private RelativeEncoder intakeEncoder;

    private ArmFeedforward intakeFeedForward;

    private double armOffset;

    public Intake(){
        armOffset = 0;

        intakeRoller = new CANSparkMax(IntakeConstants.kIntakeRollerID, MotorType.kBrushless);
        
        intakeRoller.setSmartCurrentLimit(60);
        intakeRoller.setIdleMode(IdleMode.kBrake);
        intakeRoller.burnFlash();

        intakePivot = new CANSparkMax(IntakeConstants.kIntakePivotID, MotorType.kBrushless);

        intakePivot.setIdleMode(IdleMode.kBrake);
        intakePivot.setSmartCurrentLimit(IntakeConstants.kPivotCurrentLimit);
        intakePivot.setInverted(true);
        intakePivot.burnFlash();

        intakeFeedForward = new ArmFeedforward(
            IntakeConstants.kSFeedForward, 
            IntakeConstants.kGFeedForward, 
            IntakeConstants.kVFeedForward
        );

        intakeEncoder = intakePivot.getEncoder();
        intakeEncoder.setPosition(Units.radiansToRotations( IntakeConstants.kStartingAngle));
       // intakeEncoder.setPositionConversionFactor(IntakeConstants.kIntakePivotConversionFactor);

        intakeAnglePID = new PIDController(
            IntakeConstants.kPIntake, 
            IntakeConstants.kIIntake, 
            IntakeConstants.kDIntake
        );

        armOffset = getIntakeAngle()-IntakeConstants.kStartingAngle;
    }

    
    /* 
    public Command autoIntaking(BooleanSupplier beamBreak){
        return run(() -> {
            intakeRoller.set(1.0);

            intakePivot.setVoltage(
                intakeAnglePID.calculate(
                    getIntakeAngle(), 
                    IntakeConstants.kDownAngle
                ) + intakeFeedForward.calculate(
                    IntakeConstants.kDownAngle, 
                    0.0
                )
            );
        return isFinished(() -> {
            if(!beamBreak.getAsBoolean()){
                return true
            }
            });
        });
    }
*/
    public Command intakeControl(DoubleSupplier pivotAngle, DoubleSupplier intakeSpeed) {
        return run(() -> {
            intakeRoller.set(intakeSpeed.getAsDouble());

            intakePivot.setVoltage(
                intakeAnglePID.calculate(
                    getIntakeAngle(),
                    pivotAngle.getAsDouble()
                ) + intakeFeedForward.calculate(
                    pivotAngle.getAsDouble(), 
                    0.0
                )
            );
        });
    }

    public Command intakeDownCommand() {
        return run(() -> {
            intakeRoller.set(0.0);
        
            intakePivot.setVoltage(
                intakeAnglePID.calculate(
                    getIntakeAngle(), 
                    IntakeConstants.kDownAngle
                ) + intakeFeedForward.calculate(
                    IntakeConstants.kDownAngle, 
                    0.0
                )
            );
        });
    }
    

    public Command manualPivot(DoubleSupplier pivotPower, DoubleSupplier rollerSpinny){
        return run(() ->{
            intakePivot.set(pivotPower.getAsDouble());
            intakeRoller.set(rollerSpinny.getAsDouble());
        });
    }

    public Command climbingStateCommand() {
        return run(() -> {
            intakeRoller.set(0.0);

            intakePivot.setVoltage(
                intakeAnglePID.calculate(
                    getIntakeAngle(), 
                    IntakeConstants.kDownAngle
                ) + intakeFeedForward.calculate(
                    IntakeConstants.kDownAngle, 
                    0.0
                )
            );
        });
    }

    public Command intakeUpCommand() {
        return run(() -> {
            intakeRoller.set(0.0);

            intakePivot.setVoltage(
                intakeAnglePID.calculate(
                    getIntakeAngle(), 
                    IntakeConstants.kUpAngle
                ) + intakeFeedForward.calculate(
                    IntakeConstants.kUpAngle, 
                    0.0
                )
            );
        });
    }

    public Command stopAll() {
        return runOnce(() -> {
            intakeRoller.set(0);
            intakePivot.setVoltage(0);
        });
    }

    public double getIntakeAngle(){
        return Units.rotationsToRadians(intakeEncoder.getPosition()/IntakeConstants.kIntakePivotConversionFactor)-armOffset;
    }

    public double getIntakePID(){
        return intakeAnglePID.calculate(
                    getIntakeAngle(), 
                    IntakeConstants.kDownAngle
                ) + intakeFeedForward.calculate(
                    IntakeConstants.kDownAngle, 
                    0.0
                );
    }

    public double getIntakeDegrees(){
        return Math.toDegrees(getIntakeAngle());
    }
}
