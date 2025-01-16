package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.ShooterConstants;

public class Climber extends SubsystemBase {
    private VictorSPX motor1;
    private VictorSPX motor2;

    private DoubleSupplier shooterAngle;

    public Climber(DoubleSupplier shooterAngle) {
        motor1 = new VictorSPX(ClimberConstants.kClimberMotor1CANID);
        motor2 = new VictorSPX(ClimberConstants.kClimberMotor2CANID);
        

        motor2.follow(motor1);
        motor1.setInverted(true);
        motor2.setInverted(true);
        this.shooterAngle = shooterAngle;
    }

    public Command setSpeedWithSupplier(DoubleSupplier speed) {
        return run(() -> {
            if(shooterAngle.getAsDouble() > ShooterConstants.kShooterLoadAngle){
           
                motor1.set(VictorSPXControlMode.PercentOutput, speed.getAsDouble());
                 
            }else{
                motor1.set(VictorSPXControlMode.PercentOutput, 0.0);
            }
        });
    }

    public Command setSpeed(double speed) {
        return run(() -> {
            motor1.set(VictorSPXControlMode.PercentOutput, speed);
        });
    }

    public Command stop() {
        return setSpeed(0);
    }
}
