package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;

public class Indexer extends SubsystemBase{
    
    private CANSparkMax indexerMotor;
    private DigitalInput indexerBeamBreak;

    public Indexer(){
        indexerMotor = new CANSparkMax(IndexerConstants.kIndexerID, MotorType.kBrushed);

        indexerMotor.setSmartCurrentLimit(40);
        indexerMotor.setIdleMode(IdleMode.kBrake);
        indexerMotor.burnFlash();

        indexerBeamBreak = new DigitalInput(IndexerConstants.kIndexerBeamBreakChannel);
    }

    public Command autoIndexing(){
     return run(() -> {
        if(!indexerBeamBreak.get()){
            indexerMotor.set(0.75);
        }else if(indexerBeamBreak.get()){
            indexerMotor.set(0.0);
        }

     });
    }

    public Command advanceNote(){
        return run(() -> {
            if(indexerBeamBreak.get()){
                indexerMotor.set(1);
            }else{
                indexerMotor.set(0.0);
            }
        });
    }
    
    public Command shootNote(DoubleSupplier indexerSpeed){
        return run(() -> {
            indexerMotor.set(indexerSpeed.getAsDouble());
        });
    }

    public boolean getBeamBreak(){
        return indexerBeamBreak.get();
    }
}
