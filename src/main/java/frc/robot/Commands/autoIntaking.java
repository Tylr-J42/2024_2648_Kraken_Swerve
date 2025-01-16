package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class autoIntaking extends Command{
    private Intake m_intake;
    private BooleanSupplier m_beamBreak;

    public autoIntaking(Intake intake, BooleanSupplier beamBreak){
        m_intake = intake;
        m_beamBreak = beamBreak;

        addRequirements(intake);
    }    

    @Override
    public void execute(){
        m_intake.intakeControl(() -> IntakeConstants.kDownAngle, () -> 1.0);
    }

    @Override
    public boolean isFinished(){
        if(!m_beamBreak.getAsBoolean()){
            return true;
        }else {return false;}
    }

}
