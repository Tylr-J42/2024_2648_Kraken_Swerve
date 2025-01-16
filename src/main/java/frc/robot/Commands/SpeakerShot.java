package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class SpeakerShot extends ParallelCommandGroup{
    
    SpeakerShot(Indexer indexer, Shooter shooter){
        //addCommands(shooter.angleSpeedsSetpoints(ShooterConstants.kShooterLoadAngle, 0, 0), indexer.shootNote(() -> 1.0));
    }
}
