package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class AmpHandoff extends ParallelCommandGroup{

    AmpHandoff(Indexer indexer, Shooter shooter){
        //addCommands(indexer.shootNote(() -> 1), shooter.ampHandoff());
    }
    
}
