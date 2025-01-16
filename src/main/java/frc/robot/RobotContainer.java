// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.autoIntaking;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.PhotonConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.utilities.PhotonVision;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    private static final String kAutoTabName = "Autonomous";
    private static final String kTeleopTabName = "Teloperated";

    //private PhotonVision photonvision;

    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Climber climber;
    private Indexer indexer;

    private CommandXboxController driver;
    private CommandXboxController operator;

    private Trigger isTeleopTrigger;
    private Trigger isEnabledTrigger;
    private Trigger indexerBeamBreakWatcher;

    private SendableChooser<Command> autoChooser;

    private int ampTagID;

    // TODO There's more than one source tag, how do we want to handle this?
    private int sourceTagID;

    // TODO There's more than one speaker tag, how do we want to handle this?
    private int speakerTag;

    private boolean setAmpAngle;
    private boolean intakeDown;
    private boolean invertXYDrive;

    public RobotContainer() {
        /*try {
         
        photonvision = new PhotonVision(
            PhotonConstants.kCameraName, 
            PhotonConstants.kCameraTransform, 
            PhotonConstants.kCameraHeightMeters,
            PhotonConstants.kCameraPitchRadians
        );
        } catch (IOException e) {
        photonvision = null;
        }*/

        // TODO Need to provide a real initial pose
        // TODO Need to provide a real IAprilTagProvider, null means we're not using one at all
        // TODO Need to provide a real VisualPoseProvider, null means we're not using one at all
        drivetrain = new Drivetrain(new Pose2d(), null, null);

        intake = new Intake();

        indexer = new Indexer();

        shooter = new Shooter(indexer::getBeamBreak);

        climber = new Climber(shooter::getShooterAngle);

        NamedCommands.registerCommand(
            "Charge Shooter 2 Sec", 
            shooter.angleSpeedsSetpoints(
                () -> ShooterConstants.kShooterLoadAngle, 
                1.0, 
                1.0
            ).withTimeout(2.0)
        );

        /*
        NamedCommands.registerCommand(
            "Speaker Note Shot", 
            Commands.parallel(
                shooter.angleSpeedsSetpoints(
                    () -> ShooterConstants.kShooterLoadAngle, 
                    1.0, 
                    1.0
                ), 
                indexer.shootNote(() -> 1.0)
            ).withTimeout(1.0)
        );
        */

        NamedCommands.registerCommand(
            "Speaker Note Shot", 
            shooter.angleSpeedsSetpoints(
                () -> ShooterConstants.kShooterLoadAngle, 
                1.0, 
                1.0
            ).raceWith(
                new WaitCommand(1.0)
            ).andThen(
                shooter.angleSpeedsSetpoints(
                    () -> ShooterConstants.kShooterLoadAngle, 
                    1.0, 
                    1.0
                ).alongWith(indexer.shootNote(() -> 1.0))
                .withTimeout(0.75)
            )
        );

        /* 
        NamedCommands.registerCommand(
            "Auto Intake", 
            Commands.parallel(
                intake.intakeControl(
                    () -> IntakeConstants.kDownAngle, 
                    () -> 1.0
                ), 
                indexer.advanceNote()
            ).until(() -> !indexer.getBeamBreak())
            .withTimeout(1.0)
        );*/

        NamedCommands.registerCommand(
            "Auto Intake", 
            intake.intakeControl(
                () -> IntakeConstants.kDownAngle, 
                () -> 1.0
            ).alongWith(
                indexer.advanceNote()
            ).until(() -> !indexer.getBeamBreak())
            .andThen(
                intake.stopAll()
            )
            .withTimeout(2.5)
        );

        //NamedCommands.registerCommand("Auto Intake", new PrintCommand("Intake Note"));

        // An example Named Command, doesn't need to remain once we start actually adding real things
        // ALL Named Commands need to be defined AFTER subsystem initialization and BEFORE auto/controller configuration
        NamedCommands.registerCommand("Set Drivetrain X", drivetrain.setXCommand());


        // TODO Specify a default auto string once we have one
        autoChooser = AutoBuilder.buildAutoChooser();

        driver = new CommandXboxController(OIConstants.kPrimaryXboxUSB);
        operator = new CommandXboxController(OIConstants.kSecondaryXboxUSB);

        isTeleopTrigger = new Trigger(DriverStation::isTeleopEnabled);
        isEnabledTrigger = new Trigger(DriverStation::isEnabled);
        indexerBeamBreakWatcher = new Trigger(indexer::getBeamBreak);

        setAmpAngle = false;
        intakeDown = false;

        configureBindings();
        shuffleboardSetup();
    }

    // The objective should be to have the subsystems expose methods that return commands
    // that can be bound to the triggers provided by the CommandXboxController class.
    // This mindset should help keep RobotContainer a little cleaner this year. 
    // This is not to say you can't trigger or command chain here (see driveCardnial drivetrain example),
    // but generally reduce/avoid any situation where the keyword "new" is involved if you're working 
    // with a subsystem
    private void configureBindings() {
        isTeleopTrigger.onTrue(new InstantCommand(() -> Shuffleboard.selectTab(kTeleopTabName)));

        isEnabledTrigger.onTrue(
            new InstantCommand(
                () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

                    if (alliance.isEmpty() || alliance.get() == DriverStation.Alliance.Red) {
                        ampTagID = 5;
                        sourceTagID = 9;
                        speakerTag = 4;
                        invertXYDrive = true;
                    } else {
                        ampTagID = 6;
                        sourceTagID = 1;
                        speakerTag = 8;
                        invertXYDrive = false;
                    }
                }
            )
        );

        drivetrain.setDefaultCommand(
            drivetrain.teleopCommand(
                () -> { return driver.getLeftY();}, 
                () -> { return driver.getLeftX();}, 
                () -> { return driver.getRightX();}, 
                OIConstants.kTeleopDriveDeadband
            )
        );

        intake.setDefaultCommand(
            intake.intakeControl(
                () -> {
                    if (intakeDown && indexer.getBeamBreak()) {
                        return IntakeConstants.kDownAngle;
                    } else {
                        return IntakeConstants.kUpAngle;
                    }
                }, 
                () -> {
                    if(intakeDown && indexer.getBeamBreak()){
                        return 1.0;
                    }else if(operator.leftBumper().getAsBoolean()){
                        return -1.0;
                    }else{
                        return 0.0;
                    }
                }
            )
        );

        driver.leftTrigger().whileFalse(
            new InstantCommand(
                () -> {
                    intakeDown = false;
                }
            )
        );

        driver.leftTrigger().whileTrue(Commands.parallel(
            new InstantCommand(
                () -> {
                    intakeDown = true;
                }
            ), indexer.advanceNote()
        ));

        shooter.setDefaultCommand(
            shooter.angleSpeedsSetpoints(
                () -> {
                    if (setAmpAngle) {
                        return ShooterConstants.kShooterAmpAngle;
                    } else {
                        return ShooterConstants.kShooterLoadAngle;
                    }
                },
                () -> {
                    if(driver.getRightTriggerAxis() > 0.25){
                        return 1.0;
                    } else if(operator.getRightTriggerAxis() > 0.25){
                        return -1.0;
                    }else{
                        return 0.0;
                    }
                }
            )
        );

        driver.leftBumper().onTrue(
            shooter.angleSpeedsSetpoints(
                () -> ShooterConstants.kShooterLoadAngle, 
                () -> 1.0
            ).withTimeout(.5).andThen(
                shooter.angleSpeedsSetpoints(
                    () -> ShooterConstants.kShooterLoadAngle, 
                    () -> 1.0
                ).alongWith(
                    indexer.shootNote(() -> 1.0)
                ).withTimeout(2)
            )
        );

        climber.setDefaultCommand(climber.stop());

        indexer.setDefaultCommand(indexer.shootNote(
            () -> {
                if (driver.getRightTriggerAxis() > .25) {
                        return 1.0;
                    }else {
                        return 0.0;
                    }
            }
        ));

        indexerBeamBreakWatcher.onFalse(
            new RunCommand(() -> {
                driver.getHID().setRumble(RumbleType.kBothRumble, 1);
            }).withTimeout(.5).andThen(
                new InstantCommand(() -> {
                    driver.getHID().setRumble(RumbleType.kBothRumble, 0);
                })
            )
        );

        /*
        driver.povCenter().onFalse(
            drivetrain.driveCardinal(
                driver::getLeftY, 
                driver::getLeftX, 
                driver.getHID()::getPOV,
                OIConstants.kTeleopDriveDeadband).until(
                () -> MathUtil.applyDeadband(driver.getRightX(), OIConstants.kTeleopDriveDeadband) != 0
                )
        );

        // Lock on to the appropriate Amp AprilTag while still being able to strafe and drive forward and back
        driver.leftBumper().onTrue(
            drivetrain.driveAprilTagLock(
                driver::getLeftY, 
                driver::getLeftX, 
                OIConstants.kTeleopDriveDeadband, 
                ampTagID
            ).until(
                () -> MathUtil.applyDeadband(driver.getRightX(), OIConstants.kTeleopDriveDeadband) != 0
            )
        );

        // Lock on to the appropriate Source AprilTag while still being able to strafe and drive forward and back
        driver.b().onTrue(
            drivetrain.driveAprilTagLock(
                driver::getLeftY, 
                driver::getLeftX,  
                OIConstants.kTeleopDriveDeadband, 
                sourceTagID
            ).until(
                () -> MathUtil.applyDeadband(driver.getRightX(), OIConstants.kTeleopDriveDeadband) != 0
            )
        );

        // Lock on to the appropriate Speaker AprilTag while still being able to strafe and drive forward and back
        driver.x().onTrue(
            drivetrain.driveAprilTagLock(
                driver::getLeftY, 
                driver::getLeftX, 
                OIConstants.kTeleopDriveDeadband, 
                speakerTag
            ).until(
                () -> MathUtil.applyDeadband(driver.getRightX(), OIConstants.kTeleopDriveDeadband) != 0
            )
        ); */

        // This was originally a run while held, not sure that's really necessary, change it if need be
        driver.y().onTrue(drivetrain.zeroHeadingCommand());

        // This was originally a run while held, not sure that's really necessary, change it if need be
        driver.rightBumper().onTrue(drivetrain.setXCommand());

        operator.rightTrigger().whileTrue(indexer.shootNote(() -> -1.0));

        /*
        * This has been added because interest has been expressed in trying field relative vs
        * robot relative control. This should <i>default</i> to field relative, but give the option
        * for the person practicing driving to push start on the driver's controller to quickly switch to
        * the other style. 
        * 
        * If it becomes something that we need to switch <i>prior</i> to the start of the match, a different
        * mechanism will need to be devised, this will work for now. 
        */
        driver.start().onTrue(drivetrain.toggleFieldRelativeControl());

        operator.b().whileTrue(Commands.parallel(indexer.shootNote(() -> 1.0), shooter.ampHandoff(() -> driver.getRightTriggerAxis(), () -> {
            if(operator.getRightTriggerAxis()>0.25){
                return true;
            }else{
                return false;
            }
        })));
        

        //driver.leftBumper().toggleOnTrue(intake.intakeDownCommand());

        operator.a().onTrue(new InstantCommand(() -> { setAmpAngle = true; }));
        operator.a().onFalse(new InstantCommand(() -> { setAmpAngle = false; }));
        
        //operator.a().whileTrue(shooter.angleSpeedsSetpoints(ShooterConstants.kShooterAmpAngle, 0, 0));

        operator.y().toggleOnTrue(Commands.parallel(climber.setSpeedWithSupplier(operator::getRightTriggerAxis), shooter.climbState(), intake.climbingStateCommand()));

        driver.a().whileTrue(indexer.shootNote(() -> 1.0));

        operator.back().toggleOnTrue(shooter.manualPivot(
        () -> MathUtil.clamp(-operator.getLeftY(), -0.25, 0.25),
        () -> MathUtil.clamp(driver.getRightTriggerAxis(), -1, 1)
        ));

        driver.povDown().whileTrue(indexer.shootNote(() -> -1.0));

        operator.leftTrigger().whileTrue(Commands.parallel( indexer.shootNote(() -> -1), intake.intakeControl(() -> IntakeConstants.kDownAngle, () -> -1.0)));

        operator.start().toggleOnTrue(intake.manualPivot(() -> -operator.getRightY()*0.2, () -> driver.getLeftTriggerAxis()));
        
  }

  private void shuffleboardSetup() {
    ShuffleboardTab autoTab = Shuffleboard.getTab(kAutoTabName);
    autoTab.add("Autonomous Selector", autoChooser)
      .withPosition(0, 0)
      .withSize(2, 1)
      .withWidget(BuiltInWidgets.kComboBoxChooser);
    
    

    //Always select the auto tab on startup
    Shuffleboard.selectTab(kAutoTabName);

    ShuffleboardTab teleopTab = Shuffleboard.getTab(kTeleopTabName);
    teleopTab.addBoolean("Field Relative Controls Enabled?", drivetrain::isFieldRelativeControl)
        .withPosition(0, 0)
        .withSize(2, 1)
        .withWidget(BuiltInWidgets.kBooleanBox);
    teleopTab.addBoolean("indexer beam break", indexer::getBeamBreak);
    teleopTab.addBoolean("shooter beam break", shooter::getBeamBreak);
    teleopTab.addDouble("shooter angle", shooter::getShooterAngle);
    teleopTab.addDouble("intake angle", intake::getIntakeDegrees);
    //teleopTab.addDouble("intake pid", intake::getIntakePID);
    teleopTab.addDouble("heading swerve", drivetrain::getGyroAngle);
    teleopTab.addDouble("steering encoder", drivetrain::getEncoderSteering);
  }

  public Command getAutonomousCommand() {
        return autoChooser.getSelected();
  }
}
