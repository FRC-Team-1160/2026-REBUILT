// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.OperatorConstants;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.Subsystems.Agitator.Agitator;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainRealIO;
import frc.robot.Subsystems.DriveTrain.DriveTrainSimIO;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.direction;
import frc.robot.Subsystems.Intake.Intake.intakeDirection;
import frc.robot.Subsystems.Intake.Intake.intakeMode;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.Shooter.SHOOTER_MODES;
import frc.robot.Constants.ShooterConstants;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private boolean lockSwerve = false;
  
  //private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Integer> autoChooser2 = new SendableChooser<>();
  public record JoystickInputs(double drive_x, double drive_y, double drive_a) {}
  //check is need joystick inputs or not
  private Joystick main_stick = new Joystick(Constants.IO.MAIN_PORT);
  private Joystick second_stick = new Joystick(Constants.IO.COPILOT_PORT);
  private Joystick test_stick = new Joystick(3);
  //

  public final DriveTrain m_drive = Robot.isReal() ? new DriveTrainRealIO() : new DriveTrainSimIO();
  public final Intake m_intake = new Intake();
  public final Shooter m_shooter = new Shooter();
  public final Agitator m_agitator = new Agitator();

  private boolean runningSequence1 = false;
  private boolean runningSequence2 = false;
  private boolean autoAlignHub = false;

  private final String redRightBumpAutoName = "RED Bump Intake Right";
  private final String blueRightBumpAutoName = "Bump Intake Right";
  private final String redLeftBumpAutoName = "RED Bump Intake Left";
  private final String blueLeftBumpAutoName = "Bump Intake Left";

  PathPlannerAuto redRightBumpAuto = new PathPlannerAuto(redRightBumpAutoName);
  PathPlannerAuto blueRightBumpAuto = new PathPlannerAuto(blueRightBumpAutoName);
  PathPlannerAuto redLeftBumpAuto = new PathPlannerAuto(redLeftBumpAutoName);
  PathPlannerAuto blueLeftBumpAuto = new PathPlannerAuto(blueLeftBumpAutoName);
  
  PathPlannerAuto[] autos = {
    redRightBumpAuto,
    blueRightBumpAuto,
    redLeftBumpAuto,
    blueLeftBumpAuto
  };

  //The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drive.resetGyroAngle();

    //autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser2.setDefaultOption("Red Right Bump", 0);
    autoChooser2.addOption("Blue Right Bump", 1);
    autoChooser2.addOption("Red Left Bump", 2);
    autoChooser2.addOption("Blue Left Bump", 3);
    autoChooser2.addOption("Center Shoot", 4);

    //autoChooser2.addOption("Start Hub", "RED Bump Intake Right");

    //SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Auto Selection", autoChooser2);
    // SmartDashboard.putData("Auto Side", leftRightAuto);

    InstantCommand disableVisionMeasurement = new InstantCommand(() -> {
      m_drive.autoVisionMeasurement = false;
    });

    InstantCommand enableVisionMeasurement = new InstantCommand(() -> {
      m_drive.autoVisionMeasurement = true;
    });

    NamedCommands.registerCommand("Enable Vision", enableVisionMeasurement);
    NamedCommands.registerCommand("Disable Vision", disableVisionMeasurement);

    SequentialCommandGroup intakeInOut = new SequentialCommandGroup(
      new InstantCommand(() -> {
        m_intake.extendArm();
        m_intake.setModes(direction.EXTENDING, intakeMode.AUTOMATIC);
      }),
      new WaitCommand(0.25),
      new InstantCommand(() -> {
        m_intake.retractArm();
        m_intake.setModes(direction.RETRACTING, intakeMode.AUTOMATIC);
      }),
      new WaitCommand(0.25)
    );

    NamedCommands.registerCommand("Intake In Out", intakeInOut);

    NamedCommands.registerCommand("Align Hub", new InstantCommand(() -> {autoAlignHub = true;}));
    NamedCommands.registerCommand("Cancel Align", new InstantCommand(() -> {autoAlignHub = false;}));

    NamedCommands.registerCommand("Hub Shooter", new InstantCommand(() -> {
      m_shooter.setMode(SHOOTER_MODES.AUTO_DISTANCE);
    }));

    NamedCommands.registerCommand("Run Shooter",new InstantCommand(() -> {
      m_shooter.setMode(SHOOTER_MODES.AUTO_DISTANCE);
    }
    ));
    NamedCommands.registerCommand("Stop Shooter",new InstantCommand(() -> {
      m_shooter.enabled = false;
      }
    ));

    NamedCommands.registerCommand("Run Agitator",m_agitator.runMotors);
    NamedCommands.registerCommand("Stop Agitator",m_agitator.stopMotors);

    //setting up pathplanner commands

    new EventTrigger("Extend Intake").onTrue(new InstantCommand(() -> {
      m_intake.setModes(direction.EXTENDING, intakeMode.AUTOMATIC);
      m_intake.extendArm();
    }
    ));
    new EventTrigger("Retract Intake").onTrue(new InstantCommand(() -> {
      m_intake.setModes(direction.RETRACTING, intakeMode.AUTOMATIC);
      m_intake.retractArm();
    }
    ));

    // Configure the trigger bindings
    configureBindings();
  }

  public void autoAlignSwerve() {
    if (DriverStation.isAutonomous() && autoAlignHub) {
        double angle_radiansPerSecond;
        angle_radiansPerSecond = m_drive.getTurnToHub(); //* (m_limelightio.blueAlliance == true ? 1 : -1);

        m_drive.setSwerveDrive(
        0,0,
        angle_radiansPerSecond
        );

        SmartDashboard.putNumber("auto angle", angle_radiansPerSecond);
      }
  }

  public void updateSwerve() {
    if (!DriverStation.isAutonomous() && !lockSwerve){
    double driveMult = 1.25; //change this constant to change the drive speed.
    double rotationMult = 1.5; //change this constant to change the turn speed.
    
    double mult = m_shooter.enabled ? 0.2 : driveMult;
    //double degreeDifference = getHubDegreeDiff();
    //facingHub = (degreeDifference == 0);
    SmartDashboard.putBoolean("tv",LimelightHelpers.getTV(ShooterConstants.LIMELIGHT_NAME));

    double x_metersPerSecond = (Math.abs(main_stick.getRawAxis(1)) < 0.1) ? 0 : 2.7 * -main_stick.getRawAxis(1);
    SmartDashboard.putNumber("x_mps", x_metersPerSecond);

    double y_metersPerSecond = (Math.abs(main_stick.getRawAxis(0)) < 0.1) ? 0 : 2.7 * -main_stick.getRawAxis(0);

    double angle_radiansPerSecond;

    // if pressing button 6 then we align to the hub
    if ((main_stick.getRawAxis(2) >= 0.2)) {
      angle_radiansPerSecond = m_drive.getTurnToHub(); //* (m_limelightio.blueAlliance == true ? 1 : -1);
      SmartDashboard.putBoolean("align attemp", true);
    } else {  
      angle_radiansPerSecond = (Math.abs(main_stick.getRawAxis(4)) < 0.2) ? 0 : -3 * Math.signum(main_stick.getRawAxis(4))
      * Math.pow(main_stick.getRawAxis(4), 2) * rotationMult;
      SmartDashboard.putBoolean("align attemp", false);
    }
    //negative turn values go right, positive go left
    
    //SmartDashboard.putNumber("axis_0", leftStickLeftRight);
    //SmartDashboard.putNumber("angle", angle_radiansPerSecond);

    int forwards = (m_drive.blueAlliance ? 1 : -1);
    m_drive.setSwerveDrive(
      x_metersPerSecond * mult * forwards, 
      y_metersPerSecond * mult * forwards, 
      angle_radiansPerSecond
      );
    }
  }

  public void updateShooterDistance() {
    m_shooter.distanceFromTargetInches = m_drive.getDistanceFromHub();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}
   */

  private void configureBindings() {
    //MAIN STICK -------------------------
    new JoystickButton(main_stick, 8).onTrue(
      new InstantCommand(() -> {
        m_drive.refreshAlliance();
        m_drive.resetGyroAngle();
      })
    );

    //stop intake
    new JoystickButton(main_stick, 3).onTrue(
      new InstantCommand(() -> {
        m_intake.setIntakeDirection(intakeDirection.OFF);
        m_intake.setModes(direction.IGNORE, intakeMode.MANUAL);
      })
    );

    //extend hopper
    new JoystickButton(main_stick, 5).onTrue(
      new InstantCommand(() -> {
        m_intake.extendArm();
        m_intake.setModes(direction.EXTENDING, intakeMode.AUTOMATIC);
        m_intake.setIntakeDirection(intakeDirection.IN);
      })
    );

    //retract hopper
    new JoystickButton(main_stick, 4).onTrue(
      new InstantCommand(() -> {
        m_intake.retractArm();
        m_intake.setModes(direction.RETRACTING, intakeMode.AUTOMATIC);
        m_intake.setIntakeDirection(intakeDirection.OUT);
      })
    );

    //lock wheels
    new Trigger(() -> main_stick.getRawButton(6)).whileTrue(
      new RunCommand(() -> {
        m_drive.setModuleMode(true);
        lockSwerve = true;
      }).finallyDo(
        () -> {
          m_drive.setModuleMode(false);
          lockSwerve = false;
        })
      );

    //SECOND STICK -------------------------
    //sequences fml

    //creating our sequence commands
    InstantCommand extendHopperCommand = new InstantCommand(() -> {
              if (runningSequence1) {
                  m_intake.extendArm();
                  m_intake.setModes(direction.EXTENDING, intakeMode.AUTOMATIC);
                }
            });
    InstantCommand retractHopperCommand = new InstantCommand(() -> {
              if (runningSequence1) {
                  m_intake.retractArm();
                  m_intake.setModes(direction.RETRACTING, intakeMode.AUTOMATIC);
                }
            });

    double waitInterval = 0.25;
    SequentialCommandGroup waitIntakeOuttakeCommand = new SequentialCommandGroup(extendHopperCommand, new WaitCommand(waitInterval), retractHopperCommand, new WaitCommand(waitInterval));
    RepeatCommand repeatIntakeOuttakeCommand = new RepeatCommand(waitIntakeOuttakeCommand);

    //after waiting for .4 seconds, basically do the second half of sequence 1
    var spamIntakeSequence = new WaitCommand(0.3).finallyDo(() -> {
          if (runningSequence1) {
            m_agitator.runMotors.schedule();
            //turn on gate and agitator
            repeatIntakeOuttakeCommand.schedule();
          }
        });
    
    //same explanation as above basically
    var slowRetractSequence = new WaitCommand(0.4).finallyDo(() -> {
          if (runningSequence2) {
            m_agitator.runMotors.schedule();
            //turn on gate and agitator
            m_intake.setHopperSpeed(0.3);
            m_intake.setModes(direction.RETRACTING, intakeMode.AUTOMATIC);
            m_intake.retractArm();
          }
        });
      //--------------------------------------

    //sequence 1
    //run shooter and such + intake goes in and out
    new Trigger(() -> (second_stick.getRawButton(6) || second_stick.getRawAxis(3) >= 0.2)).whileTrue(
      new StartEndCommand(() -> {
        //only run if were not running the other sequence and were not already running this sequence
        if (!runningSequence2 && !runningSequence1) {
        runningSequence1 = true;
        //start shooter first
        boolean autoDist = (second_stick.getRawButton(6));
        m_shooter.setMode(autoDist ? SHOOTER_MODES.AUTO_DISTANCE : SHOOTER_MODES.STATIC_DISTANCE);
          // 90 is where we usually are, so a good number to rev up to
        //give shooter time to rev up
        m_intake.setModes(direction.IGNORE, intakeMode.MANUAL);
        m_intake.setIntakeDirection(intakeDirection.OFF);
        //turn intake off
        //have intake go in and out with shooter
        spamIntakeSequence.schedule();
      } else {
        //if were already running a sequence then just return
        return;
      }
    },
      () -> {
        if (runningSequence1) {
          //turn everything back off
          runningSequence1 = false;
          m_agitator.stopMotors.schedule();
          m_shooter.enabled = false;
          m_intake.setModes(direction.IGNORE, intakeMode.AUTOMATIC);
          repeatIntakeOuttakeCommand.cancel();
        }
      }));

      new Trigger(() -> second_stick.getRawButton(6) || second_stick.getRawAxis(3) >= 0.2).whileTrue(
        new InstantCommand(() -> {
          if (runningSequence1 || runningSequence2) {
            if (second_stick.getRawAxis(3) >= 0.2) {
            m_shooter.setMode(SHOOTER_MODES.STATIC_DISTANCE);
          } else {m_shooter.setMode(SHOOTER_MODES.AUTO_DISTANCE);}
          }
        })
      );

        //stop everything once we stop holding down the butto
    //sequence 2
    new Trigger(() -> second_stick.getRawAxis(2) >= 0.2).whileTrue(
      new StartEndCommand(() -> {
        //only run if were not running the other sequence and were not already running this sequence
        if (!runningSequence2 && !runningSequence1) {
        //start shooter first
        runningSequence2 = true;
        m_shooter.setMode(SHOOTER_MODES.AUTO_DISTANCE);
        //give shooter time to rev up
        //slowly retract intake while shooting and such
        slowRetractSequence.schedule();
      } else {return;} //return if were already running a sequence
      },
      () -> {
        //reset back to normal
        runningSequence2 = false;
        m_intake.setHopperSpeed(1);
        m_agitator.stopMotors.schedule();
        m_shooter.enabled = false;
        slowRetractSequence.cancel();
      }
    ));

    //extend/retract hopper
    //new control
    new Trigger(() -> second_stick.getPOV() == 0).onTrue(
      new InstantCommand(() -> {
        m_intake.extendArm();
        m_intake.setModes(direction.EXTENDING, intakeMode.MANUAL);
        m_intake.setIntakeDirection(intakeDirection.OFF);
    }));

    new Trigger(() -> second_stick.getPOV() == 180).onTrue(
      new InstantCommand(() -> {
        m_intake.retractArm();
        m_intake.setModes(direction.RETRACTING, intakeMode.AUTOMATIC);
    }));

    new Trigger(() -> second_stick.getRawButton(3)).whileTrue(
      new RunCommand(() -> {
        boolean forward = !second_stick.getRawButton(5);
        m_intake.setModes(direction.IGNORE, intakeMode.MANUAL);
        if (forward) {
          m_intake.setIntakeDirection(intakeDirection.IN);
        } else {m_intake.setIntakeDirection(intakeDirection.OUT);}
      }).finallyDo(() -> {
        m_intake.setIntakeDirection(intakeDirection.OFF);
      })
    );

    new Trigger(() -> (second_stick.getRawButton(2))).whileTrue(
      new RunCommand(() -> {
        int mult = second_stick.getRawButton(5) ? -1 : 1;
        m_agitator.runMotors.schedule();
      }
      ).finallyDo(() -> {
        m_agitator.stopMotors.schedule();
      })
    );
   
    // shooter bindings
    new Trigger(() -> (second_stick.getRawButton(4)) || second_stick.getRawButton(1)).whileTrue(
        new RunCommand(() -> {
          boolean reversed = second_stick.getRawButton(5);
          m_shooter.setMode(reversed ? SHOOTER_MODES.REVERSED : SHOOTER_MODES.AUTO_DISTANCE);
        })
          .finallyDo(() -> {
            m_shooter.enabled = false;
          })
      );}

  public Command getAutonomousCommand() {
    //m_drive.refreshAlliance();
    m_drive.resetGyroAngle();
    int autoNum = autoChooser2.getSelected();

    if (autoNum == 4) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        m_shooter.enabled = true;
        m_shooter.setMode(SHOOTER_MODES.AUTO_DISTANCE);
      }),
      new WaitCommand(0.5),
      m_agitator.runMotors
    );
    } else {
      PathPlannerAuto m_auto = autos[autoNum];
      return m_auto;
    }
  }
}
