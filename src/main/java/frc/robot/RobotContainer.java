// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import org.opencv.core.Mat;

//import frc.robot.Constants.OperatorConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.Subsystems.Agitator.Agitator;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainRealIO;
import frc.robot.Subsystems.DriveTrain.DriveTrainSimIO;
// import frc.robot.SubsystemManager;
import frc.robot.commands.Autos;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
//import frc.robot.commands.ExampleCommand;
//import frc.robot.commands.IntakeCommand.ToggleIntake;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.direction;
import frc.robot.Subsystems.Intake.Intake.intakeDirection;
import frc.robot.Subsystems.Intake.Intake.intakeMode;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Vision.LimelightIO;
import frc.robot.Subsystems.Vision.VisionSubsystem;
//import frc.robot.Subsystems.Vision.Vision;
//import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ControllerButtonConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.PointTowardsZoneTrigger;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private boolean testingShooter = false;
  private boolean facingHub = false;
  private boolean shooting = false;
  
  private final SendableChooser<Command> autoChooser;
  public record JoystickInputs(double drive_x, double drive_y, double drive_a) {}
  //check is need joystick inputs or not
  private Joystick main_stick = new Joystick(Constants.IO.MAIN_PORT);
  private Joystick second_stick = new Joystick(Constants.IO.COPILOT_PORT);
  private Joystick test_stick = new Joystick(3);
  //

  public final DriveTrain m_drive = Robot.isReal() ? new DriveTrainRealIO() : new DriveTrainSimIO();
  public final Intake m_intake = new Intake();
  //public final AgitatorTest m_agitatorTest = new AgitatorTest();
  public final Shooter m_shooter = new Shooter();
  public final LimelightIO m_limelightio = new LimelightIO();
  public final VisionSubsystem m_vision = new VisionSubsystem(m_limelightio);
  public final Agitator m_agitator = new Agitator();
  
  private enum ROBOT_STATES {

  }

  //The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("Align Hub", new RunCommand(() -> {
      if (DriverStation.isAutonomous() && LimelightHelpers.getTV(ShooterConstants.LIMELIGHT_NAME)) {
        double angle_radiansPerSecond;

        double degreeDifference = m_vision.getAngleDiffBotToHub(m_drive.odom_pose.getRotation().getDegrees());
        degreeDifference = Math.abs(degreeDifference) < (2) ? 0 : degreeDifference;
        facingHub = (degreeDifference == 0);
        angle_radiansPerSecond = -Math.max(Math.min(Math.pow(degreeDifference * (Math.PI/180) * 4,2), 3),-3)
        * (degreeDifference < 0 ? -1 : 1);

        m_drive.setSwerveDrive(
        0,0,
        angle_radiansPerSecond
        );

        SmartDashboard.putNumber("auto angle", angle_radiansPerSecond);
      }
    }).until(() -> facingHub));

    NamedCommands.registerCommand("Run Shooter",new InstantCommand(() -> 
      m_shooter.runMotors(facingHub ? m_vision.getBotToHubDistance() : 70)
    ));
    NamedCommands.registerCommand("Stop Shooter",new InstantCommand(() -> 
      m_shooter.stopMotors()
    ));

    NamedCommands.registerCommand("Run Agitator",new InstantCommand(() -> {
      m_agitator.runAgitation(1);
      m_agitator.runGate(1);}));
    NamedCommands.registerCommand("Stop Agitator",new InstantCommand(() -> {
      m_agitator.stopAgitation();
      m_agitator.stopGate();}));

    //setting up pathplanner commands

    new EventTrigger("Extend Intake").onTrue(new InstantCommand(() -> 
      m_intake.extendArm()
    ));
    new EventTrigger("Retract Intake").onTrue(new InstantCommand(() -> 
      m_intake.retractArm()
    ));

    // Configure the trigger bindings
    configureBindings();
  }

  public void updateSwerve() {
    if (!DriverStation.isAutonomous()){
    double mult = shooting ? 0.2 : 1;
    SmartDashboard.putNumber("bot-hub degreeDifference", m_vision.getAngleDiffBotToHub(m_drive.getGyroAngle().getDegrees()));
    SmartDashboard.putBoolean("tv",LimelightHelpers.getTV(ShooterConstants.LIMELIGHT_NAME));

    double x_metersPerSecond = (Math.abs(main_stick.getRawAxis(1)) < 0.1) ? 0 : 2.1 * -main_stick.getRawAxis(1);
    SmartDashboard.putNumber("x_mps", x_metersPerSecond);

    double y_metersPerSecond = (Math.abs(main_stick.getRawAxis(0)) < 0.1) ? 0 : 2.1 * -main_stick.getRawAxis(0);

    double angle_radiansPerSecond;    

    // if pressing button 6 then we align to the hub
    if ((main_stick.getRawAxis(2) >= 0.2) && LimelightHelpers.getTV(ShooterConstants.LIMELIGHT_NAME)) {
      double degreeDifference = m_vision.getAngleDiffBotToHub(m_drive.odom_pose.getRotation().getDegrees());
      degreeDifference = Math.abs(degreeDifference) < (2) ? 0 : degreeDifference;
      facingHub = (degreeDifference == 0);
      angle_radiansPerSecond = -Math.max(Math.min(Math.pow(degreeDifference * (Math.PI/180) * 4,2), 3),-3)
      * (degreeDifference < 0 ? -1 : 1);
      
      SmartDashboard.putNumber("degree diff", degreeDifference);

      //angle_radiansPerSecond = degreeDifference < 0 ? 0.2 : -0.2;
    } else {  
      angle_radiansPerSecond = (Math.abs(main_stick.getRawAxis(4)) < 0.2) ? 0 : -3 * Math.signum(main_stick.getRawAxis(4))
      * Math.pow(main_stick.getRawAxis(4), 2);
    }
    
    //SmartDashboard.putNumber("axis_0", leftStickLeftRight);
    //SmartDashboard.putNumber("angle", angle_radiansPerSecond);

    m_drive.setSwerveDrive(
      x_metersPerSecond * mult, 
      y_metersPerSecond * mult, 
      angle_radiansPerSecond
      );}
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
      new InstantCommand(m_drive::resetGyroAngle)
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

    //SECOND STICK -------------------------

    //sequences
    new Trigger(() -> second_stick.getRawAxis(2) >= 0.2).whileTrue(
      new RunCommand(() -> {
        
      })
    );

    //extend/retract hopper
    new JoystickButton(second_stick, 1).onTrue(
      new InstantCommand(() -> {
        if (!m_intake.isFullyExtended()) {
        m_intake.extendArm();
        m_intake.setModes(direction.EXTENDING, intakeMode.AUTOMATIC);
        m_intake.setIntakeDirection(intakeDirection.IN);
        } else {
        m_intake.retractArm();
        m_intake.setModes(direction.RETRACTING, intakeMode.AUTOMATIC);
        }
      })
    );

    //retract hopper
    new JoystickButton(second_stick, 1).onTrue(
      new InstantCommand(() -> {
        m_intake.retractArm();
        m_intake.setModes(direction.RETRACTING, intakeMode.AUTOMATIC);
        m_intake.setIntakeDirection(intakeDirection.OUT);
      })
    );

    //running intake
    new Trigger(() -> second_stick.getRawButton(3) && 
    (second_stick.getRawAxis(2) >= 0.2 || second_stick.getRawButton(5))).whileTrue(
      new RunCommand(() -> {
        boolean forward = second_stick.getRawAxis(2) >= 0.2;
        m_intake.setModes(direction.IGNORE, intakeMode.MANUAL);
        if (forward) {
          m_intake.setIntakeDirection(intakeDirection.IN);
        } else {m_intake.setIntakeDirection(intakeDirection.OUT);}
      }).finallyDo(() -> {
        m_intake.setIntakeDirection(intakeDirection.OFF);
      })
    );

    // agitator + gate forward/reverse
    new Trigger(() -> second_stick.getRawButton(2) && 
    (second_stick.getRawAxis(2) > 0.2) || second_stick.getRawButton(5)).whileTrue(
      new RunCommand(() -> {
        int mult = second_stick.getRawButton(5) ? -1 : 1;
        m_agitator.runAgitation(mult);
        m_agitator.runGate(mult);
      }
      ).finallyDo(() -> {
        m_agitator.stopGate();
        m_agitator.stopAgitation();
      })
    );
   
    // shooter bindings
    new Trigger(() -> (second_stick.getRawButton(4) && 
    ((second_stick.getRawAxis(2) >= 0.2) || second_stick.getRawButton(5)))).whileTrue(
        new RunCommand(() -> {
          boolean forwards = second_stick.getRawAxis(2) >= 0.2;
          if (forwards) {
            m_shooter.runMotors(facingHub ? m_vision.getBotToHubDistance() : 70);
            shooting = true;
          } else {
            m_shooter.reverseMotors();
          }
        })//150
          .finallyDo(() -> {
            m_shooter.stopMotors();
            shooting = false;
          })
          //if we are facing the hub then shoot correctly
          // otherwise were probably trying to shoot from the neutral zone
          // so just launch it at a constant distance basically
      );

      // new Trigger(() -> second_stick.getRawButton(3)).whileTrue(
      //   new RunCommand(() -> m_shooter.basketballin())
      //     .finallyDo(m_shooter::stopMotors)
      // );

    // shooter test bindings
    
    if (testingShooter) {
      new JoystickButton(test_stick, 5).onTrue(
        new InstantCommand(() -> m_shooter.changeTopRollerRPS(-0.1 * (test_stick.getRawButton(4) ? 10 : 1)))
      );

      new JoystickButton(test_stick, 6).onTrue(
        new InstantCommand(() -> m_shooter.changeTopRollerRPS(0.1 * (test_stick.getRawButton(4) ? 10 : 1)))
      );

      new Trigger(() -> test_stick.getRawButton(3)).whileTrue(
        new RunCommand(() -> m_shooter.testRunMotors())
          .finallyDo(m_shooter::stopMotors)
      );
    }

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
      //  .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  } 
}
