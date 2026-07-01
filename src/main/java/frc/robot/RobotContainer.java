// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.OperatorConstants;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.Subsystems.Agitator.Agitator;
import frc.robot.Subsystems.Shooter.Shooter;
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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FieldConstants.HubMeasurements;

import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public record JoystickInputs(double drive_x, double drive_y, double drive_a) {}
  //check is need joystick inputs or not
  private Joystick main_stick = new Joystick(Constants.IO.MAIN_PORT);
  //
  public final DriveTrain m_drive = Robot.isReal() ? new DriveTrainRealIO() : new DriveTrainSimIO();
  public final Agitator m_agitator = new Agitator();
  public final Shooter m_Shooter = new Shooter();

  //The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_drive.resetGyroAngle();
    // Configure the trigger bindings
    configureBindings();
  }

  double rotationMult = 0;
  double driveMult = 0;

  public double allowMovement(boolean directionIsX, double motion) {
    double positionOnAxis = directionIsX ? m_drive.pose_estimator.getEstimatedPosition().getX() : m_drive.pose_estimator.getEstimatedPosition().getY();
    double[] positionBoundaries = directionIsX ? HubMeasurements.positionXBoundaries : HubMeasurements.positionYBoundaries;
    
    double distanceMoving = motion * 0.1; // tune: find out how much distance is actually covered in a second
    double estimatePositionUpdate = positionOnAxis + distanceMoving;
    if (estimatePositionUpdate > positionBoundaries[0] && estimatePositionUpdate < positionBoundaries[1]) {
      return motion;
    } else {
      return 0;
    }
  }

  public void updateSwerve() {
    double x_metersPerSecond = (Math.abs(main_stick.getRawAxis(1)) < 0.1) ? 0 : 2.7 * -main_stick.getRawAxis(1);
    double y_metersPerSecond = (Math.abs(main_stick.getRawAxis(0)) < 0.1) ? 0 : 2.7 * -main_stick.getRawAxis(0);

    double angle_radiansPerSecond = (Math.abs(main_stick.getRawAxis(4)) < 0.2) ? 0 : -3 * Math.signum(main_stick.getRawAxis(4))
      * Math.pow(main_stick.getRawAxis(4), 2) * rotationMult;

    int forwards = (m_drive.blueAlliance ? 1 : -1);

    double finalXMeters = x_metersPerSecond * driveMult * forwards;
    double finalYMeters = y_metersPerSecond * driveMult * forwards;

    // finalXMeters = allowMovement(true, finalXMeters);
    // finalYMeters = allowMovement(false, finalYMeters);

    m_drive.setSwerveDrive(
      finalXMeters, 
      finalYMeters, 
      angle_radiansPerSecond
    );
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

    new JoystickButton(main_stick, 1).onTrue(m_agitator.startMotors);
    new JoystickButton(main_stick, 2).onTrue(m_agitator.stopMotors);
    new JoystickButton(main_stick, 3).onTrue(m_Shooter.startMotorsyah);
    new JoystickButton(main_stick, 4).onTrue(m_Shooter.stopMotorsyah);
    //connect commands to controller bindings

    //new JoystickButton(main_stick, 0).onTrue(new SequentialCommandGroup(m_agitator.startMotors,m_agitator.stopMotors));
  }

  public Command getAutonomousCommand() {
    //m_drive.refreshAlliance();
    //m_drive.resetGyroAngle(); --
    return new SequentialCommandGroup(
    );
}
}
