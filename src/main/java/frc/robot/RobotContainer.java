// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainRealIO;
import frc.robot.Subsystems.DriveTrain.DriveTrainSimIO;
// import frc.robot.SubsystemManager;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand.ToggleIntake;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Constants.IntakeConstants;

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
  private Joystick second_stick = new Joystick(Constants.IO.COPILOT_PORT);

  //

  public final DriveTrain m_drive = Robot.isReal() ? new DriveTrainRealIO() : new DriveTrainSimIO();
  public final Intake m_intake = new Intake();
  
  
  //The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

    public void updateSwerve() {
    //double rightStickUpDown = main_stick.getRawAxis(5);
    //SmartDashboard.putNumber("joystick_axis_5", rightStickUpDown);

    double x_metersPerSecond = (Math.abs(main_stick.getRawAxis(0)) < 0.1) ? 0 : 1.5 * OperatorConstants.reverseControls * main_stick.getRawAxis(0);
    SmartDashboard.putNumber("x_mps", x_metersPerSecond);

    //double rightStickLeftRight = main_stick.getRawAxis(4);
    //SmartDashboard.putNumber("joystick_axis_4", rightStickLeftRight);

    double y_metersPerSecond = (Math.abs(main_stick.getRawAxis(1)) < 0.1) ? 0 : 1.5 * OperatorConstants.reverseControls * -main_stick.getRawAxis(1);
    //SmartDashboard.putNumber("y_mps", y_metersPerSecond);

    //double leftStickLeftRight = main_stick.getRawAxis(0);
    double angle_radiansPerSecond =  (Math.abs(main_stick.getRawAxis(4)) < 0.2) ? 0 : OperatorConstants.reverseControls * Math.signum(main_stick.getRawAxis(4)) * 1.5
    * Math.pow(main_stick.getRawAxis(4), 2);
    //SmartDashboard.putNumber("axis_0", leftStickLeftRight);
    //SmartDashboard.putNumber("angle", angle_radiansPerSecond);

    m_drive.setSwerveDrive(
      x_metersPerSecond, 
      y_metersPerSecond, 
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
    new JoystickButton(main_stick, 8).onTrue(
      new InstantCommand(m_drive::resetGyroAngle)
    );

    /*
    new JoystickButton(main_stick, 4).whileTrue(
      new InstantCommand(m_intake::runIntake)
    ); 
    */

    new Trigger(() -> main_stick.getRawButton(4)).whileTrue(
      new RunCommand(m_intake::runIntake).finallyDo(m_intake::stopIntake)
    );

    new Trigger(() -> main_stick.getRawButton(5)).whileTrue(
      new RunCommand(m_intake::extendArm).finallyDo(m_intake::stopArm)
    );

    new Trigger(() -> main_stick.getRawButton(6)).whileTrue(
      new RunCommand(m_intake::retractArm).finallyDo(m_intake::stopArm)
    );
      


    new JoystickButton(second_stick, 1).whileTrue(
      getAutonomousCommand()
      //obviously we dont have an intake setup right now so we havent coded anything for it
      //but heres what im thinking
      //since according to jessie its just the one motor
      //we just do it so like when X button is pressed we set the voltage
      //and multiply it by (second_stick.getRawButtonPressed(x)) ? -1 : 1;
      //so like we can reverse it if theyre pressing another button ykkkkk
    );

    //i would hypothesize functionality of more subsystems
    //but i dont even think we have a good idea of how theyd work yet 1/30/26

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
      //  .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  } 
}
