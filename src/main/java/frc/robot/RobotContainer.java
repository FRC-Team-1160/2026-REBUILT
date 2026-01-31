// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.SubsystemManager;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainRealIO;
import frc.robot.Subsystems.DriveTrain.DriveTrainSimIO;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  
  
  //The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(main_stick, 8).onTrue(
      new InstantCommand(DriveTrain.instance::resetGyroAngle)
    );

    new JoystickButton(second_stick, 0).whileTrue(
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

  public void updateSubsystemManager() {
    SubsystemManager.instance.update(new JoystickInputs(
                    main_stick.getRawAxis(5) * (main_stick.getRawButton(6) ? 0.4 : 1), 
                    main_stick.getRawAxis(4) * (main_stick.getRawButton(6) ? 0.4 : 1), 
                    main_stick.getRawAxis(0) * (main_stick.getRawButton(6) ? 0.5 : 1)));
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
