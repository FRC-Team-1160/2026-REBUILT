// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.ExampleSubsystem;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.Shooter.Shooter;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command shootAutoCommand(Shooter shooter, DriveTrain driveTrain) {
    return new FunctionalCommand(
      () -> shooter.runMotors(1), 
      () -> {}, 
      (interrupted) -> shooter.stopMotors(), 
      () -> Commands.waitSeconds(2).isFinished()  // change all this stuff later
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
