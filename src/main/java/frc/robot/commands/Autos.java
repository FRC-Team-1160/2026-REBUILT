// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Subsystems.ExampleSubsystem;
import frc.robot.Subsystems.Agitator.Agitator;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Vision.VisionSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static Command shootAutoCommand(Shooter m_shooter, Agitator m_agitator, VisionSubsystem m_vision) {
    return new FunctionalCommand(
      () -> {
        m_shooter.runMotors(m_vision.getBotToHubDistance());
        m_agitator.runAgitation();
      }, 
      () -> {},
      (interrupted) -> {
        m_shooter.stopMotors();
        m_agitator.stopAgitation();
      },
      () -> Commands.waitSeconds(5).isFinished()  // change all this stuff later
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
