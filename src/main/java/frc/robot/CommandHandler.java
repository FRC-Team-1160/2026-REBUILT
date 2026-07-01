package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Intake.IntakeController;
import frc.robot.Subsystems.Agitator.AgitatorController;
import frc.robot.Subsystems.Shooter.ShooterController;
import frc.robot.Subsystems.DriveTrain.DriveTrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.auto.NamedCommands;

public class CommandHandler {
    ShooterController m_shooterController;
    IntakeController m_intakeController;
    AgitatorController m_agitatorController;
    DriveTrain m_drive;
    
    public CommandHandler(ShooterController m_shooterController, IntakeController m_intakeController, AgitatorController m_agitatorController, DriveTrain m_drive) {
        this.m_shooterController = m_shooterController;
        this.m_intakeController = m_intakeController;
        this.m_agitatorController = m_agitatorController;
        this.m_drive = m_drive;
    }

    public InstantCommand disableVisionMeasurement = new InstantCommand(() -> {
      m_drive.autoVisionMeasurement = false;
    });

    public InstantCommand enableVisionMeasurement = new InstantCommand(() -> {
      m_drive.autoVisionMeasurement = true;
    });

    NamedCommands.registerCommand("Enable Vision", enableVisionMeasurement);
    NamedCommands.registerCommand("Disable Vision", disableVisionMeasurement);
}
