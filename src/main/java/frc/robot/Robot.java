// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Micro;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.TeleopPhases;

public class Robot extends TimedRobot {
  private Command autonomous_command;

  private final RobotContainer m_robot_container;
  public boolean blueAlliance;

  Alert lertlert = new Alert("Alliance Shift Alert", AlertType.kInfo);
  Timer alertRemovalTimer = new Timer();

  public Robot() {
    m_robot_container = new RobotContainer();

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        blueAlliance = (alliance.get() == DriverStation.Alliance.Blue);
    } else {blueAlliance = false;}
  }

  @Override
  public void robotPeriodic() {
    LimelightHelpers.SetRobotOrientation("limelight", 
    m_robot_container.m_drive.getGyroAngle().getDegrees() + 
    (blueAlliance == true ? 0 : 180),0,0,0,0,0);
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomous_command = m_robot_container.getAutonomousCommand();
    if (autonomous_command != null) {
      System.out.println("AUTO INITIALIZED");
      autonomous_command.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomous_command != null) {
      autonomous_command.cancel();
      m_robot_container.m_shooter.stopMotors();
      m_robot_container.m_agitator.stopAgitation();
      m_robot_container.m_agitator.stopGate();
    }

    m_robot_container.allianceShiftTimer.restart();
    m_robot_container.currentPhase = RobotContainer.TeleopPhases.TRANSITION_PHASE;
  }

  @Override
  public void teleopPeriodic() {
     m_robot_container.updateSwerve();

     if (m_robot_container.currentPhase.advancePhase(m_robot_container.allianceShiftTimer)) {
      System.out.println("Alliance Shift Message");
      lertlert.set(true);
      alertRemovalTimer.restart();

      // worst state machine ever made
      m_robot_container.currentPhase = switch (m_robot_container.currentPhase) {
        case TRANSITION_PHASE: yield (RobotContainer.TeleopPhases.SHIFT1);
        case SHIFT1: yield (RobotContainer.TeleopPhases.SHIFT2);
        case SHIFT2: yield (RobotContainer.TeleopPhases.SHIFT3);
        case SHIFT3: yield (RobotContainer.TeleopPhases.SHIFT4);
        case SHIFT4: yield (RobotContainer.TeleopPhases.ENDGAME);

        default: yield m_robot_container.currentPhase;
      };
     }

     if (alertRemovalTimer.hasElapsed(5)) {
      lertlert.set(false);
      alertRemovalTimer.stop();
     }
  }

  @Override
  public void teleopExit() {
    m_robot_container.allianceShiftTimer.stop();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
