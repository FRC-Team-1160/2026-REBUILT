// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.TransferQueue;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ShooterConstants;

public class Robot extends TimedRobot {
  private Command autonomous_command;

  private final RobotContainer m_robot_container;
  public boolean blueAlliance;

  public Robot() {
    m_robot_container = new RobotContainer();
    FollowPathCommand.warmupCommand().schedule();
    SignalLogger.enableAutoLogging(false);
  }

  @Override
  public void robotPeriodic() {
    //LimelightHelpers.SetIMUMode("limelight", 1);
    m_robot_container.updateShooterDistance();
    CommandScheduler.getInstance().run();

  }

  @Override
  public void disabledInit() {}


  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    m_robot_container.m_drive.refreshAlliance();
  }

  @Override
  public void autonomousInit() {
    //m_robot_container.m_drive.resetGyroAngle();
    m_robot_container.m_drive.refreshAlliance();
    m_robot_container.m_drive.autoVisionMeasurement = false;
    autonomous_command = m_robot_container.getAutonomousCommand();
    if (autonomous_command != null) {
      System.out.println("AUTO INITIALIZED");
      //m_robot_container.m_drive.resetGyroAngle();
      autonomous_command.schedule();
    }

  }

  @Override
  public void autonomousPeriodic() {
    m_robot_container.autoAlignSwerve();
  }

  @Override
  public void autonomousExit() {
    m_robot_container.m_shooter.enabled = false;
      m_robot_container.m_agitator.stopAgitation();
      m_robot_container.m_agitator.stopGate();
      //m_robot_container.alignHub.cancel();
      // m_robot_container.StopSwerve.schedule();
  }

  @Override
  public void teleopInit() {
    m_robot_container.m_drive.refreshAlliance();
    if (autonomous_command != null) {
      autonomous_command.cancel();
      m_robot_container.m_shooter.enabled = false;
      m_robot_container.m_agitator.stopAgitation();
      m_robot_container.m_agitator.stopGate();
      // m_robot_container.AlignHub.cancel();
      // m_robot_container.StopSwerve.schedule();
    }
  }

  @Override
  public void teleopPeriodic() {
     m_robot_container.updateSwerve();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
