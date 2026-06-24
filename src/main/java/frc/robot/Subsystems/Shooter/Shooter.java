package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotUtils;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Port;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.BottomMotorConfigs;
import frc.robot.Constants.ShooterConstants.TopMotorConfigs;

public class Shooter extends SubsystemBase {
    private TalonFX topMotor = new TalonFX(Port.SHOOTER_TOP_ROLLER_MOTOR);
    private TalonFX bottomMotor = new TalonFX(Port.FAR_SHOOTER_BOTTOM_ROLLER_MOTOR);
    private TalonFX bottomMotor2 = new TalonFX(Port.NEAR_SHOOTER_BOTTOM_ROLLER_MOTOR);

    public Shooter() {
        bottomMotor2.setControl(new Follower(Port.FAR_SHOOTER_BOTTOM_ROLLER_MOTOR, false));
    }

    //Create a method that starts both the topMotor and the bottomMotor
    //*Make sure the motors are going in opposite directions
    //Create a method that stops both the topMotor and the bottomMotor

    @Override
    public void periodic() { 
        super.periodic();
    }
}
