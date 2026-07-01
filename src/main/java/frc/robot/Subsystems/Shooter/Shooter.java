package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    public void startMotorsyah () {
        topMotor.setVoltage(2.5);
        bottomMotor.setVoltage(-2.5);
    }

    public void stopMotorsyah(){
        topMotor.setVoltage(0);
        bottomMotor.setVoltage(0);
    }
    

    
 public InstantCommand startMotorsyah = new InstantCommand(this::startMotorsyah);
 public InstantCommand stopMotorsyah = new InstantCommand(this::stopMotorsyah);

    @Override
    public void periodic() { 
        super.periodic();
    }
}
