package frc.robot.Subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Port;

import static frc.robot.Constants.IntakeConstants;
import static frc.robot.Constants.Port;

public class IntakeRealIO extends Intake {

    // i swear i saw a third sparkmax motor controller
    private SparkMax intake_motor = new SparkMax(Port.INTAKE_MOTOR, MotorType.kBrushless);
    private SparkMax deploy_motor = new SparkMax(Port.INTAKE_DEPLOY_MOTOR, MotorType.kBrushless);

    // private SparkRelativeEncoder deploy_encoder = deploy_motor.getAlternateEncoder();
    private SparkAbsoluteEncoder deploy_encoder = deploy_motor.getAbsoluteEncoder();

    protected IntakeRealIO() {
        // the motor might actually have an alternate encoder 
        AbsoluteEncoderConfig encoder_config = new AbsoluteEncoderConfig()
        .positionConversionFactor(1);

        // make a motor config and apply the encoder config to its encoder
        SparkMaxConfig motor_config = new SparkMaxConfig();
        motor_config.absoluteEncoder.apply(encoder_config);

        // i have no idea what these parameters mean
        deploy_motor.configure(motor_config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    
    // neo : pinion 40:1 maybe??
    @Override
    public void setDeployVoltage(double voltage) {
        double motor_rpm = deploy_encoder.getVelocity();
        double motor_pos = deploy_encoder.getPosition();

        if (motor_pos < IntakeConstants.DEPLOY_MIN_ROTATIONS && motor_rpm > 0) {
            voltage = 0;
        } else if (motor_pos > IntakeConstants.DEPLOY_MAX_ROTATIONS && motor_rpm < 0) {
            voltage = 0;
        }    

        deploy_motor.setVoltage(voltage);
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        intake_motor.setVoltage(voltage);
    }

    @Override
    public void runIntake(boolean on) {runIntake(on, true);}

    @Override
    public void runIntake(boolean on, boolean forward) {
        SmartDashboard.putBoolean("Intake Command", on);

        double dir = forward ? 1 : -1;
        double voltage = on ? IntakeConstants.INTAKE_VOLTAGE : 0;

        setIntakeVoltage(voltage * dir);
    }
    
    public double getVoltageUsed() {
        return intake_motor.getBusVoltage() +
            deploy_motor.getBusVoltage();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Voltage", getVoltageUsed());
        
        SmartDashboard.putNumber("Intake Deploy Position", deploy_encoder.getPosition());
        SmartDashboard.putNumber("Intake Deploy Velocity", deploy_encoder.getVelocity());
    }
}
