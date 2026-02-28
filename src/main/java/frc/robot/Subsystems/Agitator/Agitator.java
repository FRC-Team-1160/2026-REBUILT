package frc.robot.Subsystems.Agitator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Port;
import frc.robot.Constants.ShooterConstants;

public class Agitator extends SubsystemBase {
    private final SparkMax agitatorMotor;
    private final RelativeEncoder agitatorEncoder;
    private final SparkMaxConfig agitatorMotorConfig;
    private final AlternateEncoderConfig agitatorEncoderConfig;

    private final SparkMax gateMotor;
    private final RelativeEncoder gateEncoder;
    private final SparkMaxConfig gateMotorConfig;
    private final AlternateEncoderConfig gateEncoderConfig;

    public Agitator() {
        agitatorMotor = new SparkMax(Port.AGITATOR_MOTOR, MotorType.kBrushless);
        agitatorMotorConfig = new SparkMaxConfig();
        agitatorEncoderConfig = new AlternateEncoderConfig();  
        agitatorMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(IntakeConstants.AGITATOR_CURRENT_LIMIT);
        agitatorEncoderConfig.positionConversionFactor(IntakeConstants.EXTENDER_GEAR_RATIO);
        agitatorEncoderConfig.apply(agitatorEncoderConfig);
        agitatorEncoder = agitatorMotor.getEncoder();
        agitatorEncoder.setPosition(0);
        agitatorMotor.configure(agitatorMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        gateMotor = new SparkMax(Port.SHOOTER_GATE_MOTOR, MotorType.kBrushless);
        gateMotorConfig = new SparkMaxConfig();
        gateEncoderConfig = new AlternateEncoderConfig();  
        gateMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(ShooterConstants.GATE_CURRENT_LIMIT);
        gateEncoderConfig.positionConversionFactor(ShooterConstants.GATE_GEAR_RATIO);
        gateEncoderConfig.apply(gateEncoderConfig);
        gateEncoder = gateMotor.getEncoder();
        gateEncoder.setPosition(0);
        gateMotor.configure(gateMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public void runAgitation() {
        gateMotor.setVoltage(ShooterConstants.GATE_VOLTAGE);
        agitatorMotor.setVoltage(IntakeConstants.AGITATOR_VOLTAGE);
    }

    public void stopAgitation() {
        gateMotor.setVoltage(0);
        agitatorMotor.setVoltage(0);
    }
}
