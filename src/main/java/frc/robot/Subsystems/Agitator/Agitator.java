package frc.robot.Subsystems.Agitator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;                          //config stuff allat 
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
        agitatorMotorConfig.idleMode(SparkMaxConfig.IdleMode.kCoast)
            .smartCurrentLimit(IntakeConstants.AGITATOR_CURRENT_LIMIT);
        agitatorEncoderConfig.positionConversionFactor(3); //agitator gear ratio
        agitatorEncoderConfig.apply(agitatorEncoderConfig);
        agitatorEncoder = agitatorMotor.getEncoder();
        
        gateMotor = new SparkMax(Port.SHOOTER_GATE_MOTOR, MotorType.kBrushless);
        gateMotorConfig = new SparkMaxConfig();
        gateEncoderConfig = new AlternateEncoderConfig();  
        gateMotorConfig.idleMode(SparkMaxConfig.IdleMode.kCoast)
            .smartCurrentLimit(ShooterConstants.GATE_CURRENT_LIMIT);
        gateEncoderConfig.positionConversionFactor(3); //gate gear ratio
        gateEncoderConfig.apply(gateEncoderConfig);
        gateEncoder = gateMotor.getEncoder();

        gateMotor.configure(gateMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        agitatorMotor.configure(agitatorMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    private void runAgitation() {
        agitatorMotor.setVoltage(11);
        gateMotor.setVoltage(-12);
    }

    private void stopAgitation() {
        agitatorMotor.stopMotor();
        gateMotor.stopMotor();
    }

    // commands
    public InstantCommand runMotors = new InstantCommand(() -> runAgitation());
    public InstantCommand stopMotors = new InstantCommand(() -> stopAgitation());

    @Override
    public void periodic() {
        // optional telemetry:
        super.periodic();
        SmartDashboard.putNumber("Agitator Current", agitatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Gate Current", gateMotor.getOutputCurrent());
        SmartDashboard.putNumber("Agitator Velocity", agitatorEncoder.getVelocity());
        SmartDashboard.putNumber("Gate Velocity", gateEncoder.getVelocity());
    }
}
