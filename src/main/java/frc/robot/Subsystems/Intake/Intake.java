package frc.robot.Subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Port;
import com.revrobotics.spark.config.AlternateEncoderConfig;;

public class Intake extends SubsystemBase {

    //public static final Intake instance = new Intake();

    private final SparkMax extenderMotor;
    private final RelativeEncoder encoder;
    private final SparkMaxConfig extenderMotorConfig;
    private final AlternateEncoderConfig extenderEncoderConfig;

    private final SparkMax intakeMotor;
    private final RelativeEncoder intakeEncoder;
    private final SparkMaxConfig intakeMotorConfig;
    private final AlternateEncoderConfig intakeEncoderConfig;

    private boolean isExtendedToggle = false; 

    public Intake() {
        extenderMotor = new SparkMax(Port.INTAKE_EXTENDER_MOTOR, MotorType.kBrushless);
        extenderMotorConfig = new SparkMaxConfig();
        extenderEncoderConfig = new AlternateEncoderConfig();  
        // extenderMotor.restoreFactoryDefaults();
        // extenderMotorConfig = new SparkBaseConfig();
        extenderMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(IntakeConstants.EXTENDER_CURRENT_LIMIT);
        

        extenderEncoderConfig.positionConversionFactor(IntakeConstants.EXTENDER_GEAR_RATIO);
        
        extenderMotorConfig.apply(extenderEncoderConfig);
        encoder = extenderMotor.getEncoder();
        encoder.setPosition(0);
        extenderMotor.configure(extenderMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        // alalalalalallalala yayayayay

        intakeMotor = new SparkMax(Port.INTAKE_MOTOR, MotorType.kBrushless);
        intakeMotorConfig = new SparkMaxConfig();
        intakeEncoderConfig = new AlternateEncoderConfig();  
        intakeMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);
        

        intakeEncoderConfig.positionConversionFactor(IntakeConstants.EXTENDER_GEAR_RATIO);
        
        intakeEncoderConfig.apply(intakeEncoderConfig);
        intakeEncoder = intakeMotor.getEncoder();
        intakeEncoder.setPosition(0);
        intakeMotor.configure(intakeMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    /** Extend intake until max */
    public void extendArm() {
        System.out.println("lalalalalal");
        // if (isFullyExtended()) {
        //     stop();
        //     return;
        // }
        extenderMotor.setVoltage(-IntakeConstants.EXTENDER_SPEED_LIMIT);
        isExtendedToggle = true;
    }

    /** Retract intake until fully retracted */
    public void retractArm() {
        // if (isFullyRetracted()) {
        //     stop();
        //     return;
        // }
        extenderMotor.setVoltage(IntakeConstants.EXTENDER_SPEED_LIMIT);
        isExtendedToggle = false;
    }

    public void runIntake() {
        intakeMotor.setVoltage(IntakeConstants.INTAKE_SPEED);
    }

    public void stopIntake() {
        intakeMotor.setVoltage(0);
    }

    /** Stops motor */
    public void stopArm() {
        extenderMotor.set(0);
    }

    public boolean isFullyExtended() {
        return encoder.getPosition() >= IntakeConstants.EXTENSION_MAX;
    }

    public boolean isFullyRetracted() {
        return encoder.getPosition() <= 0;
    }

    /** Toggle between extend/retract based on last state */
    public void toggle() {
        if (isExtendedToggle || isFullyExtended()) {
            retractArm();
        } else {
            extendArm();
        }
        return;
    }

    /** Returns current extension in encoder units */
    public double getExtension() {
        return encoder.getPosition();
    }

    @Override
    public void periodic() {
        // optional telemetry:
        super.periodic();
        SmartDashboard.putNumber("Intake position", getExtension());
    }

}


