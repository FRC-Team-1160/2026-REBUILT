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
import com.revrobotics.spark.config.AlternateEncoderConfig;
import edu.wpi.first.wpilibj.DigitalInput;

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

    private boolean overridePosition = false;
    private DigitalInput armLimit = new DigitalInput(9);

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
        if (isFullyExtended() || overridePosition == true) {
            extenderMotor.setVoltage(-IntakeConstants.EXTENDER_SPEED_LIMIT);
        } else {
            extenderMotor.setVoltage(0);
        }
    }

    /** Retract intake until fully retracted */
    public void retractArm() {
        if (isFullyRetracted() || overridePosition == true) {
            extenderMotor.setVoltage(IntakeConstants.EXTENDER_SPEED_LIMIT);
        } else {
            extenderMotor.setVoltage(0);
        }
    }

    // intake functions
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

    // getting intake arm positions
    public boolean isFullyExtended() {
        return encoder.getPosition() >= IntakeConstants.EXTENSION_MAX;
    }

    public boolean isFullyRetracted() {
        return encoder.getPosition() <= IntakeConstants.EXTENSION_MIN;
    }

    // for when we need to reset the position with the limit switch
    public void overridePosition() {
        overridePosition = true;
    }

    public void stopPositionOverride() {
        overridePosition = false;
    }

    public void resetArmPosition() {
        encoder.setPosition(IntakeConstants.EXTENSION_MAX);
    }

    /** Returns current extension in encoder units */
    public double getExtension() {
        return encoder.getPosition();
    }

    @Override
    public void periodic() {
        // optional telemetry:
        super.periodic();
        SmartDashboard.putBoolean("Intake Arm Max Reached", !armLimit.get());
        SmartDashboard.putNumber("Intake position", getExtension());
        if (!armLimit.get()) {
            encoder.setPosition(IntakeConstants.EXTENSION_MAX);
        }
    }

}


