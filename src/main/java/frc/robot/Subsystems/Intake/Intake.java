package frc.robot.Subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
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

    private TalonFX intakeMotor = new TalonFX(Port.INTAKE_MOTOR);

    private DigitalInput armLimit = new DigitalInput(9);
    private boolean limitReached = !armLimit.get();
    private boolean positionSet = false;

    private enum direction {
        EXTENDING,
        RETRACTING
    }

    private direction currentDirection = direction.RETRACTING;

    public Intake() {
        extenderMotor = new SparkMax(Port.INTAKE_EXTENDER_MOTOR, MotorType.kBrushless);
        extenderMotorConfig = new SparkMaxConfig();
        extenderEncoderConfig = new AlternateEncoderConfig();  
        // extenderMotor.restoreFactoryDefaults();
        // extenderMotorConfig = new SparkBaseConfig();
        extenderMotorConfig.idleMode(SparkMaxConfig.IdleMode.kCoast)
            .smartCurrentLimit(IntakeConstants.EXTENDER_CURRENT_LIMIT);
        extenderEncoderConfig.positionConversionFactor(IntakeConstants.EXTENDER_GEAR_RATIO);
        extenderMotorConfig.apply(extenderEncoderConfig);
        encoder = extenderMotor.getEncoder();
        encoder.setPosition(0);
        extenderMotor.configure(extenderMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    // intake functions
    public void runIntake(int mult) {
        intakeMotor.setVoltage(IntakeConstants.INTAKE_VOLTAGE * mult);
    }

    /** Stops motor */
    public void stopArm() {
        extenderMotor.setVoltage(0);
    }

    /** Extend intake until max */
    public void extendArm() {
        if (!isFullyExtended() || !positionSet) {
            currentDirection = direction.EXTENDING;
            extenderMotor.setVoltage(-IntakeConstants.EXTENDER_VOLTAGE);
        } else {
            extenderMotor.setVoltage(0);
        }
    }

    /** Retract intake until fully retracted */
    public void retractArm() {
        if (!isFullyRetracted() || !positionSet) {
            currentDirection = direction.RETRACTING;
            extenderMotor.setVoltage(IntakeConstants.EXTENDER_VOLTAGE);
        } else {
            extenderMotor.setVoltage(0);
        }
    }

    public void toggleArmExtension() {
        switch (currentDirection) {
            case EXTENDING:
            retractArm();
            break;

            case RETRACTING:
            extendArm();
            break;
        }
    }

    // getting intake arm positions
    public boolean isFullyRetracted() {
        return positionSet ? encoder.getPosition() >= IntakeConstants.EXTENSION_MIN : false;
    }

    public boolean isFullyExtended() {
        return positionSet ? (encoder.getPosition() <= IntakeConstants.EXTENSION_MAX) : false;
    }

    /** Returns current extension in encoder units */

    @Override
    public void periodic() {
        // optional telemetry:
        super.periodic();
        limitReached = armLimit.get();
        SmartDashboard.putBoolean("Intake Arm Max Reached", limitReached);
        SmartDashboard.putNumber("Intake position", encoder.getPosition());
        
        if (limitReached && !positionSet) {
            positionSet = true;
            encoder.setPosition(IntakeConstants.EXTENSION_MAX);
        }

        if (positionSet && 
        ((encoder.getPosition() <= IntakeConstants.EXTENSION_MAX && currentDirection == direction.EXTENDING)|| 
        (encoder.getPosition() >= IntakeConstants.EXTENSION_MIN && currentDirection == direction.RETRACTING))) {
            stopArm();
        }

        if ((encoder.getPosition() <= -23) && positionSet) {
            runIntake(1);
        } else {intakeMotor.stopMotor();}
    }
}


