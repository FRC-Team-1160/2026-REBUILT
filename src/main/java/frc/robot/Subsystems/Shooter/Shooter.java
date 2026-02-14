package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Port;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
    private SparkMax agitatorMotor;
    private final RelativeEncoder encoder;
    private final SparkMaxConfig agitatorMotorConfig;
    private final AlternateEncoderConfig agitatorEncoderConfig;

    public Shooter() {
        agitatorMotor = new SparkMax(Port.SHOOTER_INTAKE_MOTOR, MotorType.kBrushless);
        agitatorMotorConfig = new SparkMaxConfig();
        agitatorEncoderConfig = new AlternateEncoderConfig();  
        agitatorMotorConfig.idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(ShooterConstants.AGITATOR_CURRENT_LIMIT);//IntakeConstants.EXTENDER_CURRENT_LIMIT);
        agitatorEncoderConfig.positionConversionFactor(IntakeConstants.EXTENDER_GEAR_RATIO);
        agitatorMotorConfig.apply(agitatorEncoderConfig);
        encoder = agitatorMotor.getEncoder();
        encoder.setPosition(0);
        agitatorMotor.configure(agitatorMotorConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }
        
    private TalonFX bottomRollerMotor = new TalonFX(Port.SHOOTER_BOTTOM_ROLLER_MOTOR);
    private TalonFX topRollerMotor = new TalonFX(Port.SHOOTER_TOP_ROLLER_MOTOR);

    public double[] getMotorVoltageFromDistanceMeters(double distanceFromTargetMeters) {
        // some formula here right????
        // idk the freaking math bro im a freshman
        return new double[]{12,12}; // using 12, 12 for testing atm
        // item 0 is for the bottom voltage, item 1 is for the top voltage
    }

    public void runMotors(double distanceFromTargetMeters) {
        double[] motorVoltages = getMotorVoltageFromDistanceMeters(distanceFromTargetMeters); // should this be an april tag or where were planning on getting the balls to actually land???
        double bottomVoltage = motorVoltages[0];
        double topVoltage = motorVoltages[1];

        agitatorMotor.setVoltage(ShooterConstants.AGITATOR_VOLTAGE);
        bottomRollerMotor.setVoltage(bottomVoltage);
        topRollerMotor.setVoltage(topVoltage);
    }

    public void stopMotors() {
        agitatorMotor.setVoltage(0);
        bottomRollerMotor.setVoltage(0);
        topRollerMotor.setVoltage(0);
    }
}
