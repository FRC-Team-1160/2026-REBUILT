package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

    private double inchesFromHub = 0; // only use for testing shooter
    private double bottomRollerVoltage = 2.75; // i think this is the one we should keep constant
    private double bottomRollerTargetRPS = 22.5;

    private TalonFX farBottomRollerMotor = new TalonFX(Port.FAR_SHOOTER_BOTTOM_ROLLER_MOTOR);
    private TalonFX nearBottomRollerMotor = new TalonFX(Port.NEAR_SHOOTER_BOTTOM_ROLLER_MOTOR);
    private TalonFX topRollerMotor = new TalonFX(Port.SHOOTER_TOP_ROLLER_MOTOR);
    Slot2Configs bottomMotor_configs = new Slot2Configs();

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

        bottomMotor_configs.kP = 0.4;
        bottomMotor_configs.kI = 0;
        bottomMotor_configs.kD = 0.75;
        bottomMotor_configs.kV = 0; 
        
        farBottomRollerMotor.getConfigurator().apply(bottomMotor_configs);
        nearBottomRollerMotor.getConfigurator().apply(bottomMotor_configs);
    }

    public double[] getMotorVoltageFromDistanceInches(double distanceFromTargetInches) {
        //distance is from center bottom of bot to ground center of hub
        double topRollerVoltage = (0.0281191*distanceFromTargetInches) + 3.82155;//test function
        return new double[]{bottomRollerVoltage,topRollerVoltage};
        // item 0 is for the bottom voltage, item 1 is for the top voltage
    }

    public void runMotors(double distanceFromTargetMeters) {
        //double[] motorVoltages = getMotorVoltageFromDistanceMeters(distanceFromTargetInches); // should this be an april tag or where were planning on getting the balls to actually land???
        double[] motorVoltages = getMotorVoltageFromDistanceInches(inchesFromHub);
        double bottomVoltage = motorVoltages[0];
        double topVoltage = motorVoltages[1];

        VelocityVoltage bottomMotor_request = new VelocityVoltage(0).withSlot(2);
        VelocityVoltage topMotor_request = new VelocityVoltage(0).withSlot(2);

        nearBottomRollerMotor.setControl(bottomMotor_request.withVelocity(bottomRollerTargetRPS).withFeedForward(0.5));
        farBottomRollerMotor.setControl(bottomMotor_request.withVelocity(bottomRollerTargetRPS).withFeedForward(0.5)); // add feedforward?

        agitatorMotor.setVoltage(ShooterConstants.AGITATOR_VOLTAGE);
        //farBottomRollerMotor.setVoltage(bottomVoltage);
        //nearBottomRollerMotor.set(bottomVoltage);
        topRollerMotor.setVoltage(topVoltage);

        SmartDashboard.putNumber("Top Roller Voltage", topVoltage);
        //SmartDashboard.putNumber("Bottom Roller Far RPS", farBottomRollerMotor.getRotorVelocity());
    }

    public void stopMotors() {
        agitatorMotor.setVoltage(0);
        farBottomRollerMotor.setVoltage(0);
        nearBottomRollerMotor.setVoltage(0);
        topRollerMotor.setVoltage(0);
    }

    // testing for shooter distances

    public void changeDistance(int change) {
        inchesFromHub += change;
        SmartDashboard.putNumber("Inches From Hub", inchesFromHub);
    }

    public void changeBottomRollerVoltage(double change) {
        bottomRollerVoltage += change;
        SmartDashboard.putNumber("Bottom Roller Voltage", bottomRollerVoltage);
    }

    @Override
    public void periodic() {
        // optional telemetry:
        super.periodic();
        SmartDashboard.putNumber("Belt Velocity", encoder.getVelocity());
    }
}
