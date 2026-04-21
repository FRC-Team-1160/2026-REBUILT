package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Port;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.BottomMotorConfigs;
import frc.robot.Constants.ShooterConstants.TopMotorConfigs;

public class Shooter extends SubsystemBase {
    private double bottomRollerFF = 0;
    private double bottomRollerTargetRPS = -25;

    private TalonFX farBottomRollerMotor = new TalonFX(Port.FAR_SHOOTER_BOTTOM_ROLLER_MOTOR);
    private TalonFX nearBottomRollerMotor = new TalonFX(Port.NEAR_SHOOTER_BOTTOM_ROLLER_MOTOR);
    private TalonFX topRollerMotor = new TalonFX(Port.SHOOTER_TOP_ROLLER_MOTOR);

    public boolean enabled = false;

    public enum SHOOTER_MODES {
        AUTO_DISTANCE,
        STATIC_DISTANCE,
        REVERSED
    }

    private SHOOTER_MODES currentModes = SHOOTER_MODES.AUTO_DISTANCE;

    private VelocityVoltage bottomMotor_request = new VelocityVoltage(0).withSlot(0);
    private VelocityVoltage topMotor_request = new VelocityVoltage(0).withSlot(0);

    public double distanceFromTargetInches = 0;

    public Shooter() {
        TalonFXConfiguration topMotor_configs = new TalonFXConfiguration();
        TalonFXConfiguration bottomMotor_configs = new TalonFXConfiguration();

        topMotor_configs.Slot0 = new Slot0Configs()
        .withKP(TopMotorConfigs.kP)
        .withKI(TopMotorConfigs.kI)
        .withKD(TopMotorConfigs.kD)
        .withKS(TopMotorConfigs.kS)
        .withKV(TopMotorConfigs.kV)
        .withKA(TopMotorConfigs.kA)
        .withKG(TopMotorConfigs.kG);

        bottomMotor_configs.Slot0 = new Slot0Configs()
        .withKP(BottomMotorConfigs.kP)
        .withKI(BottomMotorConfigs.kI)
        .withKD(BottomMotorConfigs.kD)
        .withKS(BottomMotorConfigs.kS)
        .withKV(BottomMotorConfigs.kV)
        .withKA(BottomMotorConfigs.kA)
        .withKG(BottomMotorConfigs.kG);
        
        farBottomRollerMotor.getConfigurator().apply(bottomMotor_configs);
        nearBottomRollerMotor.getConfigurator().apply(bottomMotor_configs);
        topRollerMotor.getConfigurator().apply(topMotor_configs);

        farBottomRollerMotor.setControl(new Follower(Port.NEAR_SHOOTER_BOTTOM_ROLLER_MOTOR, false));
    }

    private double getTopMotorRPSFromDistanceInches(double distanceFromTargetInches) {
        distanceFromTargetInches += 5; // just offsetting distance to shoot slightly further
        double topRollerRPS = 0.00116398*(Math.pow(distanceFromTargetInches, 2)) + 0.272546*distanceFromTargetInches + 15.25056;
        return topRollerRPS;
    }

    private double getVoltageFromRPS(double rps) {
        return 0.2203 + 0.1107*rps;
    }

    public void setMode(SHOOTER_MODES mode) {
        currentModes = mode;

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hub Distance Shooter", distanceFromTargetInches);
        
        SmartDashboard.putNumber("Bottom Roller Actual RPS", nearBottomRollerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Top Roller Actual RPS", topRollerMotor.getVelocity().getValueAsDouble());

        if (enabled) {
            double topRollerRPS = getTopMotorRPSFromDistanceInches(distanceFromTargetInches);
            double bottomRollerRPS = bottomRollerTargetRPS;

            //topRollerRPS = 20;
            SmartDashboard.putNumber("Bottom Roller Target RPS", bottomRollerRPS);
            SmartDashboard.putNumber("Top Roller Target RPS", topRollerRPS);
            bottomRollerRPS = -25;
            bottomRollerFF = -3.5;
            
            switch (currentModes) {
                case STATIC_DISTANCE:
                topRollerRPS = getTopMotorRPSFromDistanceInches(ShooterConstants.STATIC_DISTANCE_INCHES);
                break;
                case REVERSED:
                topRollerRPS = -20;
                bottomRollerRPS *= -1;
                bottomRollerFF *= -1;
                break;
                default:
                break;
            }

            // bottomRollerRPS *= 0.5;
            // bottomRollerFF *= 0.5; //juggle

            nearBottomRollerMotor.setControl(bottomMotor_request.withVelocity(-23).withFeedForward(bottomRollerFF));
            topRollerMotor.setControl(topMotor_request.withVelocity(topRollerRPS).withFeedForward(getVoltageFromRPS(topRollerRPS)));
        } else {
            nearBottomRollerMotor.stopMotor();
            topRollerMotor.stopMotor();
        }
    }
}
