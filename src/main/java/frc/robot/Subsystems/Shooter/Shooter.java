package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Port;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.BottomMotorConfigs;
import frc.robot.Constants.ShooterConstants.TopMotorConfigs;

public class Shooter extends SubsystemBase {
    private TalonFX farBottomRollerMotor = new TalonFX(Port.FAR_SHOOTER_BOTTOM_ROLLER_MOTOR);
    private TalonFX nearBottomRollerMotor = new TalonFX(Port.NEAR_SHOOTER_BOTTOM_ROLLER_MOTOR);
    private TalonFX topRollerMotor = new TalonFX(Port.SHOOTER_TOP_ROLLER_MOTOR);

    private VelocityVoltage bottomMotor_request = new VelocityVoltage(0).withSlot(0);
    private VelocityVoltage topMotor_request = new VelocityVoltage(0).withSlot(0);

    public double distanceFromTargetInches = 0;
    public boolean enabled = false;

    public enum shooterModes {
        AUTO_DISTANCE,
        STATIC_DISTANCE,
        AGAINST_HUB,
        REVERSE
    }

    private shooterModes currentShooterMode = shooterModes.AUTO_DISTANCE;

    /**
     * Setting up the configurations for the shooter motors
     */
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

        farBottomRollerMotor.setControl(new Follower(Port.NEAR_SHOOTER_BOTTOM_ROLLER_MOTOR, MotorAlignmentValue.Aligned));
    }

    /**
     * Get the needed RPS of the top shooter motor in order to shoot the balls from distanceFromTargetInches
     * Formula was found through a regression of testing the best voltages for varying distances from the hub
     * @param distanceFromTargetInches the distance from the center of the bot to the center of the hub
     * @return the top roller rotations per second needed to shoot the balls the correct distance
     */
    private double getTopMotorRPSFromDistanceInches(double distanceFromTargetInches) {
        distanceFromTargetInches += 5;
        double topRollerRPS = 0.00116398*(Math.pow(distanceFromTargetInches, 2)) + 0.272546*distanceFromTargetInches + 15.25056;
        return topRollerRPS;
    }

    /**
     * Return the voltage the top roller must be set to in order to reach the target RPS
     * Formula was found through a regression of what voltages correlated to what RPS
     * @param targetRPS The targetted rotations per second of the top motor
     * @return the voltage needed to reach targetRPS
     */
    private double getVoltageFromRPS(double targetRPS) {
        return 0.2203 + 0.1107*targetRPS;
    }

    /**
     * Set the current shooter mode to the passed in mode
     * @param mode The mode you want to set the shooter to, based on shooterModes enum
     */
    public void setShooterMode(shooterModes mode) {
        currentShooterMode = mode;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hub Distance Shooter", distanceFromTargetInches);
        
        SmartDashboard.putNumber("Bottom Roller Actual RPS", nearBottomRollerMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Top Roller Actual RPS", topRollerMotor.getVelocity().getValueAsDouble());

        if (enabled) {
            double topRollerRPS = getTopMotorRPSFromDistanceInches(distanceFromTargetInches);
            double bottomRollerRPS = -23;
            double bottomRollerFF = -3.5;
            
            switch (currentShooterMode) {
                case STATIC_DISTANCE:
                    topRollerRPS = getTopMotorRPSFromDistanceInches(ShooterConstants.STATIC_DISTANCE_INCHES);
                    break;
                case AGAINST_HUB:
                    topRollerRPS = 13;
                    bottomRollerRPS = -30;
                    bottomRollerFF = -3.1;
                    break;
                case REVERSE:
                    topRollerRPS = -20;
                    bottomRollerRPS *= -1;
                    bottomRollerFF *= -1;
                    break;
                default: // auto distance, which we set earlier when we create these variables
                    break;
            }

            SmartDashboard.putNumber("Bottom Roller Target RPS", bottomRollerRPS);
            SmartDashboard.putNumber("Top Roller Target RPS", topRollerRPS);
                
            nearBottomRollerMotor.setControl(bottomMotor_request.withVelocity(bottomRollerRPS).withFeedForward(bottomRollerFF));
            topRollerMotor.setControl(topMotor_request.withVelocity(topRollerRPS).withFeedForward(getVoltageFromRPS(topRollerRPS)));
        } else {
            nearBottomRollerMotor.stopMotor();
            topRollerMotor.stopMotor();
        }
    }
}
