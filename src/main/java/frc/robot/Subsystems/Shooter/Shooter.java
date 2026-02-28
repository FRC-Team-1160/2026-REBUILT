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
import frc.robot.Constants.ShooterConstants.BottomMotorConfigs;
import frc.robot.Constants.ShooterConstants.TopMotorConfigs;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
    private double inchesFromHub = 0; // only use for testing shooter
    private double bottomRollerVoltage = 2.75; // i think this is the one we should keep constant
    private double bottomRollerTargetRPS = 22.5;
    private double topRollerTargetRPS = 10;

    private TalonFX farBottomRollerMotor = new TalonFX(Port.FAR_SHOOTER_BOTTOM_ROLLER_MOTOR);
    private TalonFX nearBottomRollerMotor = new TalonFX(Port.NEAR_SHOOTER_BOTTOM_ROLLER_MOTOR);
    private TalonFX topRollerMotor = new TalonFX(Port.SHOOTER_TOP_ROLLER_MOTOR);

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

        SmartDashboard.putNumber("Bottom Roller Target RPS", bottomRollerTargetRPS);
        SmartDashboard.putNumber("Top Roller Target RPS", topRollerTargetRPS);
    }

    public double getTopMotorVelocityFromDistanceInches(double distanceFromTargetInches) {
        //distance is from center bottom of bot to ground center of hub
        double topRollerVelocity = (0.0281191*distanceFromTargetInches) + 3.82155;//test function
        return topRollerVelocity;
        // item 0 is for the bottom voltage, item 1 is for the top voltage
    }

    public void runMotors(double distanceFromTargetMeters) {
        //double[] motorVoltages = getMotorVoltageFromDistanceMeters(distanceFromTargetInches); // should this be an april tag or where were planning on getting the balls to actually land???

        VelocityVoltage bottomMotor_request = new VelocityVoltage(0).withSlot(0);
        VelocityVoltage topMotor_request = new VelocityVoltage(0).withSlot(0);

        // nearBottomRollerMotor.setControl(bottomMotor_request.withVelocity(-bottomRollerTargetRPS).withFeedForward(0.5));
        // farBottomRollerMotor.setControl(bottomMotor_request.withVelocity(bottomRollerTargetRPS).withFeedForward(0.5)); // add feedforward?
        // topRollerMotor.setControl(topMotor_request.withVelocity(topRollerTargetRPS).withFeedForward(0.5));
        //un comment these later!
        //agitatorMotor.setVoltage(ShooterConstants.AGITATOR_VOLTAGE);
    }

    public void stopMotors() {
        // passively stops motors
        // consider actively stopping them using setControl?
        //agitatorMotor.setVoltage(0);
        farBottomRollerMotor.setVoltage(0);
        nearBottomRollerMotor.setVoltage(0);
        topRollerMotor.setVoltage(0);
    }

    // testing for shooter distances

    // public void changeDistance(int change) {
    //     inchesFromHub += change;
    //     SmartDashboard.putNumber("Inches From Hub", inchesFromHub);
    // }

    // public void changeBottomRollerVoltage(double change) {
    //     bottomRollerVoltage += change;
    //     SmartDashboard.putNumber("Bottom Roller Voltage", bottomRollerVoltage);
    // }

    //
    // new functions for testing with rotations per second
    public void changeBottomRollerRPS(double change) {
        bottomRollerTargetRPS += change;
        SmartDashboard.putNumber("Bottom Roller Target RPS", bottomRollerTargetRPS);
    }

    public void changeTopRollerRPS(double change) {
        topRollerTargetRPS += change;
        SmartDashboard.putNumber("Top Roller Target RPS", topRollerTargetRPS);
    }
    //

    @Override
    public void periodic() {
        
    }
}
