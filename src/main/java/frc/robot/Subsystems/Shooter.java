package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants;
import static frc.robot.Constants.Port;

public class Shooter  extends SubsystemBase {
    // starting with basic stuff
    private TalonFX feed_motor, shoot_lower, shoot_upper;

    // public static final Shooter instance = new Shooter();

    private Shooter() {
        feed_motor = new TalonFX(Port.SHOOTER_FEED_MOTOR);
        shoot_lower = new TalonFX(Port.SHOOTER_LOWER_MOTOR);
        shoot_upper = new TalonFX(Port.SHOOT_UPPER_MOTOR);

        TalonFXConfiguration motor_configs = new TalonFXConfiguration();

        // configure pid
        motor_configs.Slot0 = new Slot0Configs()
        .withKP(0)
        .withKI(0)
        .withKD(0);
        
        // max voltage
        // might not be needed
        motor_configs.Voltage = new VoltageConfigs()
        .withPeakForwardVoltage(5);

        // apply pid configs
        shoot_upper.getConfigurator().apply(motor_configs);

        // set the other motor to follower it
        // note to self make sure youre looking at phoenix6 and not phoenixpro
        shoot_lower.setControl(new Follower(0, MotorAlignmentValue.Opposed));
    }

    public void setShooterSpeed(double target_speed_rotations) {
        shoot_upper.setControl(new VelocityVoltage(target_speed_rotations / ShooterConstants.SHOOTER_GEAR_RATIO));
    }

    public void setFeedMotor(double voltage) {
        feed_motor.setControl(new VoltageOut(voltage));
    }

    // some methods for getting voltage

    public double getFeedMotorVoltage() {
        return feed_motor.getMotorVoltage().getValueAsDouble();
    }

    public double getShootLowerMotorVoltage() {
        return shoot_lower.getMotorVoltage().getValueAsDouble();
    }

    public double getShootUpperMotorVoltage() {
        return shoot_upper.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Feed Motor", getFeedMotorVoltage());
    }
}
