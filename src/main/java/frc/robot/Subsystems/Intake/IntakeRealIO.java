package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants;
import static frc.robot.Constants.Port;

public class IntakeRealIO extends Intake {

    private TalonFX intake_motor = new TalonFX(Port.INTAKE_MOTOR);
    private TalonFX deploy_motor = new TalonFX(Port.INTAKE_DEPLOY_MOTOR);

    // will change later to actual encoder
    private DutyCycleEncoder deploy_encoder = new DutyCycleEncoder(IntakeConstants.DEPLOY_ENCODER_CHANNEL);

    // idk how theyre going to deploy the intake

    // public static final Intake instance = new Intake();

    protected IntakeRealIO() {}

    public void setDeployVoltage(double voltage) {
        double motor_rps = deploy_motor.getVelocity().getValueAsDouble();

        if (deploy_encoder.get() < IntakeConstants.DEPLOY_MIN_ROTATIONS && motor_rps > 0)
            return;

        if (deploy_encoder.get() > IntakeConstants.DEPLOY_MAX_ROTATIONS && motor_rps < 0)
            return; 

        deploy_motor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setIntakeVoltage(double voltage) {
        intake_motor.setControl(new VoltageOut(voltage));
    }

    // would be better for commands to use this method instead
    // unless we want it to go backward
    public void runIntake(boolean on) {
        if (on) setIntakeVoltage(IntakeConstants.INTAKE_VOLTAGE);
        else setIntakeVoltage(0);
    }

    @Override
    public void runIntake(boolean on, boolean forward) {
        double dir = forward ? 1 : -1;
        double voltage = on ? IntakeConstants.INTAKE_VOLTAGE : 0;

        setIntakeVoltage(voltage * dir);
    }
    
    public double getVoltageUsed() {
        return intake_motor.getMotorVoltage().getValueAsDouble() +
            deploy_motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Voltage", getVoltageUsed());
        
        SmartDashboard.putNumber("Intake Deploy Position", deploy_encoder.get());
        SmartDashboard.putNumber("Intake Deploy Speed", deploy_motor.getVelocity().getValueAsDouble());
    }
}
