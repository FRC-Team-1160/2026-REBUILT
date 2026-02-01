package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
    private TalonFX m_agitator = new TalonFX(HopperConstants.AGITATOR_PORT);

    public Hopper() {

    }

    public void runAgitator(boolean onOff) {
        if (onOff) m_agitator.setControl(new VoltageOut(HopperConstants.AGITATOR_VOLTAGE));
        else m_agitator.setControl(new VoltageOut(0));
    }

    public double getVoltageUsed() {
        return m_agitator.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hopper Voltage", getVoltageUsed());
    }
}
