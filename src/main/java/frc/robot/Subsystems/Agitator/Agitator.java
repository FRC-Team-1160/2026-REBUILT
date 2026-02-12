package frc.robot.Subsystems.Agitator;

import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Port;

public class Agitator extends SubsystemBase {
    public SparkMax agitator_motor;

    public Agitator() {
        agitator_motor = new SparkMax(Port.INTAKE_MOTOR, MotorType.kBrushless); //make the agitator mototr
    }

    public void SetAgitatorVolts(double volts){
        SmartDashboard.putNumber("agitator speed", volts);
        agitator_motor.set(volts);
    }

    @Override
    public void periodic(){

    }
}
