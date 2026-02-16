package frc.robot.Subsystems.Agitator;

import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Port;

public class Agitator extends SubsystemBase {
    public SparkMax agitator_motor;
    private boolean wasNotPressed;

    public Agitator() {
        agitator_motor = new SparkMax(Port.INTAKE_MOTOR, MotorType.kBrushless); //make the agitator mototr
        wasNotPressed = true; //Sets the intial/defual value as true
    }

    //Changes the status of a boolean wasNotPressed
    public void changeAgitatorButtonStatus(){
        if(wasNotPressed){
            wasNotPressed = false;
        }
        else{
            wasNotPressed = true;
        }
    }

    public boolean getAgitatorButtonStatus(){
        return wasNotPressed;
    }
    // Only runs if the boolean wasNotPressed is true
    public void setAgitatorVolts(double volts){
        if(wasNotPressed){
            SmartDashboard.putNumber("agitator speed", volts);
            agitator_motor.set(volts);
        }
    }

    @Override
    public void periodic(){

    }
}
