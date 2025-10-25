package org.firstinspires.ftc.teamcode.Resources;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

public class RotationControl {
    public double turnRate = 0;
    public PIDController AngleControler;

    public RotationControl(double turnRate, double P,double I,double D, double targetAngle){
        this.turnRate = turnRate;
        AngleControler = new PIDController(P, I, D, new ElapsedTime());
        AngleControler.setTarget(targetAngle);
    }
    public double getOutputPower(double currentAngle){
        AngleControler.update(currentAngle);
        return AngleControler.getPower();


    }
    public void setPIDController(double P, double I, double D){
        AngleControler.updateConstants(P, I, D);
    }
    public List<Double> getPIDController(){
        List<Double> Constants = new ArrayList<Double>();
        Constants.add(AngleControler.powerProportional);
        Constants.add(AngleControler.powerIntegral);
        Constants.add(AngleControler.powerDerivative);
        return Constants;
    }
    public void setTargetAngle(double targetAngle){
        AngleControler.setTarget(targetAngle);
    }
    public double getTargetAngle(){
        return AngleControler.getTarget();
    }
    public void setJoystickControler(double joystickControler){
        AngleControler.setTarget(AngleControler.getTarget()+(joystickControler*turnRate));
    }
}

