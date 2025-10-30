package org.firstinspires.ftc.teamcode.Resources;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
//This class is used to take the current angle and caulcate what turn power you need to get there
public class RotationControl {
    //The turn Rate is used to determine the rate of turn you need
    public double turnRate = 0;
    //The Angler Controler gets information from the PID Controler in order to figure out the values
    public PIDController angleControler;
    //This function is used to establish the PID controller and setting the target angle
    public RotationControl(double turnRate, double P,double I,double D, double targetAngle){
        this.turnRate = turnRate;
        angleControler = new PIDController(P, I, D, new ElapsedTime());
        angleControler.setTarget(targetAngle);
    }
    //This is used to get the speed it needs to turn
    public double getOutputPower(double currentAngle){
        angleControler.update(currentAngle);
        return angleControler.getPower();


    }
    //This is a setter used to set what the PID values would be
    public void setPIDController(double P, double I, double D){
        angleControler.updateConstants(P, I, D);
    }
    //This is a getter used to what the actual values of these numbers would be
    public List<Double> getPIDController(){
        List<Double> Constants = new ArrayList<Double>();
        Constants.add(angleControler.powerProportional);
        Constants.add(angleControler.powerIntegral);
        Constants.add(angleControler.powerDerivative);
        return Constants;
    }
    //This is used ot set the target angle we need to reach
    public void setTargetAngle(double targetAngle){
        angleControler.setTarget(targetAngle);
    }
    //This gets the target angle we need to reach
    public double getTargetAngle(){
        return angleControler.getTarget();
    }
    //It changes values based on what the joystick tells it to do
    public void changeTargetByJoystick(double joystickValue){
        angleControler.setTarget(angleControler.getTarget()+(joystickValue*turnRate));
    }
}

