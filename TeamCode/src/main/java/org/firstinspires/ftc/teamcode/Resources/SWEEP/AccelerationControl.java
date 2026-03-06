package org.firstinspires.ftc.teamcode.Resources.SWEEP;

import org.firstinspires.ftc.teamcode.Resources.OdometryPacket;
import org.firstinspires.ftc.teamcode.Resources.PIDController;
import org.firstinspires.ftc.teamcode.Resources.RotationControl;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.SplinePathInterpreter;

/**
 * Gets the current spline we are following and the time, infers the target point and outputs how to move the drivetrain to get there
 */
public class AccelerationControl {
    //Connected classes
    private SplinePathInterpreter splinePathInterpreter;
    private RotationControl rotationControl;
    private PIDController xPID;
    private PIDController yPID;
    // private constants - ie - robot data like acceleration and stuff needed to translate points, needed acceleration to drivetrain powers
    //private attributes
    private double fwdPower, sidePower, rotPower;
    double followSpeed = 1;

    // computation methods here
    public AccelerationControl(SplinePathInterpreter splinePathInterpreter, RotationControl rotationControl) {
        this.splinePathInterpreter = splinePathInterpreter;
        this.rotationControl = rotationControl;
    }

    public void update(OdometryPacket odometryPacket){
        //Things to get:
        /*
        Current Pos, lookAheadPost1, lookAheadPost2, currentVelocity, neededVelocity, currentVelocity, nextNeededVelocity,
        neededAcceleration, scale and frictionConstant
        */
    }
    public void setFollowSpeed(double followSpeed){
        this.followSpeed = followSpeed;
    }

    private void setMotorPwrs(double accelerationX, double accelerationY, double robotAngle){
        //TODO: MAKE THE CODE ALREADY!
        //I'm working on it, chill!
        fwdPower=(xPID.getPower() * Math.sin(Math.toRadians(-robotAngle)) + yPID.getPower() * Math.cos(Math.toRadians(-robotAngle))) * followSpeed * -1;;
        sidePower=(xPID.getPower() * Math.cos(Math.toRadians(-robotAngle)) - yPID.getPower() * Math.sin(Math.toRadians(-robotAngle))) * followSpeed * 1;;
        rotPower=rotationControl.getOutputPower(robotAngle);
    }

    //These functions will get the overall power of the robot in each of their respective directions
    //gets the overall forward power of the robot
    public double getForwardPower(){
        return fwdPower;
    }
    //gets the overall side power of the robot
    public double getSidePower(){
        return sidePower;
    }
    //gets the overall rotational power of the robot
    public double getRotationPower(){
        return rotPower;
    }
}
