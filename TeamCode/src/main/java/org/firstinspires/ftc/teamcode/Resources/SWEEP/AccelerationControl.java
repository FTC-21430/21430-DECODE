package org.firstinspires.ftc.teamcode.Resources.SWEEP;

import org.ejml.simple.SimpleMatrix;
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
    private double followSpeed = 1;
    private double velX;
    private double velY;
    private double velNeededX;
    private double velNeededY;
    private double velNextX;
    private double velNextY;
    private double neededAccelerationX;
    private double neededAccelerationY;
    private double nextNeededAccelerationX;

    private double nextNeededAccelerationY;

    SimpleMatrix robotPosNow;
    SimpleMatrix robotPosNext;
    SimpleMatrix robotPosNextNext;
    public AccelerationControl(SplinePathInterpreter splinePathInterpreter, RotationControl rotationControl) {
        this.splinePathInterpreter = splinePathInterpreter;
        this.rotationControl = rotationControl;
    }

    /**
     * This updates acceleration control by giving it all the correct values to stay current
     * @param odometryPacket gives the ingo needed to fully update Acceleration control
     */

    public void update(OdometryPacket odometryPacket){
        velX = odometryPacket.getVelX();
        velY = odometryPacket.getVelY();
        robotPosNow = splinePathInterpreter.getRobotPosition(0); //TODO: make these times able to be an FTC dashboard constant
        robotPosNext = splinePathInterpreter.getRobotPosition(0.5);
        robotPosNextNext = splinePathInterpreter.getRobotPosition(1);
        velNeededX = robotPosNow.get(0) - robotPosNext.get(0); // TODO: velocity here needs to be in term of (posF-posI)/time This is just posF-posI right now
        velNeededY = robotPosNow.get(1) - robotPosNext.get(1);
        velNextX = robotPosNext.get(0) - robotPosNextNext.get(0);
        velNextY = robotPosNext.get(1) - robotPosNextNext.get(1); // TODO: velocity change in the above comment ends here
        nextNeededAccelerationX = velNextX-velNeededX; //TODO: Acceleration is also change in velocity divided by time. So divide by time! the change in time is just the change in lookahead time. Another reason to make it a variable!
        nextNeededAccelerationY = velNextY-velNeededY;
        nextNeededAccelerationX = velNextX-velNeededX;
        nextNeededAccelerationY = velNextY-velNeededY;

        //TODO: these four are duplicates, neededAcceleration

        //TODO: have a ratio of the two different accelerations

        //TODO: Acceleration = majar ratio of first accel + minor ratio of second accel. such as 0.9 ratio: 0.9 first accel + 0.1 second accel

        //TODO: set the motors Powers should be called now
    }

    /**
     * Updates the followSpeed to stay as accurate as possible
     * @param followSpeed - the rate at which the robot is expected to move
     */
    public void setFollowSpeed(double followSpeed){
        this.followSpeed = followSpeed;
    }

    /**
     * Finds the motor powers required, then sets them with a lot of math
     * @param accelerationX - helps with correcting with the X velocity
     * @param accelerationY - helps with the Y axis velocity
     * @param robotAngle - it's the robots angle, and is used to help the wheels figure out where they are, and how to move accordingly
     */
    private void setMotorPwrs(double accelerationX, double accelerationY, double robotAngle){
        xPID.update(accelerationX);
        yPID.update(accelerationY);
        fwdPower=(xPID.getPower() * Math.sin(Math.toRadians(-robotAngle)) + yPID.getPower() * Math.cos(Math.toRadians(-robotAngle))) * followSpeed * -1;;
        sidePower=(xPID.getPower() * Math.cos(Math.toRadians(-robotAngle)) - yPID.getPower() * Math.sin(Math.toRadians(-robotAngle))) * followSpeed * 1;;
        rotPower=rotationControl.getOutputPower(robotAngle);
    }

    //These functions will get the overall power of the robot in each of their respective directions
    //gets the overall forward power of the robot

    /**
     * It gives classes what motor power to use when driving in auto
     * @return - the forward motor power
     */
    public double getForwardPower(){
        return fwdPower;
    }

    /**
     * It gives classes what motor power to use when driving in auto
     * @return - the side power required
     */
    public double getSidePower(){
        return sidePower;
    }

    /**
     * It gives classes what motor power to use when driving in auto
     * @return - the rotational power required
     */
    public double getRotationPower(){
        return rotPower;
    }
}
