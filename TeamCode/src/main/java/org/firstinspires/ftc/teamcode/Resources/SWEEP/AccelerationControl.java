package org.firstinspires.ftc.teamcode.Resources.SWEEP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Resources.OdometryPacket;
import org.firstinspires.ftc.teamcode.Resources.PIDController;
import org.firstinspires.ftc.teamcode.Resources.RotationControl;


/**
 * Gets the current spline we are following and the time, infers the target point and outputs how to move the drivetrain to get there
 */
@Config
public class AccelerationControl {
    //Connected classes
    private SplinePathInterpreter splinePathInterpreter;
    private RotationControl rotationControl;
    private PIDController xPID;
    private PIDController yPID;
    // private constants - ie - robot data like acceleration and stuff needed to translate points, needed acceleration to drivetrain powers
    //private attributes
    private double fwdPower, sidePower, rotPower;
    //Lookahead time is the waypoint time to look ahead. I am using point 2 for the time being
    public static double lookAheadTime1 = 0.5;
    public static double lookAheadTime2 = 1;
    //accel ratio is 
    public static double accelRatio;
    public AccelerationControl(SplinePathInterpreter splinePathInterpreter, RotationControl rotationControl, double pCon, double iCon, double dCon, ElapsedTime runtime, double accelRatio) {
        this.splinePathInterpreter = splinePathInterpreter;
        this.rotationControl = rotationControl;
        yPID= new PIDController(pCon, iCon, dCon, runtime);
        xPID= new PIDController(pCon, iCon, dCon, runtime);
        accelRatio = (1-accelRatio!=0) ? 1-accelRatio : 1e-7;
    }

    /**
     * This updates acceleration control by giving it all the correct values to stay current
     * @param odometryPacket gives the ingo needed to fully update Acceleration control
     */
    //TODO: COMMENT!!
    public void update(OdometryPacket odometryPacket){
        double velX = odometryPacket.getVelX();
        double velY = odometryPacket.getVelY();
        SimpleMatrix robotPosNow = splinePathInterpreter.getRobotPosition(0);
        SimpleMatrix robotPosNext = splinePathInterpreter.getRobotPosition(lookAheadTime1);
        SimpleMatrix robotPosNextNext = splinePathInterpreter.getRobotPosition(lookAheadTime2);
        double velNeededX = (robotPosNow.get(0) - robotPosNext.get(0)) / lookAheadTime1;
        double velNeededY = (robotPosNow.get(1) - robotPosNext.get(1)) / lookAheadTime1;
        double velNextX = (robotPosNext.get(0) - robotPosNextNext.get(0)) / (lookAheadTime2);
        double velNextY = (robotPosNext.get(1) - robotPosNextNext.get(1)) / (lookAheadTime2);
        double neededAccelerationX = (velNextX - velNeededX) / lookAheadTime1 * accelRatio;
        double neededAccelerationY = (velNextY - velNeededY) / lookAheadTime1 * accelRatio;
        
        setMotorPowers(neededAccelerationX, neededAccelerationY, robotPosNow.get(2));
    }

    /**
     * Finds the motor powers required, then sets them with a lot of math
     * @param accelerationX - helps with correcting with the X velocity
     * @param accelerationY - helps with the Y axis velocity
     * @param robotAngle - it's the robots angle, and is used to help the wheels figure out where they are, and how to move accordingly
     */
    private void setMotorPowers(double accelerationX, double accelerationY, double robotAngle){
        xPID.update(accelerationX);
        yPID.update(accelerationY);
        fwdPower=(xPID.getPower() * Math.sin(Math.toRadians(-robotAngle)) + yPID.getPower() * Math.cos(Math.toRadians(-robotAngle))) * -1;;
        sidePower=(xPID.getPower() * Math.cos(Math.toRadians(-robotAngle)) - yPID.getPower() * Math.sin(Math.toRadians(-robotAngle))) * 1;;
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
