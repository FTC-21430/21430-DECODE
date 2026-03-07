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

    public static double lookAheadTime1 = 0.5;
    public static double lookAheadTime2 = 1;
    public static double accelRatio;

    private SimpleMatrix robotPosNow;
    private SimpleMatrix robotPosNext;
    private SimpleMatrix robotPosNextNext;
    public AccelerationControl(SplinePathInterpreter splinePathInterpreter, RotationControl rotationControl, double pCon, double iCon, double dCon, ElapsedTime runtime, double accelRatioTemp) {
        this.splinePathInterpreter = splinePathInterpreter;
        this.rotationControl = rotationControl;
        yPID= new PIDController(pCon, iCon, dCon, runtime);
        xPID= new PIDController(pCon, iCon, dCon, runtime);
        accelRatio = (1-accelRatioTemp!=0) ? 1-accelRatioTemp : 1e-7;
    }

    /**
     * This updates acceleration control by giving it all the correct values to stay current
     * @param odometryPacket gives the ingo needed to fully update Acceleration control
     */

    public void update(OdometryPacket odometryPacket){
        double minorRatio = (1-accelRatio) > 0 ? 1-accelRatio:1e-7;
        double velX = odometryPacket.getVelX();
        double velY = odometryPacket.getVelY();
        robotPosNow = splinePathInterpreter.getRobotPosition(0);
        robotPosNext = splinePathInterpreter.getRobotPosition(lookAheadTime1);
        robotPosNextNext = splinePathInterpreter.getRobotPosition(lookAheadTime2);
        double velNeededX = (robotPosNow.get(0) - robotPosNext.get(0)) / lookAheadTime1;
        double velNeededY = (robotPosNow.get(1) - robotPosNext.get(1)) / lookAheadTime1;
        double velNextX = (robotPosNext.get(0) - robotPosNextNext.get(0)) / (lookAheadTime2);
        double velNextY = (robotPosNext.get(1) - robotPosNextNext.get(1)) / (lookAheadTime2);
        double targetVelX = velNeededX*accelRatio + velNextX * minorRatio;
        double targetVelY = velNeededY*accelRatio + velNextY * minorRatio;
        
        setMotorPowers(targetVelX, targetVelY, odometryPacket);
    }

    /**
     * Finds the motor powers required, then sets them with a lot of math
     * @param targetVelocityX - helps with correcting with the X velocity
     * @param targetVelocityY - helps with the Y axis velocity
     * @param odometryPacket - contains the robot position and velocity information needed for these calculations to the PID controllers
     */
    private void setMotorPowers(double targetVelocityX, double targetVelocityY, OdometryPacket odometryPacket){
        double robotAngle = odometryPacket.getYaw();
        xPID.setTarget(targetVelocityX);
        yPID.setTarget(targetVelocityY);
        xPID.update(odometryPacket.getVelX());
        yPID.update(odometryPacket.getVelY());
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
