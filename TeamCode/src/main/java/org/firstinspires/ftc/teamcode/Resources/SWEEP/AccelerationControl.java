package org.firstinspires.ftc.teamcode.Resources.SWEEP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private Telemetry telemetry;
    // private constants - ie - robot data like acceleration and stuff needed to translate points, needed acceleration to drivetrain powers
    //private attributes
    private double fwdPower, sidePower, rotPower;
    //Lookahead time is the waypoint time to look ahead. I am using point 2 for the time being
    public static double lookAheadTime1 = 0.1; // WARNING: PID Coeffs are dependant on this value!
    public static double accelRatio;
    public AccelerationControl(SplinePathInterpreter splinePathInterpreter, RotationControl rotationControl, double pCon, double iCon, double dCon, double accelRatioTemp, Telemetry telemetry) {
        this.splinePathInterpreter = splinePathInterpreter;
        this.rotationControl = rotationControl;
        this.telemetry = telemetry;
        yPID= new PIDController(pCon, iCon, dCon, new ElapsedTime(),false);
        xPID= new PIDController(pCon, iCon, dCon, new ElapsedTime(),false);

        accelRatio = (1-accelRatioTemp!=0) ? 1-accelRatioTemp : 1e-7;
    }

    /**
     * This updates acceleration control by giving it all the correct values to stay current
     * @param odometryPacket gives the ingo needed to fully update Acceleration control
     */
    public void update(OdometryPacket odometryPacket){
        double minorRatio = (1-accelRatio) > 0 ? 1-accelRatio:1e-7;
        SimpleMatrix robotPosNow = splinePathInterpreter.getRobotPosition(0);
        SimpleMatrix robotPosNext = splinePathInterpreter.getRobotPosition(lookAheadTime1);
        // get a look ahead position
        double posNeededX = robotPosNext.get(0);
        double posNeededY = robotPosNext.get(1);
        setMotorPowers(posNeededX, posNeededY, odometryPacket);
        rotationControl.setTargetAngle(robotPosNow.get(2));
    }
    public void setPIDCoeffs(double p, double i, double d){
        xPID.updatePIDConstants(p,i,d);
        yPID.updatePIDConstants(p,i,d);
    }
    /**
     * Finds the motor powers required, then sets them with a lot of math
     * @param targetPosX - helps with correcting with the X velocity
     * @param targetPosY - helps with the Y axis velocity
     * @param odometryPacket - contains the robot position and velocity information needed for these calculations to the PID controllers
     */
    private void setMotorPowers(double targetPosX, double targetPosY, OdometryPacket odometryPacket){
        double robotAngle = odometryPacket.getYaw();

        xPID.setTarget(targetPosX);
        yPID.setTarget(targetPosY);
        xPID.update(odometryPacket.getX());
        yPID.update(odometryPacket.getY());

        fwdPower=(yPID.getPower() * Math.sin(Math.toRadians(-robotAngle)) + xPID.getPower() * Math.cos(Math.toRadians(-robotAngle))) * 1;
        sidePower=(yPID.getPower() * Math.cos(Math.toRadians(-robotAngle)) - xPID.getPower() * Math.sin(Math.toRadians(-robotAngle))) * -1;
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
