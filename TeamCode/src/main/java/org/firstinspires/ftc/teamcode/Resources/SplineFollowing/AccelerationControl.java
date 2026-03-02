package org.firstinspires.ftc.teamcode.Resources.SplineFollowing;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Resources.OdometryPacket;
import org.firstinspires.ftc.teamcode.Resources.PIDController;
import org.firstinspires.ftc.teamcode.Resources.PIDFController;

/**
 * Gets the current spline we are following and the time, infers the target point and outputs how to move the drivetrain to get there
 */
public class AccelerationControl {
    //Connected classes
    private PIDController pidController = null;
    private ElapsedTime runtime;
    // private constants - ie - robot data like acceleration and stuff needed to translate points, needed acceleration to drivetrain powers
    private double p;
    private double i;
    private double d;
    //private attributes
    private double followSpeed = 1;
    private double followTolerance = 1;
    private double fwdPower, sidePower, rotPower;

    public AccelerationControl(PIDController pidController, ElapsedTime runtime) {
        this.pidController = pidController;
        this.runtime = runtime;
    }

    // computation methods here
    /*
     *There a few main functions that path following uses:
        *followPath, setTargetPosition, setFollowSpeed, setAutoConstants, isWithinTargetTolerance,
        *setFollowTolerance, and the getters for speed.
     *In these, only a few are relevant, such as:
        *setFollowSpeed, isWithinTargetTolerance, setFollowTolerance, and minor adjustments to followPath
     *As an error fixing class, the others arent relevant, and will be in other classes.
     *followPath will go into SplineFollower, but will need some error management from AccelerationControl.
     *The necessary methods will include:
        *setFollowSpeed, isWithinTargetTolerance, setFollowTolerance, along with some PIDF help for error control.
     *The old code is as follows:
     */
        public void setFollowSpeed(double speed){
        followSpeed = speed;
        }

        public boolean isWithinTargetTolerance(double robotX, double robotY){
        double distance = Math.sqrt(Math.pow(xPID.getTarget() - robotX, 2 ) + Math.pow(yPID.getTarget() - robotY,2));
        return distance <= followTolerance;
        }

        public void setFollowTolerance(double tolerance){
        followTolerance = tolerance;
        }

    public void update(OdometryPacket odometryPacket, CubicPolynomial currentSpline, double currentTime){
        // TODO: do this
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
