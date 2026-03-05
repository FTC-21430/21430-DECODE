package org.firstinspires.ftc.teamcode.Resources.SplineFollowing;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Resources.OdometryPacket;
import org.firstinspires.ftc.teamcode.Resources.PIDController;
import org.firstinspires.ftc.teamcode.Resources.PIDFController;
import org.firstinspires.ftc.teamcode.Resources.SWEEP.SplinePathInterpreter;

/**
 * Gets the current spline we are following and the time, infers the target point and outputs how to move the drivetrain to get there
 */
public class AccelerationControl {
    //Connected classes
    private PIDController pidController = null;
    private ElapsedTime runtime;
    private SplinePathInterpreter splinePathInterpreter;
    // private constants - ie - robot data like acceleration and stuff needed to translate points, needed acceleration to drivetrain powers
    //private attributes
    private double followSpeed = 1;
    private double followTolerance = 1;
    private double fwdPower, sidePower, rotPower;

    // computation methods here
    public AccelerationControl(SplinePathInterpreter splinePathInterpreter) {
        this.pidController = pidController;
        this.runtime = runtime;
        this.splinePathInterpreter = splinePathInterpreter;
    }

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
        double distance = Math.sqrt(Math.pow(pidController.getTarget() - robotX, 2 ) + Math.pow(pidController.getTarget() - robotY,2));
        return distance <= followTolerance;
        }

        public void setFollowTolerance(double tolerance){
        followTolerance = tolerance;
        }

        public void update(OdometryPacket odometryPacket, SplineFollower splineFollower, double currentTime){
        // TODO: do this
    }

    public void setMotorPwrs(){
        fwdPower=0;
        sidePower=0;
        rotPower=0;
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
