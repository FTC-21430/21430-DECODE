package org.firstinspires.ftc.teamcode.Resources.SplineFollowing;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Firmware.DecodeBot;
import org.firstinspires.ftc.teamcode.Resources.OdometryPacket;
import org.firstinspires.ftc.teamcode.Resources.PIDFController;

//TODO: Explain the entire purpose and structure of the spine following project
public class SplineFollower {
    //TODO create the needed objects in all corresponding definition categories with comments

    // custom classes
    private final AccelerationControl accelerationControl;
    private final PathPlanning pathPlanner;
    private final DecodeBot robot;
    private ElapsedTime runtime;

    // FTC Dashboard Variables

    // private constants

    // private variables
    private CubicSplineSegment[] splines;


    /**
     * Constructor for the entire spline following library.
     */
    public SplineFollower(DecodeBot robot, PIDFController pidfController, ElapsedTime runtime, double xP, double xI, double xD, double yP, double yI, double yD){
        // TODO: get to Robot Actions
        this.robot = robot;

        //TODO: figure out what parameters this class needs - ie, robot specific tuning details. - want to make this modular and reusable without changing the library here
        // init
        accelerationControl = new AccelerationControl(pidfController, runtime, xP, xI, xD, yP, yI, yD);
        pathPlanner = new PathPlanning();
    }

    public double getXForTime(double time){
        return 0;
        // TODO: make this work
    }
    public double getYForTime(double time){
        return 0;
    }
    public double getYawForTime(double time){
        return 0;
    }

    /**
     * Once all waypoints are defined by opMode, compile all splines together and save path.
     * MUST RUN before first update
     */
    public void computerSplines(){
        splines = pathPlanner.generatePath();
    }

    public void update(OdometryPacket odometryPacket){
        // TODO: fill out this method to handle switching between splines
        //  and following them with the accelerationController, then update local
        //  variables to allow for the power getters to work
    }
    public void setFollowingCoefficients(){
        //TODO: Figure out the needed coefficients, pass these to the Spline Follower
    }

  //TODO: Comment the need for these getters, how that should happen
    public double getSidePower(){
        // TODO: return the side power computed by accelerationControl
        // e.g. return accelerationControl.getSidePower();
        return 0;
    }
    public double getForwardPower(){
        // TODO: return the forward power computed by accelerationControl
        // e.g. return accelerationControl.getForwardPower();
        return 0;
    }
    public double getRotationPower(){
        // TODO: return the rotation power computed by accelerationControl
        // e.g. return accelerationControl.getRotationPower();
        return 0;
    }
}
