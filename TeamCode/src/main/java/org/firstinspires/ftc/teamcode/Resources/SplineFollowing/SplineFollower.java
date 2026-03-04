package org.firstinspires.ftc.teamcode.Resources.SplineFollowing;

import org.firstinspires.ftc.teamcode.Firmware.DecodeBot;
import org.firstinspires.ftc.teamcode.Resources.OdometryPacket;

//TODO: Explain the entire purpose and structure of the spine following project
public class SplineFollower {
    //TODO create the needed objects in all corresponding definition categories with comments

    // custom classes
    private final AccelerationControl accelerationControl;
    private final PathPlanning pathPlanner;
    private final SplinePathInterpreter splinePathInterpreter;
    private final DecodeBot robot;

    // FTC Dashboard Variables

    // private constants

    // private variables
    private CubicSplineSegment[] splines;


    /**
     * Constructor for the entire spline following library.
     */
    public SplineFollower(DecodeBot robot){
        // TODO: get to Robot Actions
        this.robot = robot;

        //TODO: figure out what parameters this class needs - ie, robot specific tuning details. - want to make this modular and reusable without changing the library here
        // init
        splinePathInterpreter = new SplinePathInterpreter();
        accelerationControl = new AccelerationControl(splinePathInterpreter);
        pathPlanner = new PathPlanning();

    }

    /**
     * Once all waypoints are defined by opMode, compile all splines together and save path.
     * MUST RUN before first update
     */
    public void computeSplines(){
        this.splines = pathPlanner.generatePath();
    }

    public void update(OdometryPacket odometryPacket){
        // TODO: fill out this method to handle switching between splines
        //  and following them with the accelerationController, then update local
        //  variables to allow for the power getters to work

        accelerationControl.update(new OdometryPacket(0,0,0,0,0));
        splinePathInterpreter.executeActions();
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
    public boolean isPathComplete(){
        return splinePathInterpreter.isPathFinished();
    }
}
