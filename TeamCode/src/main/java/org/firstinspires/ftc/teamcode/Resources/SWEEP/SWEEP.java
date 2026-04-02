package org.firstinspires.ftc.teamcode.Resources.SWEEP;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Firmware.DecodeBot;
import org.firstinspires.ftc.teamcode.Resources.OdometryPacket;

//TODO: Explain the entire purpose and structure of the spine following project
@Config
public class SWEEP {
    //TODO create the needed objects in all corresponding definition categories with comments

    // custom classes
    private final AccelerationControl accelerationControl;
    //TODO: make it so this can be private and the class compiles it and then passes it into SplineFollower
    public PathPlanning pathPlanner;
    private final SplinePathInterpreter splinePathInterpreter;
    private final DecodeBot robot;

    // FTC Dashboard Variables

    // private constants

    // private variables
    private CubicSplineSegment[] splines;
    private Action[] actions;

    public static double followSpeed = 1;

    /**
     * Constructor for the entire spline following library.
     */
    public SWEEP(DecodeBot robot, double accelRatio, double pCon, double iCon, double dCon){
        this.robot = robot;
        splinePathInterpreter = new SplinePathInterpreter(followSpeed);
        accelerationControl = new AccelerationControl(splinePathInterpreter,robot.rotationControl, pCon, iCon, dCon, accelRatio, robot.telemetry);
        pathPlanner = new PathPlanning(robot);
    }

    /**
     * Once all waypoints are defined by opMode, compile all splines together and save path.
     * MUST RUN before first update
     */
    public void computeSplines(){
        this.splines = pathPlanner.generatePath();
        this.actions = pathPlanner.compileActions();
    }
    public void startPath(){
        splinePathInterpreter.startPath(splines, actions);
        robot.driveTrain.fieldCentricDriving(false);
    }
    public void startPath(double startTime){
        splinePathInterpreter.startPath(splines, actions, startTime);
        robot.driveTrain.fieldCentricDriving(false);
    }
    public void setInterpreterSpeed(double speedRatio){
        splinePathInterpreter.setProgramSpeed(speedRatio);
    }
    public void update(OdometryPacket odometryPacket){
        accelerationControl.update(odometryPacket);
        splinePathInterpreter.executeActions();
        robot.telemetry.addData("targetX",splinePathInterpreter.getRobotPosition(0).get(0));
        robot.telemetry.addData("targetY", splinePathInterpreter.getRobotPosition(0).get(1));
        robot.telemetry.addData("targetAngle", splinePathInterpreter.getRobotPosition(0).get(2));
        robot.telemetry.addData("currentSpline", splinePathInterpreter.getCurrentSplineIndex());

    }
    public void setFollowingCoefficients(double p, double i, double d){
       accelerationControl.setPIDCoeffs(p,i,d);
    }

  //TODO: Comment the need for these getters, how that should happen
    public double getSidePower(){
        return accelerationControl.getSidePower();
    }
    public double getForwardPower(){
        return accelerationControl.getForwardPower();
    }
    public double getRotationPower(){
        return accelerationControl.getRotationPower();
    }
    public boolean isPathComplete(){
        return splinePathInterpreter.isPathFinished();
    }
}