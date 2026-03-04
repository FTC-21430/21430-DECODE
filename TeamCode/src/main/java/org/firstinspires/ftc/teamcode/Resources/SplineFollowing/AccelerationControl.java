package org.firstinspires.ftc.teamcode.Resources.SplineFollowing;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Resources.OdometryPacket;

/**
 * Gets the current spline we are following and the time, inferes the target point and outputs how to move the drivetrain to get there
 */
public class AccelerationControl {
    // private constants - ie - robot data like acceleration and stuff needed to translate points, needed acceleration to drivetrain powers
    //private attributes
    private double fwdPower, sidePower, rotPower;
    private final SplinePathInterpreter splinePathInterpreter;

    public AccelerationControl(SplinePathInterpreter splinePathInterpreter){
        this.splinePathInterpreter = splinePathInterpreter;
        // TODO: figure out what robot attributes this class needs
    }

    // computation methods here
    public void update(OdometryPacket odometryPacket){
        SimpleMatrix position = splinePathInterpreter.getRobotPosition(0.0);
        double x = position.get(0);
        double y = position.get(1);
        double yaw = position.get(2);
        // TODO: do this
    }



//TODO: Comment
    public double getForwardPower(){
        return fwdPower;
    }
    public double getSidePower(){
        return sidePower;
    }
    public double getRotationPower(){
        return rotPower;
    }
}
