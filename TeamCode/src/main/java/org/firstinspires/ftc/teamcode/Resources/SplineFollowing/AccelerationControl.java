package org.firstinspires.ftc.teamcode.Resources.SplineFollowing;

import org.firstinspires.ftc.teamcode.Resources.OdometryPacket;

/**
 * Gets the current spline we are following and the time, inferes the target point and outputs how to move the drivetrain to get there
 */
public class AccelerationControl {
    // private constants - ie - robot data like acceleration and stuff needed to translate points, needed acceleration to drivetrain powers
    //private attributes
    private double fwdPower, sidePower, rotPower;

    public AccelerationControl(){
        // TODO: figure out what robot attributes this class needs
    }

    // computation methods here
    public void update(OdometryPacket odometryPacket, CubicPolynomial currentSpline, double currentTime){
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
