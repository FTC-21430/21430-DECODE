package org.firstinspires.ftc.teamcode.Resources.SplineFollowing;

public class Waypoint {
    private final double x,y,angle,velocity;
    private final boolean shouldHoldAngle;
    public Waypoint(double x, double y, double angle, double velocity, boolean shouldHoldAngle){
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.velocity = velocity;
        this.shouldHoldAngle = shouldHoldAngle;
    }

    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getAngle(){
        return angle;
    }
    public double getSpeed(){
        return velocity;
    }
    public boolean shouldHoldAngle(){
        return shouldHoldAngle;
    }
}
