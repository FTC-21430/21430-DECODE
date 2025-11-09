package org.firstinspires.ftc.teamcode.Resources;

// The goal of this class is to input in the distance from the april tag and return the needed launch angle and speed to hit the goal
public class TrajectoryKinematics {

    // All math is based on this algorithm developed by Ian and I. run the python simulations here https://colab.research.google.com/drive/1WOSYEZrom-_3M0OcqzIpiTSzPEJKzpaY#scrollTo=M5i8exEXLY_D

    private final double gravity = 9.79; // Gravity constant for denver, change if competition in a different location
    private final double goalHeight;
    private final double finalEntryAngle;

    private double initialAngle = 0;
    private double launchMagnitude = 0;
    public TrajectoryKinematics(double goalHeight, double finalEntryAngle){
        this.goalHeight = goalHeight;
        this.finalEntryAngle = finalEntryAngle;
    }

    public void calculateTrajectory(double distanceInches){
        double inchToMeter = 0.0254;
        double distanceMeters = distanceInches * inchToMeter;

        double angleThroughGoal = Math.toRadians(finalEntryAngle);
        initialAngle = Math.atan((2*goalHeight / distanceMeters) - Math.tan(angleThroughGoal));
        launchMagnitude = Math.sqrt((gravity * distanceMeters)/(Math.pow(Math.cos(initialAngle),2)* (Math.tan(initialAngle) - Math.tan(angleThroughGoal))));
    }

    public double getInitialAngle(){
        return initialAngle;
    }

    public double getLaunchMagnitude(){
        return launchMagnitude;
    }
}
