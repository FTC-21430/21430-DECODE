package org.firstinspires.ftc.teamcode.Resources;

import java.util.ArrayList;

// The goal of this class is to input in the distance from the april tag and return the needed launch angle and speed to hit the goal
public class TrajectoryKinematics {
    // A list of values that the angles have been measured at. The order of these values need to be related to the angles and magnitudes list.
    // TODO: these values need to be tuned in a sample size of every 5 inches. Current values were from early season prototype testing
    // NOTE: these values are not actually used, but are the values that the regression functions were made. These values were really calculated in Desmos Graphing Calculator
    private final double[] distances = {
            24,
            81,
            94,
            120,

    };

    private final double[] angles = {
            20,
            35,
            35,
            35
    };
    private final double[] magnitudes = {
            1100,
            1400,
            1550,
            1750,
    };

    private double autonomousLaunchDecrement = 0;

    // The return value for the angle of the ramp. - in degrees
    private double initialAngle = 0;
    // the return value for how fast the flywheel should go to achieve a launch. In degrees per second
    private double launchMagnitude = 0;
    public TrajectoryKinematics(boolean isAuto){
        autonomousLaunchDecrement = isAuto? 30:30;
    }

    // the getBearingToTag is used to turn the robot so it is facing the center of the tag
    public double getBearingToTag(String mode, Boolean isAuto, double x, double y){
        double angle;
        double posX = x;
        double posY = y;
        double goalX = 0;
        double goalY = 0;
        double coordinate_correction_offset = 90;
        double FLYWHEEL_OFFSET = 0;


        switch (mode) {
            case "red":
                goalX = -67.2;
                goalY = 60;
                FLYWHEEL_OFFSET = Math.toDegrees(Math.atan(5/123.5));
                break;
            case "blue":
                goalX = -60.2;
                goalY = -60;
                FLYWHEEL_OFFSET = Math.toDegrees(Math.atan(5/123.5));
                break;
        }

        double x_difference = posX-goalX;
        double y_difference = posY-goalY;

        angle = 90+Math.toDegrees(Math.atan2(y_difference,x_difference));
        return angle - FLYWHEEL_OFFSET + coordinate_correction_offset;

    }

    /**
     * Updates the return variables based on the two regression functions
     * @param distanceInches Inches, the distance from the front (camera side) of the robot and the april tag.
     */
    public void calculateTrajectory(double distanceInches) {
   // All regression functions are calculated using the stored values above and put into Desmos graphing calculator to create a fourth degree regression function!
        initialAngle = angleRegression(distanceInches);
        launchMagnitude = magnitudeRegression(distanceInches);
    }

    /**
     * the regression function tuned by Desmos calculator and based on real world testing data.
     * @param distance Inches, the distance from the front (camera side) of the robot and the april tag.
     * @return returns the angle the ramp needs to be to hit the goal from the given distance
     */
    private double angleRegression(double distance){
        // values a-e represent the tuning values of this 4th degree polynomial calculated by Desmos using the distance and angle values above
       double a = -0.18684;
       double b = 67.55822;
       return a * distance + b;
    }

    /**
     * The regression function for flywheel speed tuned by Desmos calculator and based on real world testing data.
     * @param distance Inches, the distance from the front (camera side) of the robot and the april tag.
     * @return degrees per second, the speed the flywheel needs to be to hit the goal from the given distance
     */
    private double magnitudeRegression(double distance){
        //quadratic tuning values
        double a = 0.0000439265;
        double b = 0.00855249;
        double c = 3.53418;
        double d = 1029.23661;

        // Math.pow is the exponent function, this is a second degree polynomial that is tuning based on real world testing
        return a * Math.pow(distance,3) + b * Math.pow(distance,2) + c * Math.pow(distance,1) + d - autonomousLaunchDecrement;
    }

    /**
     * Getter for initial angle
     * @return degrees
     */
    public double getInitialAngle(){
        return initialAngle;
    }

    /**
     * getter for the flywheel speed
     * @return degrees per second
     */
    public double getLaunchMagnitude(){
        return launchMagnitude;
    }}
