package org.firstinspires.ftc.teamcode.Resources;

import java.util.ArrayList;

// The goal of this class is to input in the distance from the april tag and return the needed launch angle and speed to hit the goal
public class TrajectoryKinematics {

    // All math is based on this algorithm developed by Ian and I. run the python simulations here https://colab.research.google.com/drive/1WOSYEZrom-_3M0OcqzIpiTSzPEJKzpaY#scrollTo=M5i8exEXLY_D

    private final double gravity = 9.79; // Gravity constant for denver, change if competition in a different location
    private final double goalHeight;
    private final double finalEntryAngle;

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

    private double initialAngle = 0;
    private double launchMagnitude = 0;
    public TrajectoryKinematics(double goalHeight, double finalEntryAngle){
        this.goalHeight = goalHeight;
        this.finalEntryAngle = finalEntryAngle;
    }

    public void calculateTrajectory(double distanceInches) {
   // All regression functions are calculated using the stored values above and put into Desmos graphing calculator to create a fourth degree regression function!
        initialAngle = angleRegression(distanceInches);
        launchMagnitude = magnitudeRegression(distanceInches);
    }

    private double angleRegression(double distance){
        double a = 1.59536 * Math.pow(10,-7);
        double b = -1.17316 * Math.pow(10,-5);
        double c = -5.85784 * Math.pow(10,-3);
        double d = 0.865212;
        double e = 0.0;

        return a * Math.pow(distance,4)+ b * Math.pow(distance,3) + c * Math.pow(distance,2) + d * distance + e;
    }

    private double magnitudeRegression(double distance){
        double a = 0.0302344;
        double b = 2.50209;
        double c = 1020.88125;

        return a * Math.pow(distance,2) + b * Math.pow(distance,1) + c;
    }


    public double getInitialAngle(){
        return initialAngle;
    }

    public double getLaunchMagnitude(){
        return launchMagnitude;
    }
}
