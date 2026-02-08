package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedBeaverAutonomousTesting extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, true,false);
        robot.odometry.recalibrateIMU();

        waitForStart();
        //TODO; tune starting positon for actual robot
        //This is the starting location of the robot
        robot.odometry.overridePosition(10, -20, 135);
    }
}
