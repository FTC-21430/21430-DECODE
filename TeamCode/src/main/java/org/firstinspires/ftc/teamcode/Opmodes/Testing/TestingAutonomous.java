package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
@Autonomous
public class TestingAutonomous extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true);
        waitForStart();
        double start = robot.runtime.seconds();

        if(robot.runtime.seconds() < start +1){
            robot.odometry.overridePosition(0, 0, 0);
            robot.pathFollowing.setFollowSpeed(1);
            robot.pathFollowing.setAutoConstants(0, 0, 0);
            robot.autoMoveTo(5, 0, 2,2);
//        robot.autoMoveTo(24,0,0,2);
//        robot.autoMoveTo(0,24,90,2);
//        robot.autoMoveTo(0,0,-180,1);
//        robot.chill(true,2)
        }
        else {
            robot.chill(true,5);
        }
        }
    }

