package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;
@Autonomous
@Disabled
public class TestingAutonomous extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, true,false);
        waitForStart();
        double start = robot.runtime.seconds();

        robot.odometry.overridePosition(0, 0, 0);
        robot.pathFollowing.setFollowSpeed(1);
        robot.autoMoveTo(0, 24, 0,2);
        robot.autoMoveTo(24,0,90,2);
        robot.autoMoveTo(0,-24,90,2);
        robot.autoMoveTo(0,0,-180,1);
        robot.chill(true,2);
        }
    }

