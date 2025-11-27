package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;

public class TestingAutonomous extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true);
        robot.odometry.overridePosition(0,0,0);
        waitForStart();
        robot.autoMoveTo(0,24,0, 2);
        robot.autoMoveTo(24,0,0,2);
        robot.autoMoveTo(0,24,90,2);
        robot.autoMoveTo(0,0,-180,1);

    }
}
