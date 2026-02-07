package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;

@TeleOp
@Config
@Disabled
public class MovementPIDTuning extends BaseTeleOp {

    public static double P,I,D;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true);
        waitForStart();
        robot.odometry.overridePosition(0,0,0);
        while (opModeIsActive()){
            if (gamepad1.crossWasPressed()){
                robot.pathFollowing.setTargetPosition(0,0);
            }
            if (gamepad1.circleWasPressed()){
                robot.pathFollowing.setTargetPosition(30,0);
            }
            if (gamepad1.triangleWasPressed()){
                robot.pathFollowing.setTargetPosition(30,30);
            }
            robot.chill(true,0.01);
            robot.pathFollowing.setAutoConstants(P,I,D);
            robot.bulkSensorBucket.clearCache();
        }
    }
}
