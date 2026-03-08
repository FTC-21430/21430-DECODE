package org.firstinspires.ftc.teamcode.Opmodes.Testing.SWEEP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Opmodes.BaseTeleOp;
@Config
@TeleOp
public class DriveTrainTest extends BaseTeleOp {
    public static double powerForward = 0;
    public static double powerSide = 0;
    public static double powerTurn = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(false,false,false);
        waitForStart();
        robot.driveTrain.fieldCentricDriving(false);
        while (opModeIsActive()){
            robot.odometry.updateOdometry();
            robot.driveTrain.setDrivePower(powerForward,powerSide,powerTurn, 0);
        }
    }
}
