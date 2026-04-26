package org.firstinspires.ftc.teamcode.Opmodes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;

@Autonomous(name = "HAILMARY-Move-Off-Line", group = "BackupRoutes")
//@Disabled
public class AutoMoveForwardAntEater extends BaseAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true, true,false);;
        waitForStart();
        robot.driveTrain.setDrivePower(0.4, 0, 0, 0);
        sleep(1000);
        robot.driveTrain.setDrivePower(0, 0, 0, 0);
    }
}
