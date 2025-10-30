<<<<<<< Updated upstream
package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class CompetitionTeleop extends BaseTeleOp {

    // The main code that runs during init
    @Override
    public void runOpMode() throws InterruptedException {

        // initializes the robot without resetting the odometry
        initialize(false, false);

        waitForStart();

        while(opModeIsActive()) {

            // get and update functions
            robot.updateLoopTime();
            robot.odometry.updateOdometry();

            // resets Field Centric Driving
            if (gamepad1.share) {
                robot.IMUReset();
            }

            robot.driveTrain.setDrivePower(-gamepad1.left_stick_y, gamepad1.left_stick_x, robot.anglePID.getPower(), robot.odometry.getRobotAngle());
            robot.updateRobot(false, false, false);
        }
    }
}
=======
package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    @TeleOp
    public class CompetitionTeleop extends BaseTeleOp {

        // Teleop Logic

        // the angle the robot should be facing
        double targetAngle = 0;

        // stores if we are currently inputting a turn on controller 1
        boolean gp1xJoy = false;
        boolean gp2tri = false;
        
        // The main code that runs during init
        @Override
        public void runOpMode() throws InterruptedException {

            // initializes the robot without resetting the odometry
            initialize(false, false);


            waitForStart();

            while(opModeIsActive()) {

                // get and update functions
                robot.updateLoopTime();
                robot.odometry.updateOdometry();

                // resets Field Centric Driving
                if (gamepad1.share) {
                    robot.IMUReset();
                }
                        robot.setTurnPIntake(false);
                    }

                        }
                    }
>>>>>>> Stashed changes
