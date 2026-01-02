package org.firstinspires.ftc.teamcode.Opmodes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Firmware.Systems.AprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class AprilTagTesting extends OpMode {

    AprilTag aprilTag = new AprilTag();

    @Override
    public void init(){
        aprilTag.init(hardwareMap, telemetry,10);
    }

    //Do we need this function
    @Override
    public void loop(){

telemetry.addData("MotifId",aprilTag.getMotifID());
/*telemetry.addData("DistanceFromMotifID",aprilTag.getDistance("obelisk"));
telemetry.addData("BearingToTag",aprilTag.getBearingToTag("obelisk"));
telemetry.update();
 */
    }

}