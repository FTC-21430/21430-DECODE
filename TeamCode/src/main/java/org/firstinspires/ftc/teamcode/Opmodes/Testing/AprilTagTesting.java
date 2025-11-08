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
        aprilTag.init(hardwareMap, telemetry);
    }
    @Override
    public void loop(){
        // April tags 20-22 are for the Obleisk
        aprilTag.update();
        AprilTagDetection id20 = aprilTag.getSpecific(20);
        aprilTag.displayDetectionTelemetry(id20);

        aprilTag.update();
        AprilTagDetection id21 = aprilTag.getSpecific(21);
        aprilTag.displayDetectionTelemetry(id21);

        aprilTag.update();
        AprilTagDetection id22 = aprilTag.getSpecific(22);
        aprilTag.displayDetectionTelemetry(id22);


    }
}
