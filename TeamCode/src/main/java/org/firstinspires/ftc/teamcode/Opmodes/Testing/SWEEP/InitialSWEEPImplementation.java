package org.firstinspires.ftc.teamcode.Opmodes.Testing.SWEEP;

import org.firstinspires.ftc.teamcode.Opmodes.BaseAuto;

public class InitialSWEEPImplementation extends BaseAuto {

    private void defineRoute(){

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(true,true,true);
        defineRoute();
        robot.SWEEP.computeSplines();
        waitForStart();
    }
}
