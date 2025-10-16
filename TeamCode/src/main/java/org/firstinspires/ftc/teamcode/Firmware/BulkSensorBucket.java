package org.firstinspires.ftc.teamcode.Firmware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
// This class is an object to be used by Robot.Java to speed up loop times by using bulk sensor reads
// By sending the very first sensor read through android as a read for everything on that hub, any future calls to that hub do not need to wait in androids event queue.
// DOES NOT INCLUDE I2C Sensor Calls!
public class BulkSensorBucket {
//    A list we store both the control and expansion hubs within. Will be assigned during class construct.
//    LynxModule is referring to the exact circuit board that is found within the Rev Hubs.
    List<LynxModule> allHubs = null;

    public BulkSensorBucket(HardwareMap hardwareMap){
//        Get and save the references to the hubs from hardware map. This is the only time we need to do this so we do not store a reference to hardware map
        allHubs = hardwareMap.getAll(LynxModule.class);

//        Iterate through both hubs to set their internal modes to manual bulk read
        for (LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    /**
     * Allows us to disable or re-enable the bulk reading if problems arise
     * @param enabled will be set to manual if true, off if false
     */
    public void bulkReadingEnabled(boolean enabled){
        if (enabled){
//            Similar to init, we iterate through both the control and expansion hubs to set them to the desired value
            for (LynxModule hub : allHubs){
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        } else {
//            See comment on Line 32
            for (LynxModule hub : allHubs){
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
            }
        }
    }

    /**
     * At the end of every main loop iteration, this function must be called!
     * If you do not clear the bulk cache before the next iteration, all sensor values will be stale.
     * Stale means that it will still be what it was last iteration, and not representative of what it is during this iteration.
     * Cache means a stored set of data that is easier to access than what it is normally It is like programming short hand.
     */
    public void clearCache(){
        for (LynxModule hub : allHubs){
            hub.clearBulkCache();
        }
    }
}
