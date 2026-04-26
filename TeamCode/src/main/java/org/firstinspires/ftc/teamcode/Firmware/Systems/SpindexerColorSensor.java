package org.firstinspires.ftc.teamcode.Firmware.Systems;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import java.util.ArrayList;

public class SpindexerColorSensor {

//    ColorSensor class used in example op modes from the FIRST SDK
    private final NormalizedColorSensor sensor1;
    private final NormalizedColorSensor sensor2;

// constants for color ranges. At the moment the difference is calculated based on the Hue value in the HSV color space. One tuning set for each potential color we are searching for
    private final float[] noneValues = new float[]{
            105,
        (float)0.53,
        (float)0.023
    };
    private final float[] purpleValues = new float[]{
            160,
            (float)0.27,
            (float)0.0176
    };
    private final float[] greenValues = new float[]{
            138,
            (float) 0.9,
            (float)0.0137
    };

    // Enum return value for all colors we are detecting
    public enum COLORS{
        NONE,
        PURPLE,
        GREEN,
    }

//    Example initialization:
//    ColorSensorFirmware sensor = new ColorSensorFirmware(hardwareMap, "colorSensor1");
//    Type name = new type(HardwareMap, configuration name);

    public SpindexerColorSensor(HardwareMap hardwareMap, String configName1, String configName2){
        sensor1 = hardwareMap.get(NormalizedColorSensor.class, configName1);
        sensor2 = hardwareMap.get(NormalizedColorSensor.class, configName2);

//        constant value for gain of the sensor. for gain: higher the number, higher the values / the difference between different colors.
        final float sensorGain = 10;
        sensor1.setGain(sensorGain);
        sensor2.setGain(sensorGain);
    }

    //            Logic for using the detected color value.
    //             switch(sensor.getDetectedColor()){
    //                 case NONE:
    //                     break;
    //                 case GREEN:
    //                     break;
    //                 case PURPLE:
    //                     break;
    //           }

    /**
     * reads sensor values, identifies the difference between the colors we are searching for and the current readings.
     * @return Color ENUM of the color that is the closest match to readings.
     */
    public COLORS getDetectedColor(){
//        Read color values and get the distances between configuration values and raw readings
        float[] values = getRawData();

        if (values[0] <= 134){
            return COLORS.NONE;
        }
        if (values[0] >= 157){
           return COLORS.PURPLE;
        }else{
            if (values[1] > 0.42 && values[1] < 0.6){
                return COLORS.GREEN;
            }else if(values[1] > 0.32){
                return COLORS.PURPLE;
            }else {
                return COLORS.NONE;
            }
        }

    }

    /**
     * Setter for the sensors gain, must be within values of 0-15
     * @param gain the amount the sensor takes in: higher the value, larger the difference between different colors
     */
    public void setGain(float gain){
        if (gain < 0 || gain > 15){
            return;
        }
        sensor1.setGain(gain);
        sensor2.setGain(gain);
    }

    /**
     * gets value from the sensor and converts from RGBA to HSV
     * @return returns color in HSV
     */
    public float[] getRawData(){
        float[] hsvValues1 = new float[3];
        float[] hsvValues2 = new float[3];
        NormalizedRGBA colors1 = sensor1.getNormalizedColors();
        NormalizedRGBA colors2 = sensor2.getNormalizedColors();

        Color.colorToHSV(colors1.toColor(), hsvValues1);
        Color.colorToHSV((colors2.toColor()), hsvValues2);
        // if Color sensor has an ESD event, it will start returning only 0.0. If this is the case, simple only count on the other one! (lets pray that both don't go out.)
        if (hsvValues1[0] == 0.0) hsvValues1 = hsvValues2;
        if (hsvValues2[0] == 0.0) hsvValues2 = hsvValues1;

        float[] averageHsvValues = new float[3];
        for (int i = 0; i < 3; i++){
            // get the average of both channels
            averageHsvValues[i] = ((hsvValues1[i]+hsvValues2[i]) / 2);
        }

        // If it gets so bad where both sensors are out, than just return None and pray that the switch gets it!.
        if (averageHsvValues[0] == 0.0) averageHsvValues[0] = 50;

        return averageHsvValues;
    }

    /**
     * gets the raw data and compares it to the calibrated constants at the top of this file.
     * @return all three color errors in order, None, Purple, Green
     */
    private ArrayList<Double> getColorDistances(){
        ArrayList<Double> returnValues = new ArrayList<>();
        float[] hsvValues = getRawData();

        double noneDistance = hsvValues[0] - noneValues[0] + ((hsvValues[1] - noneValues[1]) * 2000) + ((hsvValues[2]-noneValues[2])*1);
        returnValues.add(noneDistance);

        double purpleDistance = hsvValues[0] - purpleValues[0] + ((hsvValues[1] - purpleValues[1]) * 2000) + ((hsvValues[2]-purpleValues[2])*1);;
        returnValues.add(purpleDistance);

        double greenDistance = hsvValues[0] - greenValues[0] + ((hsvValues[1] - greenValues[1]) * 2000) + ((hsvValues[2]-greenValues[2])*1);;
        returnValues.add(greenDistance);

        return returnValues;
    }
}
