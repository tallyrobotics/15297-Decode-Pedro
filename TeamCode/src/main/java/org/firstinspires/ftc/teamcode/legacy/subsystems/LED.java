package org.firstinspires.ftc.teamcode.legacy.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public abstract class LED implements Subsystem {

    public LED(String ledName, String colorSenseName, String distSenseName) {
        LEDname = ledName;
        colSenName = colorSenseName;
        distanceName = distSenseName;
    }

    public ServoEx led;
    public ColorSensor colSensor;
    public DistanceSensor colorDistanceSensor;
    public String colSenName;
    public String distanceName;
    public String LEDname;
    private static final Double off = 0.0;
    private static final Double red = 0.285;
    private static final Double yellow = 0.41;
    private static final Double green = 0.46;
    private static final Double blue = 0.60;
    private static final Double purple = 0.67;

    private String color = "off";
    private double distance = -1.0;

    public String getColor()
    {
        return color;
    }

    public double getDistance()
    {
        return distance;
    }

    public Command Off() {
        color = "off";
        return new SetPosition(led, off);
    }

    public Command Red() {
        color = "red";
        return new SetPosition(led, red);
    }

    public Command Yellow() {
        color = "yellow";
        return new SetPosition(led, yellow);
    }

    public Command Blue() {
        color = "blue";
        return new SetPosition(led, blue);
    }

    public Command Green() {
        color = "green";
        return new SetPosition(led, green);
    }

    public Command Purple() {
        color = "purple";
        return new SetPosition(led, purple);
    }

    @Override
    public void initialize()
    {
        led = new ServoEx(LEDname);
        if (!"".equals(colSenName))
        {
            colSensor = ActiveOpMode.hardwareMap().get(ColorSensor.class, colSenName);
        }
        if (!"".equals(distanceName))
        {
            colorDistanceSensor = ActiveOpMode.hardwareMap().get(DistanceSensor.class, distanceName);
        }
    }

    @Override
    public void periodic()
    {
        if (!"".equals(colSenName)) {
            NormalizedRGBA myNormalizedColors;
            int myColor;
//        float hue;
//        float saturation;
//        float value;

            // Save the color sensor data as a normalized color value. It's recommended
            // to use Normalized Colors over color sensor colors is because Normalized
            // Colors consistently gives values between 0 and 1, while the direct
            // Color Sensor colors are dependent on the specific sensor you're using.
            myNormalizedColors = ((NormalizedColorSensor) colSensor).getNormalizedColors();
            // Convert the normalized color values to an Android color value.
            myColor = myNormalizedColors.toColor();
            // Use the Android color value to calculate the Hue, Saturation and Value color variables.
            // See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html for an explanation of HSV color.
//        hue = JavaUtil.colorToHue(myColor);
//        saturation = JavaUtil.colorToSaturation(myColor);
//        value = JavaUtil.colorToValue(myColor);
            // Use telemetry to display feedback on the driver station. We show the red,
            // green, and blue normalized values from the sensor (in the range of 0 to
            // 1), as well as the equivalent HSV (hue, saturation and value) values.
            distance = Double.parseDouble(JavaUtil.formatNumber(colorDistanceSensor.getDistance(DistanceUnit.CM), 1));
            // If this color sensor also has a distance sensor, display the measured distance.
            // Note that the reported distance is only useful at very close
            // range, and is impacted by ambient light and surface reflectivity.
            ActiveOpMode.telemetry().addData(colSenName + " distance (cm)", distance);
            if (distance < 5.0) {
                if (myNormalizedColors.green >= myNormalizedColors.red && myNormalizedColors.green >= myNormalizedColors.blue) {
                    Green().schedule();
                } else if (myNormalizedColors.blue >= myNormalizedColors.red && myNormalizedColors.blue >= myNormalizedColors.green) {
                    Purple().schedule();
                } else {
                    Off().schedule();
                }
            } else {
                Off().schedule();
            }
        }
    }
}

