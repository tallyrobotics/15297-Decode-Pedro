package org.firstinspires.ftc.teamcode.legacy;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.legacy.subsystems.backLED;
import org.firstinspires.ftc.teamcode.legacy.subsystems.backLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyLeftShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyRightShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.frontLED;
import org.firstinspires.ftc.teamcode.legacy.subsystems.frontLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.intake;
import org.firstinspires.ftc.teamcode.legacy.subsystems.middleLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.middleLED;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.extensions.pedro.PedroComponent;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Decode Teleop")
public class Decode_Teleop extends NextFTCOpMode {

    public Decode_Teleop()
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(intake.INSTANCE),
                new SubsystemComponent(flyRightShooter.INSTANCE),
                new SubsystemComponent(flyLeftShooter.INSTANCE),
                new SubsystemComponent(backLauncher.INSTANCE),
                new SubsystemComponent(middleLauncher.INSTANCE),
                new SubsystemComponent(frontLauncher.INSTANCE),
                new SubsystemComponent(frontLED.INSTANCE),
                new SubsystemComponent(middleLED.INSTANCE),
                new SubsystemComponent(backLED.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public String leftFrontName = "leftFront";
    public String leftRearName = "leftRear";
    public String rightFrontName = "rightFront";
    public String rightRearName = "rightRear";

    public MotorEx leftFrontMotor;
    public MotorEx leftRearMotor;
    public MotorEx rightFrontMotor;
    public MotorEx rightRearMotor;

    private ColorSensor colorFront;
    private ColorSensor colorMid;
    private ColorSensor colorBack;
    private LED ledMidRed;
    private LED ledMidGreen;
    private LED ledBackRed;
    private LED ledBackGreen;
    private LED ledFrontRed;
    private LED ledFrontGreen;
    private DistanceSensor colorFront_DistanceSensor;
    private DistanceSensor colorMid_DistanceSensor;
    private DistanceSensor colorBack_DistanceSensor;
    int gain;
    String x;

//    public DcMotorEx flyLeft;
//    public IMUEx imu = new IMUEx("imu", Direction.BACKWARD,Direction.UP);

    private final Double turbo = 1.0;
    private final Double normal = 0.75;
    private final Double turtle = 0.3;
    private Double speed;
    private double shootRPM = 0.0;

    public MecanumDriverControlled driverControlled;

    @Override
    public void onInit() {
        speed = normal;
        shootRPM = 1700;

        leftFrontMotor = new MotorEx(leftFrontName);
        leftRearMotor = new MotorEx(leftRearName);
        rightFrontMotor = new MotorEx(rightFrontName);
        rightRearMotor = new MotorEx(rightRearName);


        colorFront = hardwareMap.get(ColorSensor.class, "colorFront");
        colorMid = hardwareMap.get(ColorSensor.class, "colorMid");
        colorBack = hardwareMap.get(ColorSensor.class, "colorBack");
        ledMidRed = hardwareMap.get(LED.class, "ledMidRed");
        ledMidGreen = hardwareMap.get(LED.class, "ledMidGreen");
        ledBackRed = hardwareMap.get(LED.class, "ledBackRed");
        ledBackGreen = hardwareMap.get(LED.class, "ledBackGreen");
        ledFrontRed = hardwareMap.get(LED.class, "ledFrontRed");
        ledFrontGreen = hardwareMap.get(LED.class, "ledFrontGreen");
        colorFront_DistanceSensor = hardwareMap.get(DistanceSensor.class, "colorFront");
        colorMid_DistanceSensor = hardwareMap.get(DistanceSensor.class, "colorMid");
        colorBack_DistanceSensor = hardwareMap.get(DistanceSensor.class, "colorBack");
        gain = 10;
//        flyLeft = hardwareMap.get(DcMotorEx.class,"flyLeft");
////        flyLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        flyLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        flyLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PedroComponent.follower().setStartingPose(new Pose(0, 0, 0));
        PedroComponent.follower().update();
    }

    @Override
    public void onUpdate() {
        PedroComponent.follower().update();

        driverControlled.setScalar(speed);

        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("flyLeft RPM",flyLeftShooter.INSTANCE.flyMotor.getVelocity());
//        telemetry.addData("flyLeft Target RPM", flyLeftShooter.INSTANCE.controlSystem.getGoal().getVelocity());
        telemetry.addData("flyLeft Power",flyLeftShooter.INSTANCE.flyMotor.getPower());
        telemetry.addData("flyRight RPM",flyRightShooter.INSTANCE.flyMotor.getVelocity());
//        telemetry.addData("flyRight Target RPM", flyRightShooter.INSTANCE.controlSystem.getGoal().getVelocity());
        telemetry.addData("flyRight Power",flyRightShooter.INSTANCE.flyMotor.getPower());
        telemetry.addData("shootRPM", shootRPM);
        telemetry.addData("RightTPS", flyRightShooter.INSTANCE.getTargetRPM());//TPS());
        telemetry.addData("LeftTPS", flyLeftShooter.INSTANCE.getTargetRPM());//TPS());
        telemetry.addData("flyLDir", flyLeftShooter.INSTANCE.flyMotor.getDirection());
        telemetry.addData("flyRDir", flyRightShooter.INSTANCE.flyMotor.getDirection());

        telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.");
        telemetry.addLine(" ");
        telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value.");
        telemetry.addLine(" ");
        // Update the gain value if either of the A or B gamepad buttons is being held
        // A gain of less than 1 will make the values smaller, which is not helpful.
//        if (gamepad1.a) {
//            // Only increase the gain by a small amount, since this loop will occur multiple times per second.
//            gain = (int) (gain + 0.005);
//        } else if (gamepad1.b && gain > 1) {
//            gain = (int) (gain - 0.005);
//        }
//        telemetry.addData("Gain", gain);
        // Tell the sensor our desired gain value (normally you would do this during initialization, not during the loop)
        ((NormalizedColorSensor) colorFront).setGain(gain);
        // Tell the sensor our desired gain value (normally you would do this during initialization, not during the loop)
        ((NormalizedColorSensor) colorMid).setGain(gain);
        // Tell the sensor our desired gain value (normally you would do this during initialization, not during the loop)
        ((NormalizedColorSensor) colorBack).setGain(gain);
        if(getColor("F")!=""&&getColor("M")!=""&&getColor("B")!=""){
            intake.INSTANCE.IntakeOff().schedule();
        }
        else{
            intake.INSTANCE.IntakeIn().schedule();
        }
        telemetry.update();

    }

    @Override
    public void onStartButtonPressed() {


//        flyLeft.setVelocity(200);

        driverControlled = new MecanumDriverControlled(
                leftFrontMotor,
                rightFrontMotor,
                leftRearMotor,
                rightRearMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX());

        driverControlled.schedule();

        intake.INSTANCE.IntakeIn().schedule();

        Gamepads.gamepad1().leftBumper().whenTrue(new InstantCommand(() -> {speed=turtle;}));

        Gamepads.gamepad1().leftBumper().whenBecomesFalse(new InstantCommand(() -> {speed=normal;}));
        Gamepads.gamepad1().rightBumper().whenTrue(new InstantCommand(() -> {speed=turbo;}));
        Gamepads.gamepad1().rightBumper().whenBecomesFalse(new InstantCommand(() -> {speed=normal;}));

        Gamepads.gamepad2().leftBumper().whenBecomesTrue(intake.INSTANCE.IntakeOut());
        Gamepads.gamepad2().leftBumper().whenBecomesFalse(intake.INSTANCE.IntakeIn());

        Gamepads.gamepad2().y().whenBecomesTrue(intake.INSTANCE.IntakeOff());

        Gamepads.gamepad1().x().whenBecomesTrue(frontLauncher.INSTANCE.shootCycle());
        Gamepads.gamepad1().a().whenBecomesTrue(middleLauncher.INSTANCE.shootCycle());
        Gamepads.gamepad1().b().whenBecomesTrue(backLauncher.INSTANCE.shootCycle());


        Gamepads.gamepad2().rightTrigger().greaterThan(0.1).whenTrue(new InstantCommand(() -> {
                    shootRPM = -400.0;
            flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
            flyRightShooter.INSTANCE.flySetRPM(shootRPM);
                }
                )
        );


        Gamepads.gamepad1().y().whenBecomesTrue(
            new ParallelGroup(

                    new SequentialGroup(
                            new Delay(0.1),
                            frontLauncher.INSTANCE.shootCycle()
                    ),
                    new SequentialGroup(
                            new Delay(0.00),
                            middleLauncher.INSTANCE.shootCycle()
                    ),
                    new SequentialGroup(
                            new Delay(0.1),
                            backLauncher.INSTANCE.shootCycle()
                    )



            )
        );

        Gamepads.gamepad2().dpadDown().whenBecomesTrue(
                new InstantCommand(() -> {
                    shootRPM -= 100.0;
                    flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
                    flyRightShooter.INSTANCE.flySetRPM(shootRPM);
                }
                ));

        Gamepads.gamepad2().dpadRight().whenBecomesTrue(
                new InstantCommand(() -> {
                    shootRPM = 1800;
                    flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
                    flyRightShooter.INSTANCE.flySetRPM(shootRPM);
                }
                ));

        Gamepads.gamepad2().dpadLeft().whenBecomesTrue(
                new InstantCommand(() -> {
                    shootRPM = -5.0;
                    flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
                    flyRightShooter.INSTANCE.flySetRPM(shootRPM);
                }
                ));



        Gamepads.gamepad2().dpadUp().whenBecomesTrue(
            new InstantCommand(() -> {
                shootRPM += 100.0;
                flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
                flyRightShooter.INSTANCE.flySetRPM(shootRPM);
            }
        ));

        flyRightShooter.INSTANCE.flySetRPM(shootRPM).schedule();
        flyLeftShooter.INSTANCE.flySetRPM(shootRPM).schedule();


        intake.INSTANCE.IntakeIn().schedule();


    }

    private void midLed(String color) {
        if (color.equals("R")) {
            middleLED.INSTANCE.Purple().schedule();
        } else if (color.equals("G")) {
            middleLED.INSTANCE.Green().schedule();
        } else {
            middleLED.INSTANCE.Off().schedule();
        }
    }

    /**
     * Describe this function...
     */
    private void backLed(String color) {
        if (color.equals("R")) {
            backLED.INSTANCE.Purple().schedule();
        } else if (color.equals("G")) {
            backLED.INSTANCE.Green().schedule();
        } else {
            backLED.INSTANCE.Off().schedule();
        }
    }

    /**
     * Describe this function...
     */
    private void frontLed(String color) {
        if (color.equals("R")) {
            frontLED.INSTANCE.Purple().schedule();
        } else if (color.equals("G")) {
            frontLED.INSTANCE.Green().schedule();
        } else {
            frontLED.INSTANCE.Off().schedule();
        }
    }

    /**
     * Describe this function...
     */
    private String getColor(String location) {
        String gotColor;
        NormalizedRGBA myNormalizedColors;
        int myColor;
        float hue;
        float saturation;
        float value;
        double gotDistance;

        if (location.equals("F")) {
            // Save the color sensor data as a normalized color value. It's recommended
            // to use Normalized Colors over color sensor colors is because Normalized
            // Colors consistently gives values between 0 and 1, while the direct
            // Color Sensor colors are dependent on the specific sensor you're using.
            myNormalizedColors = ((NormalizedColorSensor) colorFront).getNormalizedColors();
            // Convert the normalized color values to an Android color value.
            myColor = myNormalizedColors.toColor();
            // Use the Android color value to calculate the Hue, Saturation and Value color variables.
            // See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html for an explanation of HSV color.
            hue = JavaUtil.colorToHue(myColor);
            saturation = JavaUtil.colorToSaturation(myColor);
            value = JavaUtil.colorToValue(myColor);
            // Use telemetry to display feedback on the driver station. We show the red,
            // green, and blue normalized values from the sensor (in the range of 0 to
            // 1), as well as the equivalent HSV (hue, saturation and value) values.
            telemetry.addLine("red" + location + " | " + JavaUtil.formatNumber(myNormalizedColors.red, 3) + "green" + location + " | " + JavaUtil.formatNumber(myNormalizedColors.green, 3) + "blue" + location + JavaUtil.formatNumber(myNormalizedColors.blue, 3));
            telemetry.addLine("hue" + location + " | " + JavaUtil.formatNumber(hue, 3) + "saturation" + location + " | " + JavaUtil.formatNumber(saturation, 3) + "value" + location + JavaUtil.formatNumber(value, 3));
            telemetry.addData("alpha" + location, Double.parseDouble(JavaUtil.formatNumber(myNormalizedColors.alpha, 3)));
            gotDistance = Double.parseDouble(JavaUtil.formatNumber(colorFront_DistanceSensor.getDistance(DistanceUnit.CM), 3));
            // If this color sensor also has a distance sensor, display the measured distance.
            // Note that the reported distance is only useful at very close
            // range, and is impacted by ambient light and surface reflectivity.
            telemetry.addData("distance (cm)" + location, gotDistance);
            if (gotDistance < 5) {
                if (myNormalizedColors.green >= myNormalizedColors.red && myNormalizedColors.green >= myNormalizedColors.blue) {
                    frontLed("G");
                    gotColor = "G";
                } else if (myNormalizedColors.blue >= myNormalizedColors.red && myNormalizedColors.blue >= myNormalizedColors.blue) {
                    frontLed("R");
                    gotColor = "R";
                } else {
                    frontLed("");
                    gotColor = "";
                }
            } else {
                frontLed("");
                gotColor = "";
            }
        } else if (location.equals("M")) {
            // Save the color sensor data as a normalized color value. It's recommended
            // to use Normalized Colors over color sensor colors is because Normalized
            // Colors consistently gives values between 0 and 1, while the direct
            // Color Sensor colors are dependent on the specific sensor you're using.
            myNormalizedColors = ((NormalizedColorSensor) colorMid).getNormalizedColors();
            // Convert the normalized color values to an Android color value.
            myColor = myNormalizedColors.toColor();
            // Use the Android color value to calculate the Hue, Saturation and Value color variables.
            // See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html for an explanation of HSV color.
            hue = JavaUtil.colorToHue(myColor);
            saturation = JavaUtil.colorToSaturation(myColor);
            value = JavaUtil.colorToValue(myColor);
            // Use telemetry to display feedback on the driver station. We show the red,
            // green, and blue normalized values from the sensor (in the range of 0 to
            // 1), as well as the equivalent HSV (hue, saturation and value) values.
            telemetry.addLine("red" + location + " | " + JavaUtil.formatNumber(myNormalizedColors.red, 3) + "green" + location + " | " + JavaUtil.formatNumber(myNormalizedColors.green, 3) + "blue" + location + JavaUtil.formatNumber(myNormalizedColors.blue, 3));
            telemetry.addLine("hue" + location + " | " + JavaUtil.formatNumber(hue, 3) + "saturation" + location + " | " + JavaUtil.formatNumber(saturation, 3) + "value" + location + JavaUtil.formatNumber(value, 3));
            telemetry.addData("alpha" + location, Double.parseDouble(JavaUtil.formatNumber(myNormalizedColors.alpha, 3)));
            gotDistance = Double.parseDouble(JavaUtil.formatNumber(colorMid_DistanceSensor.getDistance(DistanceUnit.CM), 3));
            // If this color sensor also has a distance sensor, display the measured distance.
            // Note that the reported distance is only useful at very close
            // range, and is impacted by ambient light and surface reflectivity.
            telemetry.addData("distance (cm)" + location, gotDistance);
            if (gotDistance < 5) {
                if (myNormalizedColors.green >= myNormalizedColors.red && myNormalizedColors.green >= myNormalizedColors.blue) {
                    midLed("G");
                    gotColor = "G";
                } else if (myNormalizedColors.blue >= myNormalizedColors.red && myNormalizedColors.blue >= myNormalizedColors.blue) {
                    midLed("R");
                    gotColor = "R";
                } else {
                    midLed("");
                    gotColor = "";
                }
            } else {
                midLed("");
                gotColor = "";
            }
        } else if (location.equals("B")) {
            // Save the color sensor data as a normalized color value. It's recommended
            // to use Normalized Colors over color sensor colors is because Normalized
            // Colors consistently gives values between 0 and 1, while the direct
            // Color Sensor colors are dependent on the specific sensor you're using.
            myNormalizedColors = ((NormalizedColorSensor) colorBack).getNormalizedColors();
            // Convert the normalized color values to an Android color value.
            myColor = myNormalizedColors.toColor();
            // Use the Android color value to calculate the Hue, Saturation and Value color variables.
            // See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html for an explanation of HSV color.
            hue = JavaUtil.colorToHue(myColor);
            saturation = JavaUtil.colorToSaturation(myColor);
            value = JavaUtil.colorToValue(myColor);
            // Use telemetry to display feedback on the driver station. We show the red,
            // green, and blue normalized values from the sensor (in the range of 0 to
            // 1), as well as the equivalent HSV (hue, saturation and value) values.
            telemetry.addLine("red" + location + " | " + JavaUtil.formatNumber(myNormalizedColors.red, 3) + "green" + location + " | " + JavaUtil.formatNumber(myNormalizedColors.green, 3) + "blue" + location + JavaUtil.formatNumber(myNormalizedColors.blue, 3));
            telemetry.addLine("hue" + location + " | " + JavaUtil.formatNumber(hue, 3) + "saturation" + location + " | " + JavaUtil.formatNumber(saturation, 3) + "value" + location + JavaUtil.formatNumber(value, 3));
            telemetry.addData("alpha" + location, Double.parseDouble(JavaUtil.formatNumber(myNormalizedColors.alpha, 3)));
            gotDistance = Double.parseDouble(JavaUtil.formatNumber(colorBack_DistanceSensor.getDistance(DistanceUnit.CM), 3));
            // If this color sensor also has a distance sensor, display the measured distance.
            // Note that the reported distance is only useful at very close
            // range, and is impacted by ambient light and surface reflectivity.
            telemetry.addData("distance (cm)" + location, gotDistance);
            if (gotDistance < 5) {
                if (myNormalizedColors.green >= myNormalizedColors.red && myNormalizedColors.green >= myNormalizedColors.blue) {
                    backLed("G");
                    gotColor = "G";
                } else if (myNormalizedColors.blue >= myNormalizedColors.red && myNormalizedColors.blue >= myNormalizedColors.blue) {
                    backLed("R");
                    gotColor = "R";
                } else {
                    backLed("");
                    gotColor = "";
                }
            } else {
                backLed("");
                gotColor = "";
            }
        } else {
            gotColor = "";
        }
        return gotColor;
    }


    @Override
    public void onStop() {
    }
}
