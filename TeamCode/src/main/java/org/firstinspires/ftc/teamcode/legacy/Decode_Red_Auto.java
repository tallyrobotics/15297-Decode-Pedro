package org.firstinspires.ftc.teamcode.legacy;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.legacy.subsystems.backLED;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyLeftShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyRightShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.frontLED;
import org.firstinspires.ftc.teamcode.legacy.subsystems.intake;
import org.firstinspires.ftc.teamcode.legacy.subsystems.back;
import org.firstinspires.ftc.teamcode.legacy.subsystems.front;
import org.firstinspires.ftc.teamcode.legacy.subsystems.middle;
import org.firstinspires.ftc.teamcode.legacy.subsystems.middleLED;
import org.firstinspires.ftc.teamcode.legacy.subsystems.rpmLED;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.ServoEx;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;



@Autonomous(name = "Decode Red Auto")
public class Decode_Red_Auto extends NextFTCOpMode {
    public Decode_Red_Auto(){
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(intake.INSTANCE),
                new SubsystemComponent(flyRightShooter.INSTANCE),
                new SubsystemComponent(flyLeftShooter.INSTANCE),
                new SubsystemComponent(back.INSTANCE),
                new SubsystemComponent(middle.INSTANCE),
                new SubsystemComponent(front.INSTANCE),
                new SubsystemComponent(frontLED.INSTANCE),
                new SubsystemComponent(middleLED.INSTANCE),
                new SubsystemComponent(backLED.INSTANCE),
                new SubsystemComponent(rpmLED.INSTANCE)
        );
    }

    public ServoEx bT;
    public ServoEx fO;
    public String frontTwoName = "frontTwo";
    public String backOneName = "backOne";

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
    int aprilValue;



//    private Follower follower;

    private final Pose startPose = new Pose(120.9, 127.7, Math.toRadians(308.0));
    private final Pose shootPose = new Pose(111.5, 118, Math.toRadians(308.0));

    private final int shootRPM = 2000; //2000 without


    boolean USE_WEBCAM;
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal myVisionPortal;


    private PathChain line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11;

    public void buildPaths() {

               line1 = follower().pathBuilder()
                       .addPath(
                               new BezierLine(startPose, shootPose)
                       )
                       .setConstantHeadingInterpolation(Math.toRadians(308.0))
                       .build();

        line2 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPose, new Pose(100.000, 85.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(308.0), Math.toRadians(-4))
                .build();

        line3 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(100.000, 85.5), new Pose(125.000, 85.5))
                )
                .setConstantHeadingInterpolation(Math.toRadians(-4))
                .build();

        line4 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.000, 85.5), shootPose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(-4), Math.toRadians(298.0))
                .build();

        line5 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPose, new Pose(95.000, 61.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(298.0), Math.toRadians(0))
                .build();

        line6 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(95.000, 61.000), new Pose(130.000, 61.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line7 = follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(130.000, 61.000),
                                new Pose(102.216, 57.130),
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(295.0))
                .build();

        line8 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(111, 118), new Pose(98.000, 38.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(298.0), Math.toRadians(0))
                .build();

        line9 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(98.000, 38.000), new Pose(130.000, 38.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line10 = follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(130.000, 38.000),
                                new Pose(104.720, 33.222),
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(308.0))
                .build();

        line11 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPose, new Pose(123.000, 97.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(295.0), Math.toRadians(225))
                .build();

    }

    public Command doAuto() {
        return new SequentialGroup(
                new ParallelGroup(
                        intake.INSTANCE.IntakeIn(),
                        flyLeftShooter.INSTANCE.flySetRPM(shootRPM),
                        flyRightShooter.INSTANCE.flySetRPM(shootRPM),
                        new FollowPath(line1, true, 0.6)
                ),
                new Delay(0.25),
                new ParallelGroup(
                        ShootPattern(),


                        new SequentialGroup(
                                new Delay(2.0),
                                new FollowPath(line2, true, 0.6)
                ),
                new Delay(0.25),
                new FollowPath(line3, true, 0.25),
                new Delay(0.25),
                new FollowPath(line4, true, 0.6),
                new Delay(0.25),
                new ParallelGroup(
                        ShootPattern(),


                        new SequentialGroup(
                                new Delay(2.0),
                                new FollowPath(line5, true, 0.6)
                        )
                ),
                new Delay(0.25),
                new FollowPath(line6, true, 0.25),
                new Delay(0.25),
                new FollowPath(line7, true, 0.5),
                new Delay(1.0),
                new ParallelGroup(
                        ShootPattern(),


                        new SequentialGroup(
                                new Delay(2.0),
                                new FollowPath(line11, true, 0.6)
                        )
                )

//                ),
//                new Delay(0.25),
//                new FollowPath(line9, true, 0.3),
//                new Delay(0.25),
//                new FollowPath(line10, true, 0.85),
//                new Delay(0.25),
//                new ParallelGroup(
//                        backTwo.INSTANCE.shootCycle(),
//                        frontOne.INSTANCE.shootCycle()
//                        ,
//
//                        new SequentialGroup(
//                                new Delay(1.5),
//                                new FollowPath(line11, true, 1.0)
//                        )
                )

        );


    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void onUpdate() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower().update();

        // Feedback to Driver Hub for debugging

        telemetry.addData("x", follower().getPose().getX());
        telemetry.addData("y", follower().getPose().getY());
        telemetry.addData("heading", follower().getPose().getHeading());
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

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void onInit() {
        bT = new ServoEx(frontTwoName);
        bT.getServo().setDirection(Servo.Direction.FORWARD);
        fO = new ServoEx(backOneName);
        fO.getServo().setDirection(Servo.Direction.REVERSE);

        bT.setPosition(0.05);
        fO.setPosition(0.05);



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


//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
        follower().setStartingPose(startPose);
        buildPaths();

        new InstantCommand(() -> {
            new SequentialGroup(
                    new ParallelGroup(
                            back.INSTANCE.up(),
                            front.INSTANCE.up()),
                    new Delay(1),
                    new ParallelGroup(
                            back.INSTANCE.down(),
                            front.INSTANCE.down()
                    )
            );
            }).schedule();


        USE_WEBCAM = true;
        // Initialize AprilTag before waitForStart.
        initAprilTag();
        // Wait for the match to begin.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void onWaitForStart() {
//        telemetryAprilTag();
//        // Push telemetry to the Driver Station.
//        telemetry.update();
        if(telemetryAprilTag()>=21&&telemetryAprilTag()<=23){
            aprilValue = telemetryAprilTag();
        }
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void onStartButtonPressed() {

        bT.setPosition(0.0);
        fO.setPosition(0.0);

        doAuto().schedule();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void onStop() {

    }

    /**
     * Initialize AprilTag Detection.
     */
    private void initAprilTag() {
        // First, create an AprilTagProcessor.
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        // Next, create a VisionPortal.
        if (USE_WEBCAM) {
            myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);
        } else {
            myVisionPortal = VisionPortal.easyCreateWithDefaults(BuiltinCameraDirection.BACK, myAprilTagProcessor);
        }
    }

    /**
     * Display info (using telemetry) for a recognized AprilTag.
     */
    private int telemetryAprilTag() {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;
        int returnValue = 0;

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
        // Iterate through list and call a function to display info for each recognized AprilTag.
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            // Display info about the detection.
            telemetry.addLine("");
            if (myAprilTagDetection.metadata != null) {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
                telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
                telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
                returnValue = myAprilTagDetection.id;
            } else {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
                telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
            }
        }
        telemetry.addLine("");
        telemetry.addLine("key:");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        return returnValue;

    }







    private void midLed(String color) {
        if (color.equals("R")) {
            middleLED.INSTANCE.Purple().schedule();
        } else if (color.equals("G")) {
            middleLED.INSTANCE.Green();
        } else {
            middleLED.INSTANCE.Off();
        }
    }

    /**
     * Describe this function...
     */
    private void backLed(String color) {
        if (color.equals("R")) {
            ledBackRed.on();
            ledBackGreen.off();
        } else if (color.equals("G")) {
            ledBackGreen.on();
            ledBackRed.off();
        } else {
            ledBackRed.off();
            ledBackGreen.off();
        }
    }

    /**
     * Describe this function...
     */
    private void frontLed(String color) {
        if (color.equals("R")) {
            ledFrontRed.on();
            ledFrontGreen.off();
        } else if (color.equals("G")) {
            ledFrontGreen.on();
            ledFrontRed.off();
        } else {
            ledFrontRed.off();
            ledFrontGreen.off();
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

    private Command ShootPattern(){
        String order = "";
        if(aprilValue==21){
            if(getColor("F")=="G"){
                order = "FMB";
            }
            else if (getColor("M")=="G"){
                order = "MBF";
            }
            else if(getColor("B")=="G"){
                order = "BFM";
            }
        }
        else if(aprilValue == 22){
            if(getColor("F")=="G"){
                order = "BFM";
            }
            else if (getColor("M")=="G"){
                order = "FMB";
            }
            else if(getColor("B")=="G"){
                order = "MBF";
            }
        }
        else if(aprilValue == 23){
            if(getColor("F")=="G"){
                order = "MBF";
            }
            else if (getColor("M")=="G"){
                order = "BFM";
            }
            else if(getColor("B")=="G"){
                order = "FMB";
            }
        }
        if(order=="FMB"){
            new SequentialGroup(
                    front.INSTANCE.shootCycle(),
                    new Delay(0.5),
                    middle.INSTANCE.shootCycle(),
                    new Delay(0.5),
                    back.INSTANCE.shootCycle()
            );
        }
        else if(order=="MBF"){
            new SequentialGroup(
                    middle.INSTANCE.shootCycle(),
                    new Delay(0.5),
                    back.INSTANCE.shootCycle(),
                    new Delay(0.5),
                    front.INSTANCE.shootCycle()
            );
        }
        else if(order=="BFM"){
            new SequentialGroup(
                    back.INSTANCE.shootCycle(),
                    new Delay(0.5),
                    front.INSTANCE.shootCycle(),
                    new Delay(0.5),
                    middle.INSTANCE.shootCycle()
            );
        }
        return new SequentialGroup();
    }

}
