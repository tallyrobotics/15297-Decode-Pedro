package org.firstinspires.ftc.teamcode.legacy;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.legacy.subsystems.backLED;
import org.firstinspires.ftc.teamcode.legacy.subsystems.backLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyLeftShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyRightShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.frontLED;
import org.firstinspires.ftc.teamcode.legacy.subsystems.frontLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.intake;
import org.firstinspires.ftc.teamcode.legacy.subsystems.launcher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.intakeLED;
import org.firstinspires.ftc.teamcode.legacy.subsystems.middleLED;
import org.firstinspires.ftc.teamcode.legacy.subsystems.middleLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.shootersLED;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Objects;

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
                new SubsystemComponent(shootersLED.INSTANCE),
                new SubsystemComponent(frontLauncher.INSTANCE),
                new SubsystemComponent(middleLauncher.INSTANCE),
                new SubsystemComponent(backLauncher.INSTANCE),
                new SubsystemComponent(intakeLED.INSTANCE)
        );
    }

    int aprilValue;
    String order;



//    private Follower follower;

    private final Pose startPose = new Pose(120.643, 130, Math.toRadians(-51.0));
    private final Pose shootPose1 = new Pose(114.5, 123.6, Math.toRadians(-58.0));
    private final Pose shootPose2 = new Pose(111.5, 117.5, Math.toRadians(-61.0));
    private final Pose shootPose3 = new Pose(111.0, 112.5, Math.toRadians(-61.0));


    private final int shootRPM = 1700; //2000 without


    boolean USE_WEBCAM;
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal myVisionPortal;


    private PathChain line1, line2, line3, line4, line5, line6, line7, line8;

    public void buildPaths() {

               line1 = follower().pathBuilder()
                       .addPath(
                               new BezierLine(startPose, shootPose1)
                       )
                       .setConstantHeadingInterpolation(Math.toRadians(-51.0))
                       .build();

        line2 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPose1, new Pose(98.000, 83.750+6.5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-51.0), Math.toRadians(0.0))
                .build();

        line3 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(98.000, 83.750+6.5), new Pose(120.000, 83.750+6.5))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0.0))
                .build();

        line4 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(124.000, 83.750+6.5), shootPose2)
                )
                .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(-61.0))
                .build();

        line5 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPose2, new Pose(98.000, 59.750+5))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-61.0), Math.toRadians(0.0))
                .build();

        line6 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(98.000, 59.750+5), new Pose(120.000, 59.750+5))
                )
                .setConstantHeadingInterpolation(Math.toRadians(0.0))
                .build();

        line7 = follower()
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(124.000, 59.750+5),
                                new Pose(109.100, 55.600+5),
                                shootPose3
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(-61.0))
                .build();

        line8 = follower()
                .pathBuilder()
                .addPath(
                        new BezierLine(shootPose3, new Pose(124.300, 101.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(-61.0), Math.toRadians(-144.0))
                .build();

    }

    public Command doAuto() {
        return new SequentialGroup(
                new ParallelGroup(
                        intake.INSTANCE.IntakeIn(),
                        flyLeftShooter.INSTANCE.flySetRPM(shootRPM),
                        flyRightShooter.INSTANCE.flySetRPM(shootRPM),
                        new FollowPath(line1, true, 0.7)
                ),
                new Delay(0.75),
                new ParallelGroup(
                        ShootPattern(),


                        new SequentialGroup(
                                new Delay(3.0),
                                new FollowPath(line2, true, 0.7)
                                )
                        ),
                new Delay(0.25),
                new FollowPath(line3, true, 0.3),
                new Delay(0.25),
                new FollowPath(line4, true, 0.7),
                new Delay(0.25),
                new ParallelGroup(
                        ShootPattern(),


                        new SequentialGroup(
                                new Delay(3.0),
                                new FollowPath(line5, true, 0.7)
                        )
                ),
                new Delay(0.25),
                new FollowPath(line6, true, 0.3),
                new Delay(0.25),
                new FollowPath(line7, true, 0.7),
                new Delay(1.0),
                new ParallelGroup(
                        ShootPattern(),


                        new SequentialGroup(
                                new Delay(3.0),
                                new FollowPath(line8, true, 1.0)
                        )
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
        telemetry.addData("heading", Math.toRadians(follower().getPose().getHeading()));

        telemetry.addData("Order", order);

        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void onInit() {
//        b = new ServoEx(backName);
//        b.getServo().setDirection(Servo.Direction.FORWARD);
//        f = new ServoEx(frontName);
//        f.getServo().setDirection(Servo.Direction.FORWARD);
//        m = new ServoEx(midName);
//        m.getServo().setDirection(Servo.Direction.REVERSE);
//
//        b.setPosition(0.05);
//        f.setPosition(0.05);
//        m.setPosition(0.05);


//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
        follower().setStartingPose(startPose);
        buildPaths();




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
        int av = telemetryAprilTag();
        if(av>=21&&av<=23){
            aprilValue = av;
        }
        else{
            aprilValue = -1;
        }
        telemetry.addData("AprilValue", aprilValue);
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void onStartButtonPressed() {

        USE_WEBCAM = false;

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



    private Command ShootPattern(){
        order = "";
        String colorF = frontLED.INSTANCE.getColor();
        String colorM = middleLED.INSTANCE.getColor();
        String colorB = backLED.INSTANCE.getColor();
        launcher launchOne;
        launcher launchTwo;
        launcher launchThree;
        if(aprilValue==21){
            if(Objects.equals(colorF, "green")){
                order = "FMB";
            }
            else if (Objects.equals(colorM, "green")){
                order = "MBF";
            }
            else if(Objects.equals(colorB, "green")){
                order = "BFM";
            }
            else{
                order = "FMB";
            }
        }
        else if(aprilValue == 22){
            if(Objects.equals(colorF, "green")){
                order = "BFM";
            }
            else if (Objects.equals(colorM, "green")){
                order = "FMB";
            }
            else if(Objects.equals(colorB, "green")){
                order = "MBF";
            }
            else{
                order = "FMB";
            }
        }
        else if(aprilValue == 23){
            if(Objects.equals(colorF, "green")){
                order = "MBF";
            }
            else if (Objects.equals(colorM, "green")){
                order = "BFM";
            }
            else if(Objects.equals(colorB, "green")){
                order = "FMB";
            }
            else{
                order = "FMB";
            }
        }
        else{
            order = "FMB";
        }

        if(order =="MBF") {
            launchOne = middleLauncher.INSTANCE;
            launchTwo = backLauncher.INSTANCE;
            launchThree = frontLauncher.INSTANCE;
        }
        else if(order =="BFM") {
            launchOne = backLauncher.INSTANCE;
            launchTwo = frontLauncher.INSTANCE;
            launchThree = middleLauncher.INSTANCE;
        }
        else {
            launchOne = frontLauncher.INSTANCE;
            launchTwo = middleLauncher.INSTANCE;
            launchThree = backLauncher.INSTANCE;
        }

        return new ParallelGroup(
                    launchOne.shootCycle(),
                    new SequentialGroup(
                            new Delay(1.0),
                            new ParallelGroup(
                                    launchTwo.shootCycle(),
                                    new SequentialGroup(
                                            new Delay(1.0),
                                            launchThree.shootCycle()
                                    )
                            )
                    )
            );

//        return new SequentialGroup(new Delay (0)
//        );
    }

}
