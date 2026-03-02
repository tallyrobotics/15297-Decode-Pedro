package org.firstinspires.ftc.teamcode.legacy;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.legacy.subsystems.backLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyLeftShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyRightShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.frontLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.launcher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.intakeLED;
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

import static dev.nextftc.extensions.pedro.PedroComponent.follower;



@Autonomous(name = "Decode Red Auto No Lever")
public class Decode_Red_Auto_No_Lever extends NextFTCOpMode {
    public Decode_Red_Auto_No_Lever(){
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(shootersLED.INSTANCE),
                new SubsystemComponent(frontLauncher.INSTANCE),
                new SubsystemComponent(middleLauncher.INSTANCE),
                new SubsystemComponent(backLauncher.INSTANCE),
                new SubsystemComponent(intakeLED.INSTANCE)
        );
    }

    Integer aprilValue = 0;
    String order;
    int counter=0;
    Boolean isShooting = false;



//    private Follower follower;

    private final Pose startPose = new Pose(124.900, 125.700, Math.toRadians(-51.34));
    private final Pose shootPose = new Pose(96.700, 95.300, Math.toRadians(-45));

    private final int shootRPM = 1770; //2000 without


    boolean USE_WEBCAM;
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal myVisionPortal;


    private PathChain line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11;

    public void buildPaths() {

        line1 = follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                startPose,
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-51.34), Math.toRadians(-45))
                .build();

        line2 = follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose,
                                new Pose(97.000, 84.150),
                                new Pose(120.000, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        line3 = follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.000, 84.000),
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(-45))
                .build();

        line4 = follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose,
                                new Pose(97.052, 56.494),
                                new Pose(102.002, 60.396),
                                new Pose(120.000, 60.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0.0))
                .build();

        line5 = follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.000, 60.000),
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0.0),Math.toRadians(-45))
                .build();

        line6 = follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                shootPose,
                                new Pose(97.330, 37.330),
                                new Pose(97.068, 35.527),
                                new Pose(105.871, 36.183),
                                new Pose(120.000, 36.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0.0))
                .build();

        line7 = follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.000, 36.000),
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0.0), Math.toRadians(-45))
                .build();

        line8 = follower().pathBuilder()
                .addPath(
                        new BezierLine(
                                shootPose,
                                new Pose(116.750, 72.500)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(270))
                .build();

    }

    public Command doAuto1() {
        return new SequentialGroup(
                new ParallelGroup(
                        //intake.INSTANCE.IntakeIn(),
                        flyLeftShooter.INSTANCE.flySetRPM(shootRPM),
                        flyRightShooter.INSTANCE.flySetRPM(shootRPM),
                        new FollowPath(line1, true, 0.75)
                ),
                new Delay(0.75),
                new ParallelGroup(
                        //ShootPattern(intakeLED.INSTANCE.frontColor(), intakeLED.INSTANCE.middleColor(),intakeLED.INSTANCE.backColor()),
                        new InstantCommand(()-> {isShooting=true;}),

                        new SequentialGroup(
                                new Delay(1.7),
                                new FollowPath(line2, true, 0.75)
                        )
                ),
                new Delay(0.25)



        );


    }
    public Command doAuto2() {
        return new SequentialGroup(
                new FollowPath(line3, true, 0.75),
                new Delay(0.6),
                new ParallelGroup(
//                        new SequentialGroup(
//                                new InstantCommand(()-> {intakeLED.INSTANCE.frontColor();}),
//                                new InstantCommand(()-> {intakeLED.INSTANCE.middleColor();}),
//                                new InstantCommand(()-> {intakeLED.INSTANCE.backColor();}),
                        //
                        // ShootPattern(intakeLED.INSTANCE.frontColor(), intakeLED.INSTANCE.middleColor(),intakeLED.INSTANCE.backColor()),
                        new InstantCommand(()-> {isShooting=true;}),
//                        ),



                        new SequentialGroup(
                                new Delay(1.7),
                                new FollowPath(line4, true, 0.75)
                        )
                )
        );


    }
    public Command doAuto3() {
        return new SequentialGroup(
                new Delay(0.25),
                new FollowPath(line5, true, 0.75),
                new Delay(0.6),
                new ParallelGroup(
//                        ShootPattern(intakeLED.INSTANCE.frontColor(), intakeLED.INSTANCE.middleColor(),intakeLED.INSTANCE.backColor()),
                        new InstantCommand(()-> {isShooting=true;}),

                        new SequentialGroup(
                                new Delay(1.7),
                                new FollowPath(line6, true, 0.75)
                        )
                ),
                new Delay(0.25),
                new FollowPath(line7, true, 0.75),
                new Delay(0.6),
                new ParallelGroup(
//                        new SequentialGroup(
//                                new InstantCommand(()-> {intakeLED.INSTANCE.frontColor();}),
//                                new InstantCommand(()-> {intakeLED.INSTANCE.middleColor();}),
//                                new InstantCommand(()-> {intakeLED.INSTANCE.backColor();}),
                        //
                        // ShootPattern(intakeLED.INSTANCE.frontColor(), intakeLED.INSTANCE.middleColor(),intakeLED.INSTANCE.backColor()),
                        new InstantCommand(()-> {isShooting=true;}),
//                        ),



                        new SequentialGroup(
                                new Delay(1.7),
                                new FollowPath(line8, true, 0.75)
                        )
                )


        );
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void onUpdate() {
        counter++;
        // These loop the movements of the robot, these must be called continuously in order to work
        follower().update();
        if(isShooting){
            isShooting = false;
            ShootPattern(intakeLED.INSTANCE.frontColor(), intakeLED.INSTANCE.middleColor(),intakeLED.INSTANCE.backColor()).schedule();

        }
        follower().update();

        // Feedback to Driver Hub for debugging

        telemetry.addData("x", follower().getPose().getX());
        telemetry.addData("y", follower().getPose().getY());
        telemetry.addData("heading", Math.toRadians(follower().getPose().getHeading()));
        follower().update();
        telemetry.addData("Order", order);
        telemetry.addData("Updates", counter);
        follower().update();
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




//        USE_WEBCAM = true;
        // Initialize AprilTag before waitForStart.
//        initAprilTag();
        // Wait for the match to begin.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void onWaitForStart() {
//        int av = telemetryAprilTag();
//        if(av>=21&&av<=23){
//            aprilValue = av;
//        }
//        else{
//            aprilValue = -1;
//        }
        telemetry.addData("AprilValue", aprilValue);
        telemetry.update();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void onStartButtonPressed() {

        follower().activateAllPIDFs();
        new SequentialGroup(
                doAuto1(),
                doAuto2(),
                doAuto3()
        ).schedule();

//        USE_WEBCAM = false;
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
                if(myAprilTagDetection.id>=21&&myAprilTagDetection.id<=23){
                    returnValue = myAprilTagDetection.id;
                }
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



    private Command ShootPattern(String fc, String mc, String bc){
        order = "";
        launcher launchOne;
        launcher launchTwo;
        launcher launchThree;
        if(aprilValue==21){
            if(Objects.equals(fc, "green")){
                order = "FMB";
            }
            else if (Objects.equals(mc, "green")){
                order = "MBF";
            }
            else if(Objects.equals(bc, "green")){
                order = "BFM";
            }
            else{
                order = "FMB";
            }
        }
        else if(aprilValue == 22){
            if(Objects.equals(fc, "green")){
                order = "BFM";
            }
            else if (Objects.equals(mc, "green")){
                order = "FMB";
            }
            else if(Objects.equals(bc, "green")){
                order = "MBF";
            }
            else{
                order = "FMB";
            }
        }
        else if(aprilValue == 23){
            if(Objects.equals(fc, "green")){
                order = "MBF";
            }
            else if (Objects.equals(mc, "green")){
                order = "BFM";
            }
            else if(Objects.equals(bc, "green")){
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

        order = "";

        return new ParallelGroup(
                launchOne.shootCycle(),
                new SequentialGroup(
                        new Delay(0.55),
                        new ParallelGroup(
                                launchTwo.shootCycle(),
                                new SequentialGroup(
                                        new Delay(0.55),
                                        launchThree.shootCycle()
                                )
                        )
                )
        );

//        return new SequentialGroup(new Delay (0)
//        );
    }

}