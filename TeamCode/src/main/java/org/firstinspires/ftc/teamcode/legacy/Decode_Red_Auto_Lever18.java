package org.firstinspires.ftc.teamcode.legacy;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.legacy.subsystems.backLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyLeftShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyRightShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.frontLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.intakeLED;
import org.firstinspires.ftc.teamcode.legacy.subsystems.launcher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.limeLight;
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


@Autonomous(name = "Decode Red Auto Lever 18 Ball")
public class Decode_Red_Auto_Lever18 extends NextFTCOpMode {
    public Decode_Red_Auto_Lever18(){
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(shootersLED.INSTANCE),
                new SubsystemComponent(frontLauncher.INSTANCE),
                new SubsystemComponent(middleLauncher.INSTANCE),
                new SubsystemComponent(backLauncher.INSTANCE),
                new SubsystemComponent(intakeLED.INSTANCE),
                new SubsystemComponent(limeLight.INSTANCE)
        );
    }

    Integer aprilValue = 0;
    String order;
    int counter=0;
    Boolean isShooting = false;



//    private Follower follower;

    private final Pose startPose = new Pose(124.9, 125.7, Math.toRadians(-51.34));
    private final Pose shootPose1 = new Pose(99.000, 85.000, Math.toRadians(-42));


    private final int shootRPM = 1860;


    boolean USE_WEBCAM;
    AprilTagProcessor myAprilTagProcessor;
    VisionPortal myVisionPortal;


    private PathChain line1, line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12;

    public void buildPaths() {

        line1 = follower().pathBuilder().addPath(
                        new BezierLine(
                                startPose,

                                shootPose1
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-51.34), Math.toRadians(-42))

                .build();

        line2 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                shootPose1,
                                new Pose(95.573, 58.511),
                                new Pose(100.119, 59.995),
                                new Pose(120.000, 60.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        line3 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(120.000, 60.000),

                                shootPose1
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-42))

                .build();

        line4= follower().pathBuilder().addPath(
                        new BezierCurve(
                                shootPose1,
                                new Pose(110.000, 60.000),
                                new Pose(130.500, 61.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-42), Math.toRadians(25))

                .build();

        line5= follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(130.500, 61.000),
                                new Pose(110.000, 60.000),
                                shootPose1
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(-42))

                .build();

        line6 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                shootPose1,
                                new Pose(110.000, 60.000),
                                new Pose(130.500, 61.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-42), Math.toRadians(25))

                .build();

        line7 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(130.500, 61.000),
                                new Pose(110.000, 60.000),
                                shootPose1
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(-42))

                .build();

        line8 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                shootPose1,
                                new Pose(110.000, 60.000),
                                new Pose(130.500, 61.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-42), Math.toRadians(25))

                .build();

        line9 = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(130.500, 61.000),
                                new Pose(110.000, 60.000),
                                shootPose1
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(-42))

                .build();

        line10 = follower().pathBuilder().addPath(
                        new BezierLine(
                                shootPose1,

                                new Pose(120.000, 84.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        line11 = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(120.000, 84.000),

                                shootPose1
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-42))

                .build();

        line12 = follower().pathBuilder().addPath(
                        new BezierLine(
                                shootPose1,

                                new Pose(120.000, 72.000)
                        )
                ).setConstantHeadingInterpolation(-90)

                .build();






    }

    public Command doAuto1() {
        return new SequentialGroup(
                new ParallelGroup(
                        //intake.INSTANCE.IntakeIn(),
                        flyLeftShooter.INSTANCE.flySetRPM(shootRPM),
                        flyRightShooter.INSTANCE.flySetRPM(shootRPM),
                        new FollowPath(line1, true, 1.0)
                ),
                new Delay(0.4),
                new ParallelGroup(
                        //ShootPattern(intakeLED.INSTANCE.frontColor(), intakeLED.INSTANCE.middleColor(),intakeLED.INSTANCE.backColor()),
                        frontLauncher.INSTANCE.shootCycle(),
                        new SequentialGroup(
                                new Delay(0.25),
                                middleLauncher.INSTANCE.shootCycle()
                        ),
                        new SequentialGroup(
                                new Delay(0.5),
                                backLauncher.INSTANCE.shootCycle()
                        ),


                        new SequentialGroup(
                                new Delay(0.9),
                                new FollowPath(line2, true, 0.8)
//                                new FollowPath(line2, true, 0.7)
                                )
                        ),
                new Delay(0.1),
                new FollowPath(line3, true, 1.0),
                new Delay(0.75),
                new ParallelGroup(
                        //ShootPattern(intakeLED.INSTANCE.frontColor(), intakeLED.INSTANCE.middleColor(),intakeLED.INSTANCE.backColor()),
                        frontLauncher.INSTANCE.shootCycle(),
                        new SequentialGroup(
                                new Delay(0.25),
                                middleLauncher.INSTANCE.shootCycle()
                        ),
                        new SequentialGroup(
                                new Delay(0.5),
                                backLauncher.INSTANCE.shootCycle()
                        ),

                        new SequentialGroup(
                                new Delay(0.9),
                                new FollowPath(line4, true, 1.0)
//                                new FollowPath(line2, true, 0.7)
                        )
                ),
                new Delay(1.0),
                new FollowPath(line5, true, 1.0),
                new Delay(0.75),
                new ParallelGroup(
                        //ShootPattern(intakeLED.INSTANCE.frontColor(), intakeLED.INSTANCE.middleColor(),intakeLED.INSTANCE.backColor()),
                        frontLauncher.INSTANCE.shootCycle(),
                        new SequentialGroup(
                                new Delay(0.25),
                                middleLauncher.INSTANCE.shootCycle()
                        ),
                        new SequentialGroup(
                                new Delay(0.5),
                                backLauncher.INSTANCE.shootCycle()
                        ),

                        new SequentialGroup(
                                new Delay(0.9),
                                new FollowPath(line6, true, 1.0)
//                                new FollowPath(line2, true, 0.7)
                        )
                ),
                new Delay(1.0),
                new FollowPath(line7, true, 1.0),
                new Delay(0.75),
                new ParallelGroup(
                        //ShootPattern(intakeLED.INSTANCE.frontColor(), intakeLED.INSTANCE.middleColor(),intakeLED.INSTANCE.backColor()),
                        frontLauncher.INSTANCE.shootCycle(),
                        new SequentialGroup(
                                new Delay(0.25),
                                middleLauncher.INSTANCE.shootCycle()
                        ),
                        new SequentialGroup(
                                new Delay(0.5),
                                backLauncher.INSTANCE.shootCycle()
                        ),

                        new SequentialGroup(
                                new Delay(0.9),
                                new FollowPath(line8, true, 1.0)
//                                new FollowPath(line2, true, 0.7)
                        )
                ),
                new Delay(1.0),
                new FollowPath(line9, true, 1.0),
                new Delay(0.75),
                new ParallelGroup(
                        //ShootPattern(intakeLED.INSTANCE.frontColor(), intakeLED.INSTANCE.middleColor(),intakeLED.INSTANCE.backColor()),
                        new InstantCommand(()-> {isShooting=true;}),

                        new SequentialGroup(
                                new Delay(1.4),
                                new FollowPath(line10, true, 1.0)
//                                new FollowPath(line2, true, 0.7)
                        )
                ),
                new Delay(0.1),
                new FollowPath(line11, true, 1.0),
                new Delay(0.75),
                new ParallelGroup(
                        //ShootPattern(intakeLED.INSTANCE.frontColor(), intakeLED.INSTANCE.middleColor(),intakeLED.INSTANCE.backColor()),
                        new InstantCommand(()-> {isShooting=true;}),

                        new SequentialGroup(
                                new Delay(1.4),
                                new FollowPath(line12, true, 1.0)
//                                new FollowPath(line2, true, 0.7)
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


        // Feedback to Driver Hub for debugging

        telemetry.addData("x", follower().getPose().getX());
        telemetry.addData("y", follower().getPose().getY());
        telemetry.addData("heading", Math.toRadians(follower().getPose().getHeading()));

        telemetry.addData("Order", order);
        telemetry.addData("Updates", counter);

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
//        limeLight.INSTANCE.On();
        follower().deactivateAllPIDFs();
        follower().activateDrive();
        follower().activateCentripetal();
        follower().activateHeading();
        follower().activateTranslational();
        new SequentialGroup(
                doAuto1()
                ).schedule();

        USE_WEBCAM = false;
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
if(true){
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
        }
//        else if(limeLight.INSTANCE.GetBalls()%3==1){
else if(true){
            if(aprilValue==21){

                if(Objects.equals(fc, "green")){
                    order = "MBf";
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
            else if(aprilValue == 22){
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
            else if(aprilValue == 23){
                if(Objects.equals(fc, "green")){
                    order = "BFM";
                }
                else if (Objects.equals(mc, "green")){
                    order = "FBM";
                }
                else if(Objects.equals(bc, "green")){
                    order = "MBF";
                }
                else{
                    order = "FMB";
                }
            }
            else{
                order = "FMB";
            }
        }
//        else if(limeLight.INSTANCE.GetBalls()%3==2){
else if(true){
            if(aprilValue==21){
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
            else if(aprilValue == 22){
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
            else if(aprilValue == 23){
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
            else{
                order = "FMB";
            }
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
                            new Delay(0.50),
                            new ParallelGroup(
                                    launchTwo.shootCycle(),
                                    new SequentialGroup(
                                            new Delay(0.50),
                                            launchThree.shootCycle()
                                    )
                            )
                    )
            );

//        return new SequentialGroup(new Delay (0)
//        );
    }

}
