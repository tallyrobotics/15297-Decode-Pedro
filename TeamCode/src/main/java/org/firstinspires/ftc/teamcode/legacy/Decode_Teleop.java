package org.firstinspires.ftc.teamcode.legacy;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.legacy.subsystems.backLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.frontLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyLeftShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyRightShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.intake;
import org.firstinspires.ftc.teamcode.legacy.subsystems.intakeLED;
import org.firstinspires.ftc.teamcode.legacy.subsystems.middleLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.shootersLED;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.extensions.pedro.PedroComponent;

@Configurable
@TeleOp(name = "Decode Teleop")
public class Decode_Teleop extends NextFTCOpMode {

    public Decode_Teleop()
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(intakeLED.INSTANCE),
                new SubsystemComponent(shootersLED.INSTANCE),
                new SubsystemComponent(frontLauncher.INSTANCE),
                new SubsystemComponent(middleLauncher.INSTANCE),
                new SubsystemComponent(backLauncher.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private String alliance = "R";
    private final Double turbo = 1.0;
    private final Double normal = 0.75;
    private final Double turtle = 0.3;
    private Double speed;
    private double shootRPM = 0.0;
    private String shootOrder = "quick";
    private boolean isShooting = false;
    private boolean isActive = false;

    static double frontWait = 0.00;
    static double middleWait = 0.55;
    static double backWait = 1.1;

    private boolean inPath = false;
    private boolean startTele = false;

    private final Pose shootPoseRed = new Pose(96.1, 95.7, Math.toRadians(-45.0));
    private final Pose targetPoseRed = new Pose(96.0, 95.6, Math.toRadians(-45.0));
    private final Pose startPoseRed = new Pose(120.0, 72.0, Math.toRadians(-90.0));
    private final Pose shootPoseBlue = new Pose(47.6, 96.4, Math.toRadians(-135.0));
    private final Pose targetPoseBlue = new Pose(47.7, 96.3, Math.toRadians(-135.0));
    private final Pose startPoseBlue = new Pose(24.0, 72.0, Math.toRadians(-90.0));
    private final Double shootRPMRed = 1820.0;
    private final Double shootRPMBlue = 1820.0;
    private Double setRPM;
    

    public MecanumDriverControlled driverControlled;

    private PathChain line1;
    public void buildPaths() {

        if(alliance=="R"){
            line1 = follower().pathBuilder().addPath(
                            new BezierLine(
                                    targetPoseRed,
                                    shootPoseRed
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-45.0))

                    .build();
        }
        else if(alliance=="B")
        {
            line1 = follower().pathBuilder().addPath(
                            new BezierLine(
                                    targetPoseBlue,
                                    shootPoseBlue
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(-135.0))

                    .build();
        }

    }


        @Override
    public void onInit() {

        speed = normal;
        if(alliance=="R"){
            setRPM = shootRPMRed;
        }
        else if(alliance=="B"){
            setRPM = shootRPMBlue;
        }
        shootRPM = setRPM;


//        colorFront = hardwareMap.get(ColorSensor.class, "colorFront");
//        colorMid = hardwareMap.get(ColorSensor.class, "colorMid");
//        colorBack = hardwareMap.get(ColorSensor.class, "colorBack");
//        ledMidRed = hardwareMap.get(LED.class, "ledMidRed");
//        ledMidGreen = hardwareMap.get(LED.class, "ledMidGreen");
//        ledBackRed = hardwareMap.get(LED.class, "ledBackRed");
//        ledBackGreen = hardwareMap.get(LED.class, "ledBackGreen");
//        ledFrontRed = hardwareMap.get(LED.class, "ledFrontRed");
//        ledFrontGreen = hardwareMap.get(LED.class, "ledFrontGreen");
//        colorFront_DistanceSensor = hardwareMap.get(DistanceSensor.class, "colorFront");
//        colorMid_DistanceSensor = hardwareMap.get(DistanceSensor.class, "colorMid");
//        colorBack_DistanceSensor = hardwareMap.get(DistanceSensor.class, "colorBack");
//        gain = 10;
//        flyLeft = hardwareMap.get(DcMotorEx.class,"flyLeft");
////        flyLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        flyLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        flyLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            follower().deactivateAllPIDFs();
            follower().activateDrive();
            follower().activateCentripetal();
            follower().activateHeading();
            follower().activateTranslational();
            if(alliance=="R"){
                follower().setStartingPose(startPoseRed);
            }
            else if(alliance=="B"){
                follower().setStartingPose(startPoseBlue);
            }

        follower().update();
            buildPaths();
    }

    @Override
    public void onUpdate() {
        follower().update();
        follower().setTeleOpDrive(
                -gamepad1.left_stick_y*speed,
                -gamepad1.left_stick_x*speed,
                -gamepad1.right_stick_x*speed*0.5,
                true);

        telemetry.addData("flyLeft RPM",flyLeftShooter.INSTANCE.flyMotor.getVelocity());
//        telemetry.addData("flyLeft Target RPM", flyLeftShooter.INSTANCE.controlSystem.getGoal().getVelocity());
        telemetry.addData("flyRight RPM",flyRightShooter.INSTANCE.flyMotor.getVelocity());
//        telemetry.addData("flyRight Target RPM", flyRightShooter.INSTANCE.controlSystem.getGoal().getVelocity());
        telemetry.addData("shootRPM", shootRPM);

        telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.");
        telemetry.addLine(" ");
        telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value.");
        telemetry.addLine(" ");
        if(isShooting&&!isActive){
            Shoot().schedule();
            isActive = true;
        }
//        if((Math.abs(Gamepads.gamepad1().leftStickX().get())>0.5||Math.abs(Gamepads.gamepad1().leftStickY().get())>0.5||Math.abs(Gamepads.gamepad1().rightStickX().get())>0.5)&&follower().isBusy()){
//
//
//
//        }
        if((Math.abs(gamepad1.left_stick_x)>0.2||Math.abs(gamepad1.left_stick_y)>0.2||Math.abs(gamepad1.right_stick_x)>0.2||isShooting||follower().atPose(shootPoseRed,0.15,0.15,0.04))){
            if(!follower().isTeleopDrive()){
                follower().startTeleopDrive();
            }
        }




        telemetry.update();

    }

    @Override
    public void onStartButtonPressed() {


//        flyLeft.setVelocity(200);

        follower().startTeleopDrive();

        intake.INSTANCE.on().schedule();

        Gamepads.gamepad1().leftBumper().whenTrue(new InstantCommand(() -> {speed=turtle;}));

        Gamepads.gamepad1().leftBumper().whenBecomesFalse(new InstantCommand(() -> {speed=normal;}));
        Gamepads.gamepad1().rightBumper().whenTrue(new InstantCommand(() -> {speed=turbo;}));
        Gamepads.gamepad1().rightBumper().whenBecomesFalse(new InstantCommand(() -> {speed=normal;}));

        Gamepads.gamepad1().dpadUp().whenBecomesTrue(
                new InstantCommand(() -> {
                    shootRPM = 2200.0;
                    flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
                    flyRightShooter.INSTANCE.flySetRPM(shootRPM);
                }
        ));
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(
                new InstantCommand(() -> {
                    shootRPM = setRPM;
                    flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
                    flyRightShooter.INSTANCE.flySetRPM(shootRPM);
                }
        ));


        Gamepads.gamepad1().y().whenBecomesTrue(
                    new InstantCommand(()-> {
                        isShooting = true;
                    }));
//                new ParallelGroup(
//                        frontLauncher.INSTANCE.shootCycle(),
//                        new SequentialGroup(
//                                new Delay(0.2),
//                                middleLauncher.INSTANCE.shootCycle()
//                        ),
//                        new SequentialGroup(
//                                new Delay(0.4),
//                                backLauncher.INSTANCE.shootCycle(),
//                                new Delay(0.3)
//                        )
//                )
//                );

        Gamepads.gamepad1().a().whenTrue(
                new FollowPath(line1, true, 1.0)
        );

        Gamepads.gamepad2().dpadDown().whenBecomesTrue(
                new InstantCommand(() -> {
                    shootRPM -= 20.0;
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
                shootRPM += 20.0;
                flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
                flyRightShooter.INSTANCE.flySetRPM(shootRPM);
            }
        ));
        Gamepads.gamepad2().y().whenBecomesTrue(new InstantCommand(()-> {shootOrder = "quick";}));
        Gamepads.gamepad2().x().whenBecomesTrue(new InstantCommand(()-> {shootOrder = "BFM";}));
        Gamepads.gamepad2().a().whenBecomesTrue(new InstantCommand(()-> {shootOrder = "MBF";}));
        Gamepads.gamepad2().b().whenBecomesTrue(new InstantCommand(()-> {shootOrder = "FMB";}));


        flyRightShooter.INSTANCE.flySetRPM(shootRPM).schedule();
        flyLeftShooter.INSTANCE.flySetRPM(shootRPM).schedule();
        intake.INSTANCE.on().schedule();


    }

    public Command Shoot(){
        if (shootOrder == "FMB") {
            return new ParallelGroup(
                    frontLauncher.INSTANCE.shootCycle(),
                    new SequentialGroup(
                            new Delay(0.5),
                            middleLauncher.INSTANCE.shootCycle()
                    ),
                    new SequentialGroup(
                            new Delay(1.0),
                            backLauncher.INSTANCE.shootCycle(),
                            new Delay(0.3),
                            new InstantCommand(()->{isActive = false;
                                isShooting = false;})
                    )
            );
        } else if (shootOrder == "MBF") {
            return new ParallelGroup(
                    middleLauncher.INSTANCE.shootCycle(),
                    new SequentialGroup(
                            new Delay(0.5),
                            backLauncher.INSTANCE.shootCycle()
                    ),
                    new SequentialGroup(
                            new Delay(1.0),
                            frontLauncher.INSTANCE.shootCycle(),
                            new Delay(0.3),
                            new InstantCommand(()->{isActive = false;
                            isShooting = false;})
                    )
            );
        } else if (shootOrder == "BFM") {
            return new ParallelGroup(
                    backLauncher.INSTANCE.shootCycle(),
                    new SequentialGroup(
                            new Delay(0.5),
                            frontLauncher.INSTANCE.shootCycle()
                    ),
                    new SequentialGroup(
                            new Delay(1.0),
                            middleLauncher.INSTANCE.shootCycle(),
                            new Delay(0.3),
                            new InstantCommand(()->{isActive = false;
                                isShooting = false;})
                    )
            );
        } else {
            return new ParallelGroup(
                    frontLauncher.INSTANCE.shootCycle(),
                    new SequentialGroup(
                            new Delay(0.2),
                            middleLauncher.INSTANCE.shootCycle()
                    ),
                    new SequentialGroup(
                            new Delay(0.4),
                            backLauncher.INSTANCE.shootCycle(),
                            new Delay(0.05),
                            new InstantCommand(()->{isActive = false;
                                isShooting = false;})
                    )
            );
        }
    }



    @Override
    public void onStop() {
    }
}
