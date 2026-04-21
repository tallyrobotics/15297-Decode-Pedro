package org.firstinspires.ftc.teamcode.legacy;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.legacy.subsystems.backLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.frontLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyLeftShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyRightShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.intake;
import org.firstinspires.ftc.teamcode.legacy.subsystems.intakeLED;
import org.firstinspires.ftc.teamcode.legacy.subsystems.limeLight;
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
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.MotorEx;

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
                new SubsystemComponent(limeLight.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private String alliance = "B";
    private final Double turbo = 1.0;
    private final Double normal = 0.75;
    private final Double turtle = 0.35;
    private Double speed;
    private double shootRPM = 0.0;
    private String shootOrder = "quick";
    private boolean isShooting = false;
    private MotorEx leftFront;
    private MotorEx leftRear;
    private MotorEx rightFront;
    private MotorEx rightRear;

    private Pose shootPoseClose = new Pose(96.0, 95.6, Math.toRadians(-45.0));
    private final Pose startPoseRed = new Pose(94.0, 110.0, Math.toRadians(-58.0));
    private final Pose startPoseBlue = new Pose(50.0, 110.0, Math.toRadians(-122.0));
    private final Double shootRPMClose = 1900.0;
    private final Double shootRPMFar = 2250.0;
    private Double setRPM;
    private boolean bPressed = false;
    private boolean isFollowing = false;
    private boolean correctShootPose = true;

    private PathChain line1, line2;
    public void buildPaths() {
        line1 = follower().pathBuilder().addPath(
                        new BezierLine(
                                shootPoseClose,
        new Pose(shootPoseClose.getX()-0.1, shootPoseClose.getY()-0.1)
                        )
                ).setConstantHeadingInterpolation(shootPoseClose.getHeading())
                .build();
    }

        @Override
    public void onInit() {
            leftFront = new MotorEx("leftFront");
            leftRear = new MotorEx("leftRear");
            rightFront = new MotorEx("rightFront");
            rightRear = new MotorEx("rightRear");
            limeLight.INSTANCE.Off().schedule();
            setRPM = shootRPMClose;

//            follower().setConstraints(new PathConstraints(0.99, 100, 1.0, 1.0));

        speed = normal;
        shootRPM = setRPM;
        follower().activateAllPIDFs();
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
        follower().setTeleOpDrive(
                -gamepad1.left_stick_y*speed,
                -gamepad1.left_stick_x*speed,
                -gamepad1.right_stick_x*speed*0.6,
                true);

        if(isShooting){
            Shoot().schedule();
            if(correctShootPose){
                shootPoseClose = follower().getPose();
                buildPaths();
            }
            correctShootPose = true;
        }
        if(isShooting||bPressed){
            if(!follower().isTeleopDrive()){
                follower().startTeleopDrive();
            }
            isFollowing = false;
        }
        if(isFollowing){
            buildPaths();
            new SequentialGroup(
                    new FollowPath(line1, true, 1.0)
            ).schedule();
        }
        telemetry.addData("shootRPM", shootRPM);
        telemetry.addData("Shoot Order", shootOrder);
        leftFront.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.update();

    }

    @Override
    public void onStartButtonPressed() {
        follower().startTeleopDrive();

        intakeLED.INSTANCE.PriorityOn().schedule();
        Gamepads.gamepad1().leftBumper().whenTrue(new InstantCommand(() -> {speed=turtle;}));

        Gamepads.gamepad1().leftBumper().whenBecomesFalse(new InstantCommand(() -> {speed=normal;}));
        Gamepads.gamepad1().rightBumper().whenTrue(new InstantCommand(() -> {speed=turbo;}));
        Gamepads.gamepad1().rightBumper().whenBecomesFalse(new InstantCommand(() -> {speed=normal;}));
        Gamepads.gamepad1().x().whenFalse(new InstantCommand(()->{follower().update();}));

        Gamepads.gamepad1().dpadUp().whenBecomesTrue(
                new InstantCommand(() -> {
                    shootRPM = shootRPMFar;
                    flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
                    flyRightShooter.INSTANCE.flySetRPM(shootRPM);
                }
        ));
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(
                new InstantCommand(() -> {
                    shootRPM = shootRPMClose;
                    flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
                    flyRightShooter.INSTANCE.flySetRPM(shootRPM);
                }
        ));

        Gamepads.gamepad1().y().whenBecomesTrue(
                    new InstantCommand(()-> {
                        isShooting = true;
                    }));

        Gamepads.gamepad1().a().whenTrue(
                new InstantCommand(()->{
                    correctShootPose = false;
                    isFollowing = true;
                })
        );

        Gamepads.gamepad1().b().whenTrue(
                new InstantCommand(()->{bPressed = true;})
        );

        Gamepads.gamepad1().b().whenFalse(
                new InstantCommand(()->{bPressed = false;
                    follower().update();})
        );

        Gamepads.gamepad2().dpadDown().whenBecomesTrue(
                new InstantCommand(() -> {
                    shootRPM -= 50.0;
                    flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
                    flyRightShooter.INSTANCE.flySetRPM(shootRPM);
                }
                ));

        Gamepads.gamepad2().dpadUp().whenBecomesTrue(
            new InstantCommand(() -> {
                shootRPM += 50.0;
                flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
                flyRightShooter.INSTANCE.flySetRPM(shootRPM);
            }
        ));

        Gamepads.gamepad2().y().whenBecomesTrue(new InstantCommand(()-> {shootOrder = "quick";}));
        Gamepads.gamepad2().b().whenBecomesTrue(new InstantCommand(()-> {shootOrder = "BFM";}));
        Gamepads.gamepad2().a().whenBecomesTrue(new InstantCommand(()-> {shootOrder = "MBF";}));
        Gamepads.gamepad2().x().whenBecomesTrue(new InstantCommand(()-> {shootOrder = "FMB";}));

        Gamepads.gamepad2().leftBumper().whenBecomesTrue(
                intakeLED.INSTANCE.PriorityOff()
        );
        Gamepads.gamepad2().leftBumper().whenBecomesFalse(
                intakeLED.INSTANCE.PriorityOn()
        );
        Gamepads.gamepad1().rightStickButton().whenFalse(new InstantCommand(()->{follower().update();}));
        flyRightShooter.INSTANCE.flySetRPM(shootRPM).schedule();
        flyLeftShooter.INSTANCE.flySetRPM(shootRPM).schedule();
    }

    public Command Shoot(){
        if (shootOrder == "FMB") {
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
                            new InstantCommand(()->{
                                isShooting = false;})
                    )
            );
        } else if (shootOrder == "MBF") {
            return new ParallelGroup(
                    middleLauncher.INSTANCE.shootCycle(),
                    new SequentialGroup(
                            new Delay(0.2),
                            backLauncher.INSTANCE.shootCycle()
                    ),
                    new SequentialGroup(
                            new Delay(0.4),
                            frontLauncher.INSTANCE.shootCycle(),
                            new Delay(0.05),
                            new InstantCommand(()->{
                            isShooting = false;})
                    )
            );
        } else if (shootOrder == "BFM") {
            return new ParallelGroup(
                    backLauncher.INSTANCE.shootCycle(),
                    new SequentialGroup(
                            new Delay(0.2),
                            frontLauncher.INSTANCE.shootCycle()
                    ),
                    new SequentialGroup(
                            new Delay(0.4),
                            middleLauncher.INSTANCE.shootCycle(),
                            new Delay(0.05),
                            new InstantCommand(()->{
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
                            new InstantCommand(()->{
                                isShooting = false;})
                    )
            );
        }
    }

    @Override
    public void onStop() {
    }
}
