package org.firstinspires.ftc.teamcode.legacy;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.legacy.subsystems.backTwo;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyLeftShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyRightShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.frontOne;
import org.firstinspires.ftc.teamcode.legacy.subsystems.intake;
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

@TeleOp(name = "Decode Teleop")
public class Decode_Teleop extends NextFTCOpMode {

    public Decode_Teleop()
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(intake.INSTANCE),
                new SubsystemComponent(flyRightShooter.INSTANCE),
                new SubsystemComponent(flyLeftShooter.INSTANCE),
                new SubsystemComponent(backTwo.INSTANCE),
                new SubsystemComponent(frontOne.INSTANCE),
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
//    public DcMotorEx flyLeft;

    public IMUEx imu = new IMUEx("imu", Direction.BACKWARD,Direction.UP);

    private final Double turbo = 1.0;
    private final Double normal = 0.75;
    private final Double turtle = 0.5;
    private Double speed;
    private double shootRPM = 0.0;

    public MecanumDriverControlled driverControlled;

    @Override
    public void onInit() {
        speed = normal;
        shootRPM = 1900.0;

        leftFrontMotor = new MotorEx(leftFrontName);
        leftRearMotor = new MotorEx(leftRearName);
        rightFrontMotor = new MotorEx(rightFrontName);
        rightRearMotor = new MotorEx(rightRearName);
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

        Gamepads.gamepad1().leftBumper().whenTrue(new InstantCommand(() -> {speed=turtle;}));

        Gamepads.gamepad1().leftBumper().whenBecomesFalse(new InstantCommand(() -> {speed=normal;}));
        Gamepads.gamepad1().rightBumper().whenTrue(new InstantCommand(() -> {speed=turbo;}));
        Gamepads.gamepad1().rightBumper().whenBecomesFalse(new InstantCommand(() -> {speed=normal;}));

        Gamepads.gamepad2().leftBumper().whenBecomesTrue(intake.INSTANCE.IntakeOut());
        Gamepads.gamepad2().leftBumper().whenBecomesFalse(intake.INSTANCE.IntakeIn());

        Gamepads.gamepad2().y().whenBecomesTrue(intake.INSTANCE.IntakeOff());

        Gamepads.gamepad2().x().whenBecomesTrue(frontOne.INSTANCE.shootCycle());
        Gamepads.gamepad2().a().whenBecomesTrue(backTwo.INSTANCE.shootCycle());

        Gamepads.gamepad2().b().whenBecomesTrue(
            new ParallelGroup(
                backTwo.INSTANCE.shootCycle(),
//                new SequentialGroup(
//                    new Delay(0.5),
                    frontOne.INSTANCE.shootCycle()
                )
//            )
        );

        Gamepads.gamepad2().dpadDown().whenBecomesTrue(
            new InstantCommand(() -> {
                shootRPM -= 100.0;
                flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
                flyRightShooter.INSTANCE.flySetRPM(shootRPM);
            }
        ));
//        Gamepads.gamepad2().dpadLeft().whenBecomesTrue(
//            new InstantCommand(() -> {
//                shootRPM = 200.0;
//                flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
//                flyRightShooter.INSTANCE.flySetRPM(shootRPM*2);
//            }
//        ));
//        Gamepads.gamepad2().dpadRight().whenBecomesTrue(
//            new InstantCommand(() -> {
//                shootRPM = -300.0;
//                flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
//                flyRightShooter.INSTANCE.flySetRPM(shootRPM*2);
//            }
//        ));
        Gamepads.gamepad2().dpadUp().whenBecomesTrue(
            new InstantCommand(() -> {
                shootRPM += 100.0;
                flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
                flyRightShooter.INSTANCE.flySetRPM(shootRPM);
            }
        ));

        flyRightShooter.INSTANCE.flySetRPM(shootRPM).schedule();
        flyLeftShooter.INSTANCE.flySetRPM(shootRPM).schedule();
//        flyRightShooter.INSTANCE.getDefaultCommand().schedule();// .flySetRPM(shootRPM).schedule();
//        flyLeftShooter.INSTANCE.getDefaultCommand().schedule();// .flySetRPM(shootRPM).schedule();

        intake.INSTANCE.IntakeIn().schedule();
    }

    @Override
    public void onStop() {
    }
}
