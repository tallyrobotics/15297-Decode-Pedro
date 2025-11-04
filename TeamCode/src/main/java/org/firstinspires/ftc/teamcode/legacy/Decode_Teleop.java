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
import dev.nextftc.hardware.driving.FieldCentric;
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

    public IMUEx imu = new IMUEx("imu", Direction.BACKWARD,Direction.UP);

    private final Double turbo = 1.0;
    private final Double normal = 0.75;
    private final Double turtle = 0.5;
    private Double speed;
    private final int shootRPM = 1000;

    public MecanumDriverControlled driverControlled;


    @Override
    public void onInit(){
        speed = normal;
        leftFrontMotor = new MotorEx(leftFrontName);
        leftRearMotor = new MotorEx(leftRearName);
        rightFrontMotor = new MotorEx(rightFrontName);
        rightRearMotor = new MotorEx(rightRearName);

        leftFrontMotor.setDirection(1);
        leftRearMotor.setDirection(1);
        rightFrontMotor.setDirection(1);
        rightRearMotor.setDirection(1);


        PedroComponent.follower().setStartingPose(new Pose(0, 0, 0));

        flyRightShooter.INSTANCE.flyRightOff();
        flyLeftShooter.INSTANCE.flyLeftOff();
        PedroComponent.follower().update();

    }

    @Override
    public void onUpdate(){
        PedroComponent.follower().update();
        driverControlled.setScalar(speed);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("flyLeft RPM",flyLeftShooter.INSTANCE.flyLeft.getVelocity());
        telemetry.addData("flyRight RPM",flyRightShooter.INSTANCE.flyRight.getVelocity());
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed()
    {
        intake.INSTANCE.IntakeIn().schedule();


        driverControlled = new MecanumDriverControlled(
                leftFrontMotor,
                rightFrontMotor,
                leftRearMotor,
                rightRearMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX());//,
//


        driverControlled.schedule();

        Gamepads.gamepad1().leftBumper().whenTrue(() -> new InstantCommand(() -> {speed=turtle;}));
        Gamepads.gamepad1().rightBumper().whenTrue(() -> new InstantCommand(() -> {speed=turbo;}));
        Gamepads.gamepad1().rightBumper().whenBecomesFalse(() -> new InstantCommand(() -> {speed=normal;}));
        Gamepads.gamepad1().leftBumper().whenBecomesFalse(() -> new InstantCommand(() -> {speed=normal;}));

        Gamepads.gamepad2().leftBumper().whenBecomesTrue(intake.INSTANCE.IntakeOut());
        Gamepads.gamepad2().leftBumper().whenBecomesFalse(intake.INSTANCE.IntakeIn());
        Gamepads.gamepad2().x().whenBecomesTrue(intake.INSTANCE.IntakeOff());
        Gamepads.gamepad2().rightBumper().whenBecomesTrue(new ParallelGroup(
                backTwo.INSTANCE.shootCycle(),
                new SequentialGroup(
                        new Delay(0.5),
                        frontOne.INSTANCE.shootCycle()
                )
                )
        );

        Gamepads.gamepad2().dpadDown().whenBecomesTrue(new ParallelGroup(
                flyRightShooter.INSTANCE.flyRightSetRPM(400),
                flyLeftShooter.INSTANCE.flyLeftSetRPM(400)
        ));
        Gamepads.gamepad2().dpadLeft().whenBecomesTrue(new ParallelGroup(
                flyRightShooter.INSTANCE.flyRightSetRPM(200),
                flyLeftShooter.INSTANCE.flyLeftSetRPM(200)
        ));
        Gamepads.gamepad2().dpadRight().whenBecomesTrue(new ParallelGroup(
                flyRightShooter.INSTANCE.flyRightSetRPM(100),
                flyLeftShooter.INSTANCE.flyLeftSetRPM(100)
        ));
        Gamepads.gamepad2().dpadUp().whenBecomesTrue(new ParallelGroup(
                flyRightShooter.INSTANCE.flyRightSetRPM(1000),
                flyLeftShooter.INSTANCE.flyLeftSetRPM(1000)
        ));

    }


    @Override
    public void onStop(){
    }

}
