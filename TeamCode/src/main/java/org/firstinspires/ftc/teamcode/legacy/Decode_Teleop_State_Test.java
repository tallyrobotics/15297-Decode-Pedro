package org.firstinspires.ftc.teamcode.legacy;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.legacy.subsystems.backLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyLeftShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.flyRightShooter;
import org.firstinspires.ftc.teamcode.legacy.subsystems.frontLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.intakeLED;
import org.firstinspires.ftc.teamcode.legacy.subsystems.middleLauncher;
import org.firstinspires.ftc.teamcode.legacy.subsystems.shootersLED;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
@TeleOp(name = "Decode Teleop TEST")
public class Decode_Teleop_State_Test extends NextFTCOpMode {

    public Decode_Teleop_State_Test()
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

    public String leftFrontName = "leftFront";
    public String leftRearName = "leftRear";
    public String rightFrontName = "rightFront";
    public String rightRearName = "rightRear";

    public MotorEx leftFrontMotor;
    public MotorEx leftRearMotor;
    public MotorEx rightFrontMotor;
    public MotorEx rightRearMotor;

    private final Double turbo = 1.0;
    private final Double normal = 0.75;
    private final Double turtle = 0.3;
    private Double speed;
    private double shootRPM = 0.0;

    static double frontWait = 0.00;
    static double middleWait = 0.55;
    static double backWait = 1.1;

    private final Pose shootPose = new Pose(100.7, 99.3, Math.toRadians(-45.0));

    public MecanumDriverControlled driverControlled;

    private PathChain line1;
    public void buildPaths() {

        line1 = follower().pathBuilder().addPath(
                        new BezierLine(
                                follower().getPose(),

                                shootPose
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(-45.0))

                .build();
    }


        @Override
    public void onInit() {

        speed = normal;
        shootRPM = 1800;

        leftFrontMotor = new MotorEx(leftFrontName);
        leftRearMotor = new MotorEx(leftRearName);
        rightFrontMotor = new MotorEx(rightFrontName);
        rightRearMotor = new MotorEx(rightRearName);


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

        follower().setStartingPose(new Pose(120.000, 65.000, 0));
        follower().update();
    }

    @Override
    public void onUpdate() {
        follower().update();

        driverControlled.setScalar(speed);

        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("flyLeft RPM",flyLeftShooter.INSTANCE.flyMotor.getVelocity());
//        telemetry.addData("flyLeft Target RPM", flyLeftShooter.INSTANCE.controlSystem.getGoal().getVelocity());
        telemetry.addData("flyRight RPM",flyRightShooter.INSTANCE.flyMotor.getVelocity());
//        telemetry.addData("flyRight Target RPM", flyRightShooter.INSTANCE.controlSystem.getGoal().getVelocity());
        telemetry.addData("shootRPM", shootRPM);

        telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.");
        telemetry.addLine(" ");
        telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value.");
        telemetry.addLine(" ");
        buildPaths();
        // Update the gain value if either of the A or B gamepad buttons is being held
        // A gain of less than 1 will make the values smaller, which is not helpful.
//        if (gamepad1.a) {
//            // Only increase the gain by a small amount, since this loop will occur multiple times per second.
//            gain = (int) (gain + 0.005);
//        } else if (gamepad1.b && gain > 1) {
//            gain = (int) (gain - 0.005);
//        }
//        telemetry.addData("Gain", gain);
//        // Tell the sensor our desired gain value (normally you would do this during initialization, not during the loop)
//        ((NormalizedColorSensor) colorFront).setGain(gain);
//        // Tell the sensor our desired gain value (normally you would do this during initialization, not during the loop)
//        ((NormalizedColorSensor) colorMid).setGain(gain);
//        // Tell the sensor our desired gain value (normally you would do this during initialization, not during the loop)
//        ((NormalizedColorSensor) colorBack).setGain(gain);
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

        Gamepads.gamepad2().x().whenBecomesTrue(backLauncher.INSTANCE.shootCycle());
        Gamepads.gamepad2().a().whenBecomesTrue(middleLauncher.INSTANCE.shootCycle());
        Gamepads.gamepad2().b().whenBecomesTrue(frontLauncher.INSTANCE.shootCycle());

        Gamepads.gamepad1().x().whenBecomesTrue(            new ParallelGroup(

                new SequentialGroup(
                        new Delay(frontWait),
                        backLauncher.INSTANCE.shootCycle()
                ),
                new SequentialGroup(
                        new Delay(middleWait),
                        frontLauncher.INSTANCE.shootCycle()
                ),
                new SequentialGroup(
                        new Delay(backWait),
                        middleLauncher.INSTANCE.shootCycle()
                )));
        Gamepads.gamepad1().a().whenBecomesTrue(            new ParallelGroup(

                new SequentialGroup(
                        new Delay(frontWait),
                        middleLauncher.INSTANCE.shootCycle()
                ),
                new SequentialGroup(
                        new Delay(middleWait),
                        backLauncher.INSTANCE.shootCycle()
                ),
                new SequentialGroup(
                        new Delay(backWait),
                        frontLauncher.INSTANCE.shootCycle()
                )));
        Gamepads.gamepad1().b().whenBecomesTrue(            new ParallelGroup(

                new SequentialGroup(
                        new Delay(frontWait),
                        frontLauncher.INSTANCE.shootCycle()
                ),
                new SequentialGroup(
                        new Delay(middleWait),
                        middleLauncher.INSTANCE.shootCycle()
                ),
                new SequentialGroup(
                        new Delay(backWait),
                        backLauncher.INSTANCE.shootCycle()
                )));

        Gamepads.gamepad1().dpadUp().whenBecomesTrue(
                new InstantCommand(() -> {
                    shootRPM = 2200.0;
                    flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
                    flyRightShooter.INSTANCE.flySetRPM(shootRPM);
                }
        ));
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(
                new InstantCommand(() -> {
                    shootRPM = 1700.0;
                    flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
                    flyRightShooter.INSTANCE.flySetRPM(shootRPM);
                }
        ));


        Gamepads.gamepad2().rightTrigger().greaterThan(0.1).whenTrue(new InstantCommand(() -> {
                    shootRPM = -400.0;
            flyLeftShooter.INSTANCE.flySetRPM(shootRPM);
            flyRightShooter.INSTANCE.flySetRPM(shootRPM);
                }
                )
        );


        Gamepads.gamepad1().y().whenBecomesTrue(
                new SequentialGroup(
                        new FollowPath(line1, false, 1.0),
                        new Delay(0.25),
                        new ParallelGroup(

                                new SequentialGroup(
                                        new Delay(0.00),
                                        frontLauncher.INSTANCE.shootCycle()
                                ),
                                new SequentialGroup(
                                        new Delay(0.25),
                                        middleLauncher.INSTANCE.shootCycle()
                                ),
                                new SequentialGroup(
                                        new Delay(0.4),
                                        backLauncher.INSTANCE.shootCycle()
                                )
                        )
                )
        );

        Gamepads.gamepad1().leftStickX().lessThan(-0.1).whenBecomesTrue(
                new InstantCommand(() -> {follower().breakFollowing();})
        );
        Gamepads.gamepad1().leftStickX().greaterThan(0.1).whenBecomesTrue(
                new InstantCommand(() -> {follower().breakFollowing();})
        );
        Gamepads.gamepad1().leftStickY().lessThan(-0.1).whenBecomesTrue(
                new InstantCommand(() -> {follower().breakFollowing();})
        );
        Gamepads.gamepad1().leftStickY().greaterThan(0.1).whenBecomesTrue(
                new InstantCommand(() -> {follower().breakFollowing();})
        );
        Gamepads.gamepad1().rightStickX().lessThan(-0.1).whenBecomesTrue(
                new InstantCommand(() -> {follower().breakFollowing();})
        );
        Gamepads.gamepad1().rightStickX().greaterThan(0.1).whenBecomesTrue(
                new InstantCommand(() -> {follower().breakFollowing();})
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
                    shootRPM = 1700;
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




    }


    @Override
    public void onStop() {
    }
}
