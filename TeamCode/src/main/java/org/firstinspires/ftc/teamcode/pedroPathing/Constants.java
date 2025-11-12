package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(23.2)
            .forwardZeroPowerAcceleration(-25.9588070151)
            .lateralZeroPowerAcceleration(-53.1563064896)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.01, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.5, 0, 0.00001, 0.6, 0.01))
            .centripetalScaling(0.0005);
    public static MecanumConstants driverConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(76.2576928996)
            .yVelocity(60.6159339653);
    public static PathConstraints pathConstraints = new PathConstraints(0.995, 100, 3, 1);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .leftEncoder_HardwareMapName("leftFront")
            .rightEncoder_HardwareMapName("rightFront")
            .strafeEncoder_HardwareMapName("rightRear")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP))
            .leftPodY((109.53798/25.4))
            .rightPodY(-(109.53798/25.4))
            .strafePodX(-35.5/25.4)
            .forwardTicksToInches(0.0019607091)
            .strafeTicksToInches(0.0019603589)
            .turnTicksToInches(0.0020178409346387613);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driverConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }
}
