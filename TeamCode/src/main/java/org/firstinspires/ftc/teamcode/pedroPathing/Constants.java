package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.control.ControlSystem;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.8)
            .forwardZeroPowerAcceleration(-27.7409466667)//-38.493816196, -31.758151072886, -27.22610409475
            .lateralZeroPowerAcceleration(-73.5227566667)//-84.513153348, -99.67502078995, -88.8435980572
            .translationalPIDFCoefficients(new PIDFCoefficients(0.125, 0, 0.015, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.045, 0, 0.0008,0.0, 0.0))
            .centripetalScaling(0.00075);
    public static MecanumConstants driverConstants = new MecanumConstants()
            .maxPower(1.0)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(69.96105)
            .yVelocity(58.47987);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

//    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
//            .forwardTicksToInches(0.002004585898)
//            .strafeTicksToInches(0.00199865671847) //-0.00199865671847
//            .turnTicksToInches(0.001985945224)
//            .leftPodY(4.3125)
//            .rightPodY(-4.3125)
//            .strafePodX(1.125)
//            .leftEncoder_HardwareMapName("leftRear")
//            .rightEncoder_HardwareMapName("leftFront")
//            .strafeEncoder_HardwareMapName("rightRear")
//            .leftEncoderDirection(Encoder.REVERSE)
//            .rightEncoderDirection(Encoder.REVERSE)
//            .strafeEncoderDirection(Encoder.FORWARD)
//            .IMU_HardwareMapName("imu")
//            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(110.8) // /25.4
            .strafePodX(-34.2) // /25.4
            .distanceUnit(DistanceUnit.MM)
//            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
//            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .customEncoderResolution(34.31) //34.31
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driverConstants)
                .pinpointLocalizer(localizerConstants)
//                .threeWheelIMULocalizer(localizerConstants)
                .build();

    }

}
