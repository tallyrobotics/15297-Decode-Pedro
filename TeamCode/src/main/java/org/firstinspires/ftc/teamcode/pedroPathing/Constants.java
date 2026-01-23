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
            .mass(13.15)
            .forwardZeroPowerAcceleration(-32.49269045)//-38.493816196, -31.758151072886, -27.22610409475
            .lateralZeroPowerAcceleration(-91.010590732)//-84.513153348, -99.67502078995, -88.8435980572
            .translationalPIDFCoefficients(new PIDFCoefficients(0.125, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.045, 0, 0.0,0.0, 0.0))
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
            .xVelocity(72.761491057)
            .yVelocity(51.1936930945);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.002004585898)
            .strafeTicksToInches(0.00199865671847) //-0.00199865671847
            .turnTicksToInches(0.001985945224)
            .leftPodY(4.3125)
            .rightPodY(-4.3125)
            .strafePodX(1.125)
            .leftEncoder_HardwareMapName("leftRear")
            .rightEncoder_HardwareMapName("leftFront")
            .strafeEncoder_HardwareMapName("rightRear")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));

//    public static PinpointConstants localizerConstants = new PinpointConstants()
//            .forwardPodY(-109.538/25.4) // -109.538 mm / 2.54 inches
//            .strafePodX(89.048/25.4) //need to change  89.048 mm / 2.54 inches
//            .distanceUnit(DistanceUnit.INCH)
//            .hardwareMapName("pinpoint")
//            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED) //need to change
//            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD); //need to change


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driverConstants)
//                .pinpointLocalizer(localizerConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();

    }

}
