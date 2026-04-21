package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12)
            .forwardZeroPowerAcceleration(-30.9947)//-29.94639, -34.12680, -28.91091
            .lateralZeroPowerAcceleration(-66.40157)//-65.89225, -61.68155, -71.63092
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.015, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.0, 0, 0.05, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04, 0, 0.0008,0, 0.0))
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
            .xVelocity(71.76357)
            .yVelocity(43.71089);

    public static PathConstraints pathConstraints = new PathConstraints(0.975, 100, 0.625, 0.75);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(110.8)
            .strafePodX(-34.2)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .customEncoderResolution(34.31)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driverConstants)
                .pinpointLocalizer(localizerConstants)
                .build();

    }

}
