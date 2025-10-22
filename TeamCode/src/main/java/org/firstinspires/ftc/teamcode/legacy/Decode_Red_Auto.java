package org.firstinspires.ftc.teamcode.legacy;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.opencv.core.Point;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;


@Autonomous(name = "Decode Red Auto")
public class Decode_Red_Auto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opModeTimer;
    private int pathState;
    private final Pose startPose = new Pose(0, 0, 0);
    private int counter = 0;


    private Path scorePreload;
    private PathChain line1, line2, line3, line4;

    public void buildPaths() {

               line1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, new Pose(18, 0,0 )))
                .setConstantHeadingInterpolation(startPose.getHeading())
                       .build();
               line2 = follower.pathBuilder()
                       .addPath(new BezierLine(new Pose(18,0,0), new Pose(18,18,0)))
                       .setConstantHeadingInterpolation(startPose.getHeading())
                       .build();
               line3 = follower.pathBuilder()
                       .addPath(new BezierLine(new Pose(18,18,0), new Pose(0,18,0))).
                       setConstantHeadingInterpolation(startPose.getHeading())
                       .build();
               line4 = follower.pathBuilder()
                       .addPath(new BezierLine(new Pose(0,18,0), new Pose(0,0,0))).
                       setConstantHeadingInterpolation(startPose.getHeading())
                       .build();

    }

    public Command doAuto() {
        return new SequentialGroup(
                new FollowPath(line1, true, 0.5),
                new Delay(1),
                new FollowPath(line2, true, 0.5),
                new Delay(1),
                new FollowPath(line3, true, 0.5),
                new Delay(1),
                new FollowPath(line4, true, 0.5)
        );


    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        doAuto();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}
