package org.firstinspires.ftc.teamcode.NextFTC.misc;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Pathing Test", group = "Autonomous")
@Configurable // Panels
public class PathingTest extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(126, 118, Math.toRadians(36)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain scorePreloads;
        public PathChain prepareSet2;
        public PathChain grabSet2;
        public PathChain prepareGate;
        public PathChain hitGate;
        public PathChain scoreSet2;
        public PathChain prepareSet3;
        public PathChain grabSet3;
        public PathChain scoreSet3;
        public PathChain prepareSet4;
        public PathChain grabSet4;
        public PathChain scoreSet4;

        public Paths(Follower follower) {
            scorePreloads = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125.500, 118.300), new Pose(97,97))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(45))
                    .build();

            prepareSet2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(97,97), new Pose(96.500, 83.800))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            grabSet2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.500, 83.800), new Pose(124, 83.800))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            prepareGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(124, 83.800), new Pose(123.000, 70.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            hitGate = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(123.000, 70.000), new Pose(125, 70.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            scoreSet2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125, 70.000), new Pose(97,97))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                    .build();

            prepareSet3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(97,97), new Pose(96.500, 58.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            grabSet3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.500, 58), new Pose(131, 59.5))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            scoreSet3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(131, 59.5),
                                    new Pose(91.262, 56.240),
                                    new Pose(95.176, 82.815),
                                    new Pose(97,97)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            prepareSet4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(97,97), new Pose(96.000, 38))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            grabSet4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(96.000, 38), new Pose(130, 37))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            scoreSet4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(130, 37), new Pose(90.000, 110.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(29))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.scorePreloads);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.prepareSet2);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.grabSet2);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.prepareGate);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.hitGate);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.scoreSet2);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.prepareSet3);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.grabSet3);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.scoreSet3);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.prepareSet4);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.grabSet4);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(paths.scoreSet4);
                    setPathState(-1); // stop
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
    }
}
