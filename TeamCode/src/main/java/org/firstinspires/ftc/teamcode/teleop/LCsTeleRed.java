package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.fsm.FSM;

import java.util.function.Supplier;

@Config
@TeleOp
public class LCsTeleRed extends OpMode {

    private GamepadMapping controls;
    private FSM fsm;
    private Robot robot;

    private Follower follower;

    private boolean autoTurnVision = false;
    private boolean autoTurnOdo = false;

    private boolean automatedDrive;
    private Supplier<PathChain> gateRightClose;
    private Supplier<PathChain> gateBackClose;
    private Supplier<PathChain> gateLeftClose;
    //no gate front, so make it default to right

    private Supplier<PathChain> gateBackFar;



    private TelemetryManager telemetryM;
    public static double odoDistance;


    public static double tolerance = 0.02;

    // Vision tuning
    public static double visionTurn_kP = 0.1;
    public static double visionMinTurnPower = 0.08;
    public static double visionMiniTolerance = 0.02;

    // ODO target
    public static double GOAL_X = 140;
    public static double GOAL_Y = 140;

    // ODO tuning
    public static double odoTurn_kP = 0.3;
    public static double odoMinTurnPower = 0.08;

    public static Pose gatePose = new Pose(129,70);

    @Override
    public void init() {

        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        fsm = new FSM(hardwareMap, controls, robot);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.startingPose);
        //follower.setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();



        //Gate code
        gateBackFar = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(
                        follower::getPose,
                        //TODO - tune control point
                        new Pose(112,70)
                )))
                .setHeadingInterpolation(
                        HeadingInterpolator.tangent.reverse()
                )

                .addPath(new Path(new BezierLine(
                        follower::getPose,
                        gatePose
                )))
                .setHeadingInterpolation(
                        HeadingInterpolator.constant(Math.toRadians(180))
                )
                .build();

        gateRightClose = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(
                        follower::getPose,
                        gatePose
                )))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(90),
                                0.8 //TODO - change and/or tune end t value
                        )
                )
                .build();

        gateBackClose = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(
                        follower::getPose,
                        gatePose
                )))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(180),
                                0.8 //TODO - change and/or tune end t value
                        )
                )
                .build();

        gateLeftClose = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(
                        follower::getPose,
                        gatePose
                )))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(270),
                                0.8 //TODO - change and/or tune end t value
                        )
                )
                .build();
    }

    @Override
    public void start() {
        robot.hardwareSoftReset();
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {

        fsm.update();
        follower.update();
        telemetryM.update();
        telemetry.update();

        Pose pose = follower.getPose();
        double heading = pose.getHeading();
        double headingDeg = Math.toDegrees(heading);
        odoDistance = pose.distanceFrom(new Pose(140,140));


        boolean controllerBusy =
                Math.abs(gamepad1.left_stick_x) > 0.05 ||
                        Math.abs(gamepad1.left_stick_y) > 0.05 ||
                        Math.abs(gamepad1.right_stick_x) > 0.05;

        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate;



        //------------- error calculation -------------\\
        // Vision error
        double visionBearing = Math.toRadians(Robot.cam.getATangle());
        double visionHeadingError = angleWrap(visionBearing);
        boolean visionTurnFinished =
                Math.abs(visionHeadingError) < tolerance;

        // ODO error
        double dx = GOAL_X - pose.getX();
        double dy = GOAL_Y - pose.getY();
        double targetHeading = Math.atan2(dy, dx);
        double odoHeadingError = angleWrap(targetHeading - heading);
        boolean odoTurnFinished =
                Math.abs(odoHeadingError) < tolerance;

        //------------- rotate logic -------------\\

        if (autoTurnVision || autoTurnOdo) {

            forward = 0;
            strafe = 0;

            double error;
            double kP;
            double minPower;
            double miniTolerance;

            if (autoTurnVision) {
                error = visionHeadingError;
                kP = visionTurn_kP;
                minPower = visionMinTurnPower;
                miniTolerance = visionMiniTolerance;
            } else {
                error = odoHeadingError;
                kP = odoTurn_kP;
                minPower = odoMinTurnPower;
                miniTolerance = tolerance;
            }

            rotate = error * kP;

            if (Math.abs(rotate) < minPower && Math.abs(error) > miniTolerance) {
                rotate = Math.signum(rotate) * minPower;
            }

        } else {
            rotate = -gamepad1.right_stick_x * 0.55;
        }


        follower.setTeleOpDrive(forward, strafe, rotate, true);

        if (gamepad1.x) {
            follower.setPose(new Pose(72,8,Math.toRadians(90)));
        }

        // Auto Gate

        //TODO - try just start boolean instead of startWasPressed

        if (gamepad1.startWasPressed()) {

            // FAR CASE
            //TODO - threshold for going far method
            if (pose.getX() < 100) {
                follower.followPath(gateBackFar.get());
                automatedDrive = true;
            }

            // CLOSE CASE
            else {
                if (headingDeg > 45 && headingDeg < 135) {
                    follower.followPath(gateRightClose.get());
                    telemetry.addLine("Forward");
                }
                else if (headingDeg > 135 && headingDeg < 225) {
                    follower.followPath(gateBackClose.get());
                    telemetry.addLine("Left");

                }
                else if (headingDeg > 225 && headingDeg < 315) {
                    follower.followPath(gateLeftClose.get());
                    telemetry.addLine("Back");

                }
                else { // 315–360 OR 0–45
                    follower.followPath(gateRightClose.get());
                    telemetry.addLine("Right");

                }

                automatedDrive = true;
            }
        }

        if (automatedDrive && (controllerBusy || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }


        /* ---------------- AUTO TURN TOGGLES ---------------- */

        if (gamepad1.a && !autoTurnVision) {
            autoTurnVision = true;
        }

        if (gamepad1.left_trigger > 0 && !autoTurnOdo) {
            autoTurnOdo = true;
        }

        // Turn cancellation

        if ((autoTurnVision && visionTurnFinished) || controllerBusy) {
            autoTurnVision = false;
        }

        if ((autoTurnOdo && odoTurnFinished) || controllerBusy) {
            autoTurnOdo = false;
        }

        // Telemetry

        telemetry.addData("Pose", pose);
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.addLine("---- AUTO TURN ----");
        telemetry.addData("Vision Error (deg)", Math.toDegrees(visionHeadingError));
        telemetry.addData("ODO Error (deg)", Math.toDegrees(odoHeadingError));
        telemetry.addData("AutoTurn Vision", autoTurnVision);
        telemetry.addData("AutoTurn ODO", autoTurnOdo);
        telemetry.addData("AutoPath", automatedDrive);
    }

    @Override
    public void stop() {
        PoseStorage.startingPose = follower.getPose();
        robot.shooter.variableHood.setPosition(0.5);
    }
    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
