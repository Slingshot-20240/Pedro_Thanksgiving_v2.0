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

import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.fsm.FSM;

import java.util.function.Supplier;

@Config
@TeleOp
public class ExampleTurnTele extends OpMode {

    private GamepadMapping controls;
    private FSM fsm;
    private Robot robot;

    private Follower follower;
    private boolean autoTurnVision = false;
    private boolean autoTurnOdo = false;

    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    public static Pose startingPose;

    /* ---------------- AUTO TURN CONSTANTS ---------------- */

    public static double tolerance = 0.02;

    // Vision
    public static double turn_kP = 0.9;
    public static double minTurnPower = 0.08;
    public static double toMiniTolerance = 0.02;

    // ODO
    public static double GOAL_X = 140;
    public static double GOAL_Y = 140;
    public static double odoTurn_kP = 1.7;
    public static double odoMinTurnPower = 0.08;

    @Override
    public void init() {

        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        fsm = new FSM(hardwareMap, controls, robot);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(
                        follower::getPose,
                        new Pose(105.4, 33.4)
                )))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(90),
                                0.96
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

        boolean controllerBusy =
                Math.abs(gamepad1.left_stick_x) > 0.05 ||
                        Math.abs(gamepad1.left_stick_y) > 0.05 ||
                        Math.abs(gamepad1.right_stick_x) > 0.05;

        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate;

        /* ---------------- ODO HEADING ERROR ---------------- */

        double dx = GOAL_X - pose.getX();
        double dy = GOAL_Y - pose.getY();
        double targetHeading = Math.atan2(dy, dx);
        double odoHeadingError = angleWrap(targetHeading - heading);

        /* ---------------- ROTATE LOGIC ---------------- */

        if (autoTurnVision || autoTurnOdo) {

            forward = 0;
            strafe = 0;

            double error;
            double kP;
            double minPower;

            if (autoTurnVision) {
                error = odoHeadingError;      // vision error would go here
                kP = turn_kP;
                minPower = minTurnPower;
            } else {
                error = odoHeadingError;
                kP = odoTurn_kP;
                minPower = odoMinTurnPower;
            }

            rotate = error * kP;

            if (Math.abs(rotate) < minPower && Math.abs(error) > tolerance) {
                rotate = Math.signum(rotate) * minPower;
            }

        } else {
            rotate = -gamepad1.right_stick_x * 0.55;
        }

        follower.setTeleOpDrive(forward, strafe, rotate, true);

        /* ---------------- PATH FOLLOWING ---------------- */

        if (gamepad1.startWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (controllerBusy || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        /* ---------------- AUTO TURN TOGGLES ---------------- */

        if (gamepad1.a && !autoTurnVision) {
            autoTurnVision = true;
        }

        if (gamepad1.dpad_down && !autoTurnOdo) {
            autoTurnOdo = true;
        }

        /* ---------------- AUTO TURN EXIT ---------------- */

        if ((autoTurnVision && Math.abs(odoHeadingError) < tolerance) || controllerBusy) {
            autoTurnVision = false;
        }

        if ((autoTurnOdo && Math.abs(odoHeadingError) < tolerance) || controllerBusy) {
            autoTurnOdo = false;
        }

        /* ---------------- HOLD / BREAK ---------------- */

        if (gamepad1.x) {
            follower.holdPoint(pose);
        }

        if (gamepad1.b) {
            follower.breakFollowing();
            follower.startTeleopDrive();
        }

        /* ---------------- TELEMETRY ---------------- */

        telemetry.addData("Pose", pose);
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.addData("ODO Target Heading (deg)", Math.toDegrees(targetHeading));
        telemetry.addData("ODO Heading Error (deg)", Math.toDegrees(odoHeadingError));
        telemetry.addData("AutoTurn Vision", autoTurnVision);
        telemetry.addData("AutoTurn ODO", autoTurnOdo);
        telemetry.addData("AutoPath", automatedDrive);
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
