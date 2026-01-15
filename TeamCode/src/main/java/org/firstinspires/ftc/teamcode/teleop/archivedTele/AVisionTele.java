package org.firstinspires.ftc.teamcode.teleop.archivedTele;

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
import org.firstinspires.ftc.teamcode.pedroPathing.DrawingNew;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.fsm.FSM;

import java.util.function.Supplier;

@Config
@TeleOp
public class AVisionTele extends OpMode {

    private GamepadMapping controls;
    private FSM fsm;
    private Robot robot;
    //private logi cam;

    private Follower follower;
    double oldTime = 0;
    private boolean autoTurnVision = false;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    //auto align
    public static double tolerance = 0.02;
    public static double turn_kP = 0.2;
    public static double minTurnPower = 0.08;
    public static double toMiniTolerance = 0.02;
    public int count = 0;
    public static double odoDistance;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        fsm = new FSM(hardwareMap, controls, robot);

        follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));
        follower.setStartingPose(PoseStorage.startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(
                        follower::getPose,
                        new Pose(105.4, 33.4)
                )))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        follower::getHeading, Math.toRadians(90), 0.96)
                )
                .build();
        DrawingNew.init();
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
        DrawingNew.drawDebug(follower);

        Pose pose = follower.getPose();
        double heading = pose.getHeading();

        double newTime = getRuntime();
        double loopTime = newTime - oldTime;
        oldTime = newTime;
        telemetry.addData("Looptime", loopTime);

        double atBearing = Math.toRadians(Robot.cam.getATangle());
        double atHeadingError = angleWrap(atBearing);
        boolean visionTurnFinished = Math.abs(atHeadingError) < tolerance;

        boolean controllerBusy =
                Math.abs( gamepad1.left_stick_x) > 0.05
            || Math.abs(gamepad1.left_stick_y) > 0.05
            || Math.abs(gamepad1.right_stick_x) > 0.05;

//        if (gamepad1.xWasPressed()) {
//            follower.setPose(new Pose(pose.getX(), pose.getY(), Math.toRadians(90)));
//        }

        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate;

        if (autoTurnVision) {
            forward = 0;
            strafe = 0;

            double Kp = turn_kP;
            rotate = atHeadingError * Kp;

            double minPower = minTurnPower;
            //TODO - TRY NOT EVEN HAVING THIS
            if (Math.abs(rotate) < minPower && Math.abs(atHeadingError) > toMiniTolerance) {
                rotate = Math.signum(rotate) * minPower;
            }

        } else {
            rotate = -gamepad1.right_stick_x * 0.55;
        }

        follower.setTeleOpDrive(forward, strafe, rotate, true);

        if (gamepad1.startWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        if (automatedDrive && (controllerBusy || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (gamepad1.a && !autoTurnVision) {
            autoTurnVision = true;
        }

        if ((autoTurnVision && visionTurnFinished) || controllerBusy) {
            autoTurnVision = false;
        }


//        if (fsm.state == FSM.FSMStates.SHOOT_BACK) {
//            count = 1;
//            follower.holdPoint(pose);
//        } else if ((fsm.state == FSM.FSMStates.BASE_STATE && count == 1) || controllerBusy) {
//            follower.startTeleopDrive();
//            count = 0;
//        }


//        if (gamepad1.x) {
//            follower.holdPoint(pose);
//        } if (gamepad1.b) {
//            follower.breakFollowing();
//            follower.setTeleOpDrive(forward,strafe,heading);
//            follower.startTeleopDrive();
//        }

        telemetry.addData("pose", pose.toString());
        telemetry.addData("Odo Distance", odoDistance);
        telemetry.addData("Shooter Vel", robot.shooter.outtake1.getVelocity());
        telemetry.addData("Hood Pos", robot.shooter.variableHood.getPosition());

        telemetry.addData("Heading", heading);
        telemetry.addData("AT angle", Robot.cam.getATangle());
        telemetry.addData("AT dist",  Robot.cam.getATdist());
        telemetry.addData("Artifact Travel distance", Robot.cam.getATdist());
        telemetry.addData("last velo",  fsm.lastVelo);

        telemetry.addLine("--------------------------------");
        telemetry.addData("Vision AutoTurn", autoTurnVision);
        telemetry.addData("AutoPark", automatedDrive);
        telemetry.addData("Tele Drive", follower.getTeleopDrive());
        telemetry.addData("graph AT heading error", atHeadingError);


    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
