package org.firstinspires.ftc.teamcode.teleop;

import android.print.PageRange;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.vision.logi;
import org.firstinspires.ftc.teamcode.teleop.fsm.FSM;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class LCsTele extends OpMode {

    private GamepadMapping controls;
    private FSM fsm;
    private Robot robot;
    private logi cam;

    private Follower follower;
    private boolean turning;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    public static Pose pose;


    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        fsm = new FSM(hardwareMap, controls, robot);
        cam = new logi(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(new Pose(18, 118, Math.toRadians(144)));
        pose = follower.getPose();
        follower.setStartingPose(pose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


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

        double atBearing = Math.toRadians(cam.getATangle());
        double atHeadingError = angleWrap(atBearing);

//        pathChain = () -> follower.pathBuilder()
//                .addPath(new Path(new BezierLine(
//                        follower::getPose,
//                        //TODO may have to add 0.01 or any small number so it actually turns and no error
//                        new Pose(
//                                follower.getPose().getX() + 3,
//                                follower.getPose().getY() + 3
//                        )
//                )))
//                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
//                        //TODO may have to make it SUBSTRACT heading error!!!
//                        follower::getHeading, Math.toRadians(follower.getHeading() + atHeadingError), 0.96)
//                )
//                .build();
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierPoint(new Pose(
                                follower.getPose().getX() + 1,
                                follower.getPose().getY() + 1
                        ))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(
                        //TODO may have to make it SUBSTRACT heading error!!!
                        follower::getHeading, Math.toRadians(follower.getHeading() - atHeadingError), 0.96)
                )
                .build();


        boolean controllerBusy =
                Math.abs( gamepad1.left_stick_x) > 0.05
                        || Math.abs(gamepad1.left_stick_y) > 0.05
                        || Math.abs(gamepad1.right_stick_x) > 0.05;

        if (gamepad1.xWasPressed()) {
            follower.setPose(new Pose(pose.getX(), pose.getY(), Math.toRadians(90)));
        }


        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x * 0.65;

        follower.setTeleOpDrive(forward, strafe, rotate, true);

        if (gamepad1.dpad_down) {
            follower.followPath(pathChain.get());
            turning = true;
        }

        if (turning && (controllerBusy || !follower.isBusy())) {
            follower.startTeleopDrive();
            turning = false;
        }

        telemetry.addData("pose", pose);
        telemetry.addData("Heading", heading);
        telemetry.addData("AT angle", cam.getATangle());
        telemetry.addData("AT dist", cam.getATdist());
        telemetry.addLine("--------------------------------");
        telemetry.addData("Auto Align", turning);
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
