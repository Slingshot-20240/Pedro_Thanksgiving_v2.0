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

        if (gamepad1.dpad_down) {
            //TODO - try true, and positive error degrees
            follower.turnDegrees(-atHeadingError,false);
            follower.holdPoint(pose);
        }

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

        if (controllerBusy || !follower.isBusy()) {
            follower.resumePathFollowing();
            follower.breakFollowing();
            follower.startTeleopDrive();
        }

        telemetry.addData("pose", pose);
        telemetry.addData("Heading", heading);
        //TODO - change offset hard coded values of goal
        telemetry.addData("Odo Distance", pose.distanceFrom(new Pose(137,137)));
        telemetry.addLine("--------------------------------");
        telemetry.addData("AT angle", cam.getATangle());
        telemetry.addData("AT dist", cam.getATdist());
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
