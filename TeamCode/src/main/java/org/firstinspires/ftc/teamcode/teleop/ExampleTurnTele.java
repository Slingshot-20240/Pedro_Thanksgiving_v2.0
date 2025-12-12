package org.firstinspires.ftc.teamcode.teleop;
import com.bylazar.configurables.annotations.Configurable;
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
import org.firstinspires.ftc.teamcode.subsystems.vision.logi;
import org.firstinspires.ftc.teamcode.teleop.fsm.FSM;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class ExampleTurnTele extends OpMode {

    private GamepadMapping controls;
    private FSM fsm;
    private Robot robot;
    private logi cam;
    private Supplier<PathChain> pathChain;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this

    private TelemetryManager telemetryM;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        fsm = new FSM(hardwareMap, controls, robot);
        cam = new logi(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {

        //Call this once per loop
        follower.update();
        telemetryM.update();
        fsm.update();
        Pose pose = follower.getPose();
        double heading = pose.getHeading();

        double atBearing = Math.toRadians(cam.getATangle());
        double atHeadingError = angleWrap(atBearing);

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(pose.getX() + 1, pose.getY() + 1))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, pose.getHeading() + atHeadingError, 0.9))
                .build();

        boolean controllerBusy =
                Math.abs( gamepad1.left_stick_x) > 0.05
                        || Math.abs(gamepad1.left_stick_y) > 0.05
                        || Math.abs(gamepad1.right_stick_x) > 0.05;



        follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x ,
                    -gamepad1.right_stick_x * 0.6,
                    true // Robot Centric
            );

        //Automated PathFollowing


        //Stop automated following if the follower is done
        if (controllerBusy || gamepad1.bWasPressed() || !follower.isBusy()) {
            follower.startTeleopDrive();
        }
        if (gamepad1.a) {
            follower.followPath(pathChain.get());

        }



        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("atHeadingError", atHeadingError);
        telemetry.addData("AT angle", cam.getATangle());


    }
    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}