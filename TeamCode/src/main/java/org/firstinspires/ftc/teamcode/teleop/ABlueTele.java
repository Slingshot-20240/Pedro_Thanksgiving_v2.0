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
import org.firstinspires.ftc.teamcode.teleop.fsm.FSM;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class ABlueTele extends OpMode {

    private GamepadMapping controls;
    private FSM fsm;
    private Robot robot;
    private Follower follower;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;

    private boolean autoTurn = false;
    private double goalHeading = 0;

    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        fsm = new FSM(hardwareMap, controls, robot);

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(18, 118, Math.toRadians(144)));
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
                                0.9
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

        // Updates fsm, follower, and telemetry
        fsm.update();
        follower.update();
        telemetryM.update();
        telemetry.update();

        // Robot pose
        Pose pose = follower.getPose();
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        // Relocalize robot
        if (gamepad1.xWasPressed()) {
            follower.setPose(new Pose(18, 118, Math.toRadians(144)));
        }

        // -------------------------------
        //          HEADING LOCK
        // -------------------------------

        if (gamepad1.a && !autoTurn) {
            autoTurn = true;

            // BLUE GOAL TARGET (-144, 144)
            goalHeading = Math.atan2(144 - y, -144 - x);
        }

        double headingError = angleWrap(goalHeading - heading);
        boolean turnFinished = Math.abs(headingError) < Math.toRadians(2);

        if (autoTurn && turnFinished) {
            follower.holdPoint(follower.getPose());
            autoTurn = false;
        }

        // Driving inputs
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate;

        if (autoTurn) {
            forward = 0;
            strafe = 0;

            double Kp = 0.8;
            rotate = headingError * Kp;

        } else {
            rotate = -gamepad1.right_stick_x;
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x * 0.6,
                true // Robot Centric
        );


        // Telemetry
        telemetry.addData("pose", pose);
        telemetry.addData("Heading", heading);
        telemetry.addLine("--------------------------------");
        telemetry.addData("autoTurn", autoTurn);
        telemetry.addData("Automated Drive", automatedDrive);
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
