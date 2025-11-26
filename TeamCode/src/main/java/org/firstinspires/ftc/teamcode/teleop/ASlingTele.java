package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

import org.firstinspires.ftc.robotcore.external.Supplier;

import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.fsm.FSM;

@TeleOp
public class ASlingTele extends OpMode {
    private GamepadMapping controls;
    private FSM fsm;
    private Robot robot;

    private Follower follower;
    private LocalizationMath localizationMath;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;



    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        fsm = new FSM(hardwareMap, controls, robot);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Pedro Localization
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }


    @Override
    public void start() {
        // run once when we start
        robot.hardwareSoftReset();
        follower.startTeleopDrive(true);
    }


    @Override
    public void loop() {
        fsm.update();
        follower.update();
        //Potential error with both?
        telemetryM.update();
        telemetryM.update(telemetry);
        Pose pose = follower.getPose();



//----------------------------Telemetry and Dash Field Overlay----------------------------\\

        telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());

        draw();

        telemetry.addData("-----------------------", "");

        //Limelight telemetry
        telemetry.addData("limelight angle", Math.toDegrees(robot.limelight.getAngle()));
        telemetry.addData("limelight nav", (robot.limelight.getLastNav()));
        telemetry.addData("limelight obelisk", (robot.limelight.getObelisk().order));

        telemetry.update();


        //Dash field overlay and draw bot with Canvas
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

//----------------------------Localization Math----------------------------\\


        double dist = LocalizationMath.getRedDistance(pose);
        double err  = LocalizationMath.getRedHeadingError(pose);

        telemetry.addData("Red Distance (in)", dist);
        telemetry.addData("Red Heading Error (deg)", err);
//----------------------------Drive Controls----------------------------\\

    //Robo Centric
        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.startWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        //PRESS B TO ESCAPE FROM PATH
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive(true);
            automatedDrive = false;
        }



    }
}

