package org.firstinspires.ftc.teamcode.pres_code;

import com.acmerobotics.dashboard.config.Config;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.teleop.fsm.FSM;

@Config
@TeleOp
public class AutoAlignAT extends OpMode {

    private GamepadMapping controls;
    private Robot robot;
    private Shooter shooter;
    private Follower follower;

    private boolean autoTurnVision = false;


    //auto align
    public static double tolerance = 0.008;
    public static double turn_kP = 0.1;
    public static double minTurnPower = 0.08;
    public static double toMiniTolerance = 0.01;


    @Override
    public void init() {
        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        shooter = new Shooter(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.startingPose);
        follower.update();

    }

    @Override
    public void start() {
        robot.hardwareSoftReset();
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.update();

        //AUTO HOOD
        double distance = Robot.cam.getTargetArtifactTravelDistanceX();
        double targetHoodPos;
        //TODO - TUNE THIS OFFSET VALUE
        if (Robot.cam.getATdist() < 54) {
            targetHoodPos = robot.shooter.calculateHoodPos(distance) + 0.2;
        } else {
            targetHoodPos = robot.shooter.calculateHoodPos(distance) + 0.1;
        }
        if (Robot.cam.getTargetArtifactTravelDistanceX() == 22) {
            robot.shooter.setHoodAngle(shooter.variableHood.getPosition());
        } else {
            robot.shooter.setHoodAngle(targetHoodPos);
        }

        //LED
        if (Robot.cam.getATdist() != 0) {
            robot.ledBoard0.setState(true);
            robot.ledBoard1.setState(true);
        } else {
            robot.ledBoard0.setState(false);
            robot.ledBoard1.setState(true);
        }


        //AUTO ALIGN
        double atBearing = Math.toRadians(Robot.cam.getATangle());
        double atHeadingError = angleWrap(atBearing);
        boolean visionTurnFinished = Math.abs(atHeadingError) < tolerance;

        boolean controllerBusy =
                Math.abs( gamepad1.left_stick_x) > 0.05
                        || Math.abs(gamepad1.left_stick_y) > 0.05
                        || Math.abs(gamepad1.right_stick_x) > 0.05;



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
            rotate = -gamepad1.right_stick_x * 0.4;
        }

        //TODO - if you don't want drive controls to be activated, just change forward and strafe to be 0
        follower.setTeleOpDrive(0, 0, rotate, true);


        if (gamepad1.a && !autoTurnVision) {
            autoTurnVision = true;
        }

        if ((autoTurnVision && visionTurnFinished) || controllerBusy) {
            autoTurnVision = false;
        }

        telemetry.addData("AT angle", Robot.cam.getATangle());
        telemetry.addData("AT dist",  Robot.cam.getATdist());

        telemetry.addLine("--------------------------------");
        telemetry.addData("Vision AutoTurn", autoTurnVision);

    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
