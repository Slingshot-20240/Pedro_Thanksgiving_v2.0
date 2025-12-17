package org.firstinspires.ftc.teamcode.teleop.archivedTele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.teleop.fsm.FSM;

@TeleOp
public class FSMTest extends OpMode {
    private Robot robot;
    private FSM fsm;
    private GamepadMapping gamepad;


    @Override
    public void init() {
        gamepad = new GamepadMapping(gamepad1,gamepad2);
        robot = new Robot(hardwareMap, gamepad);
        fsm = new FSM(hardwareMap, gamepad, robot);
        robot.shooter.variableHood.setPosition(0);
    }

    @Override
    public void loop() {
//        telemetry.addData("loopTime", fsm.loopTime.milliseconds() - fsm.startTime);
        telemetry.addData("velo", fsm.robot.shooter.outtake1.getVelocity());
        fsm.update();
    }
}
