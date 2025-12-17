package org.firstinspires.ftc.teamcode.teleop.fsm.practice;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;

public class PracticeFSM {

    private States state = States.BASE_STATE;
    private Gamepad gamepad1, gamepad2;
    Robot robot;
    public PracticeFSM(HardwareMap hardwareMap, GamepadMapping gamepad, Gamepad gamepad1, Gamepad gamepad2) {
        robot = new Robot(hardwareMap, gamepad);
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void update() {

        robot.drivetrain.update();

        switch(state) {
            case BASE_STATE:
                // intake on
                if(gamepad1.a) {
                    // shoot
                    state = States.SHOOTING;
                }
                if(gamepad1.b) {
                    state = States.INTAKING;
                }
                break;

            case SHOOTING:
                // intake off
                // activate shooter
                if(gamepad1.a == false) {
                    state = States.BASE_STATE;
                }
                break;

            case INTAKING:
                // intake ball
                if(gamepad1.b == false) {

                }
                break;

            case TRANSFER:

                break;
        }
    }


    public enum States {
        BASE_STATE,
        SHOOTING,
        INTAKING,
        TRANSFER
    }

}
