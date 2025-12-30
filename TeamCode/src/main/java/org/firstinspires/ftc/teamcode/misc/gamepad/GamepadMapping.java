package org.firstinspires.ftc.teamcode.misc.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadMapping {
    // GAMEPADS
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    // DRIVETRAIN - LM1
    // --------------
    public static double drive = 0.0;
    public static double strafe = 0.0;
    public static double turn = 0.0;

    // INTAKE
    public Toggle intake;

    // SHOOTER
    public Toggle shootBack;
    public Toggle shootFront;

    // MISC
    public Toggle transfer; // LM1
    public Toggle outtake; // LM1

    public GamepadMapping(Gamepad gamepad1, Gamepad gamepad2) {
        // GAMEPADS
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        // INTAKE
        intake = new Toggle(false);

        // SHOOTER
        shootBack = new Toggle(false);
        shootFront = new Toggle(false);

        // MISC
        transfer = new Toggle(false);
        outtake = new Toggle(false);
    }

    public void joystickUpdate() {
        drive = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
    }

    public void update() {
        joystickUpdate();
        // INTAKE
        intake.update(gamepad1.right_trigger >= 0.5);

        // SHOOTER
        shootBack.update(gamepad1.left_trigger >= 0.5);
        shootFront.update(gamepad1.left_bumper);

        // TRANSFER
        transfer.update(gamepad1.left_bumper);

        // OUTTAKE
        outtake.update(gamepad1.y);
    }

    public void resetMultipleControls(Toggle... toggles) {
        for (Toggle toggle : toggles) {
            toggle.set(false);
        }
    }
}

