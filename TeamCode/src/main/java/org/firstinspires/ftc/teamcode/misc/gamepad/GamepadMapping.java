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
    public Toggle pidShoot;
    public Toggle shootBack;
    public Toggle shootFront;
    public Toggle apriltagLock;

    // MISC
    // public Toggle failSafeReset;
    public Toggle transfer; // LM1
    public Toggle outtake; // LM1
    public Toggle farLock;

    public GamepadMapping(Gamepad gamepad1, Gamepad gamepad2) {
        // GAMEPADS
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        // INTAKE
        intake = new Toggle(false);

        // SHOOTER
         shootBack = new Toggle(false);
         shootFront = new Toggle(false);
        // pidShoot = new Toggle(false);
        apriltagLock = new Toggle(false);

        // MISC
        // failSafeReset = new Toggle(false);
        transfer = new Toggle(false);
        outtake = new Toggle(false);
        farLock = new Toggle(false);
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
        //pidShoot.update(gamepad2.right_bumper);
        apriltagLock.update(gamepad1.start);

        // failSafeReset.update(gamepad1.dpad_down);

        // TRANSFER
        transfer.update(gamepad1.dpad_up);

        // OUTTAKE
        outtake.update(gamepad1.y);

        farLock.update(gamepad1.dpad_down);
    }

    public void resetMultipleControls(Toggle... toggles) {
        for (Toggle toggle : toggles) {
            toggle.set(false);
        }
    }
}

