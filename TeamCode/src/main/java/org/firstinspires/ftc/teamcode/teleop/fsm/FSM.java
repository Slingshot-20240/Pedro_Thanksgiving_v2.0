package org.firstinspires.ftc.teamcode.teleop.fsm;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.misc.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.park.Park;
import org.firstinspires.ftc.teamcode.subsystems.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Transfer;

public class FSM {
    // GENERAL ROBOT STATES + CLASSES
    public Robot robot;
    public FSMStates state = FSMStates.BASE_STATE;
    public ControlType type = ControlType.PID_CONTROL;
    private final GamepadMapping gamepad;

    // SUBSYSTEMS
    private final Intake intake;
    private final Transfer transfer;
    private final Shooter shooter;
    private final Park park;

    public double lastVelo = 800;

    private ControlType savedType;


    // Everyone ignore this horrendous OOP
    // private double odoDistance = AVisionTele.odoDistance;

    public FSM(HardwareMap hardwareMap, GamepadMapping gamepad, Robot robot) {
        this.robot = robot;
        this.gamepad = robot.controls;

        intake = robot.intake;
        transfer = robot.transfer;
        park = robot.park;
        shooter = robot.shooter;
    }

    public void update() {
        gamepad.update();

        switch (state) {
            case BASE_STATE:

                if (type == ControlType.HARDCODED_CONTROL) {
                    shooter.shootFromFront();
                    shooter.hoodToFront();
                }

                // TODO: test, should work
//                if (gamepad.switchMode.value()) {
//                    if(type == ControlType.HARDCODED_CONTROL) {
//                        savedType = ControlType.HARDCODED_CONTROL;
//                        type = ControlType.PID_CONTROL;
//                    } else {
//                        savedType = ControlType.PID_CONTROL;
//                        type = ControlType.HARDCODED_CONTROL;
//                    }
//                } else {
//                    type = savedType;
//                }

                if (gamepad.park.value()) {
                    state = FSMStates.PARK;
                }

                transfer.hotDog();

                if (gamepad.outtake.locked()) {
                    state = FSMStates.OUTTAKING;
                }

                if (gamepad.shootBack.locked() && type == ControlType.HARDCODED_CONTROL) {
                    state = FSMStates.SHOOT_BACK;
                }

                if (gamepad.shootFront.locked() && type == ControlType.HARDCODED_CONTROL) {
                    state = FSMStates.SHOOT_FRONT;
                }

                if (gamepad.intake.locked()) {
                    intake.intakeOn();
                } else {
                    intake.intakeOff();
                }

                if (gamepad.transfer.locked() && type == ControlType.PID_CONTROL) {
                    state = FSMStates.PID_SHOOT;
                }

                if (type == ControlType.PID_CONTROL) {
                    double distance = Robot.cam.getTargetArtifactTravelDistanceX();

                    double targetVelocity = robot.shooter.calculateShooterRPM(distance) + 100;

                    double targetHoodPos;
                    //TODO - TUNE THIS OFFSET VALUE
                    if (Robot.cam.getATdist() < 54) {
                        targetHoodPos = robot.shooter.calculateHoodPos(distance) + 0.2;
                    } else {
                        targetHoodPos = robot.shooter.calculateHoodPos(distance) + 0.1;
                    }

                    if (Robot.cam.getATdist() != 0) {
                        lastVelo = targetVelocity;
                    }

//                    if (targetVelocity >= robot.shooter.outtake1.getVelocity() - 50 || targetVelocity <= robot.shooter.outtake1.getVelocity() + 50) {


//                    } else {
//                        robot.ledBoard0.setState(false);
//                        robot.ledBoard1.setState(false);
//                    }

                    // This should prevent the shooter from changing hood pos if it can't see the AprilTag (so if it cuts out it's fine)
                    if (Robot.cam.getTargetArtifactTravelDistanceX() == 22) {
                        robot.shooter.setHoodAngle(shooter.variableHood.getPosition());
                        robot.shooter.setShooterVelocity(-lastVelo);
                    } else {
                        robot.shooter.setHoodAngle(targetHoodPos);
                        robot.shooter.setShooterVelocity(-targetVelocity);
                    }

                    if (Robot.cam.getTargetArtifactTravelDistanceX() != 22) {
                        robot.ledBoard0.setState(true);
                        robot.ledBoard1.setState(true);
                    } else {
                        robot.ledBoard0.setState(false);
                        robot.ledBoard1.setState(true);
                    }
                }

                break;
            case OUTTAKING:
                intake.intakeReverse();
                if (!gamepad.outtake.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.transfer, gamepad.shootFront, gamepad.shootBack);
                }
                break;

            case SHOOT_BACK:
                shooter.shootFromBack();
                shooter.hoodToBack();
                intake.intakeOn();

                if (shooter.outtake1.getVelocity() <= Shooter.outtakeVels.HARDCODED_SHOOT_BACK.getOuttakeVel() + 50) {
                    transfer.transferOn();
                }

                if (!gamepad.shootBack.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.shootBack, gamepad.shootFront, gamepad.transfer);
                }
                break;
            case SHOOT_FRONT:
                intake.intakeOn();
                shooter.shootFromFront();

                transfer.transferOn();

                if (!gamepad.shootFront.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.shootBack, gamepad.shootFront, gamepad.transfer);
                }
                break;

            case PID_SHOOT:
                intake.intakeOn();

                transfer.transferOn();

                if (!gamepad.transfer.locked()) {
                    state = FSMStates.BASE_STATE;
                    gamepad.resetMultipleControls(gamepad.transfer, gamepad.outtake);
                }

            case PARK:
                park.tilt();
                if (!gamepad.park.value()) {
                    state = FSMStates.BASE_STATE;
                    park.unTilt();
//                    gamepad.resetMultipleControls(gamepad.park, gamepad.outtake, gamepad.transfer);
                }

        }
    }

    public void setState(FSMStates newState) {
        state = newState;
    }

    public FSMStates getState() {
        return state;
    }

    public void setControlType(ControlType newCType) {
        type = newCType;
    }

    public ControlType getControlType() {
        return type;
    }

    public enum FSMStates {
        BASE_STATE,
        SHOOT_FRONT,
        SHOOT_BACK,
        OUTTAKING,
        PID_SHOOT,
        PARK
    }

    public enum ControlType {
        HARDCODED_CONTROL,
        PID_CONTROL
    }

}