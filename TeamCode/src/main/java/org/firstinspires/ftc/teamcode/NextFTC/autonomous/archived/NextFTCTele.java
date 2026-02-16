package org.firstinspires.ftc.teamcode.NextFTC.autonomous.archived;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NextFTC.sequences_and_groups.f;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Hoodnf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Intakenf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Lednf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Loginf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Shooternf;
import org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf.Transfernf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.DrawingNew;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
@Disabled
@TeleOp(name = "NextFTC TeleOp Program Java")
public class NextFTCTele extends NextFTCOpMode {
    public NextFTCTele() {
        addComponents(
                new SubsystemComponent(
                        Loginf.INSTANCE,
                        Intakenf.INSTANCE
                ),
                new PedroComponent(Constants::createFollower),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE
        );
    }

    private Follower follower;
    double atErrorDeg;

    private Command atCorrection() {
//        return new LambdaCommand()
//                .setUpdate(() -> {
//                    new TurnBy(Angle.fromDeg(atErrorDeg)).schedule();
//                })
//                .setIsDone(() -> true);
        return new InstantCommand(() -> new TurnBy(Angle.fromDeg(atErrorDeg)).schedule());
    }


    @Override
    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void onStartButtonPressed() {
//        follower.startTeleopDrive(true);
        Gamepads.gamepad1().a()
                .whenBecomesTrue(
                        atCorrection()
                );
    }
    @Override
    public void onUpdate() {

        follower.update();
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x * 0.55;
//        follower.setTeleOpDrive(forward, strafe, rotate, true);

        atErrorDeg = Loginf.INSTANCE.getATangle();
        telemetry.addData("atErrorDeg", atErrorDeg);
        telemetry.addData("ATangle", Loginf.INSTANCE.getATangle());
        telemetry.update();
    }
}