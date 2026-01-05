package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToState;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.FeedbackCRServoEx;
import dev.nextftc.hardware.impl.FeedbackServoEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Turretnf implements Subsystem {
    public static final Turretnf INSTANCE = new Turretnf();

    private Turretnf() {
    }

    public FeedbackServoEx turretServo1 = new FeedbackServoEx("analog0", "turret1", 0.01);
    public FeedbackServoEx turretServo2 = new FeedbackServoEx("analog1", "turret2", 0.01);

    // TODO got from docs idk we'll see :sk

    double totalAngle = 0.0; // This is your angle of the servo
    double previousAngle = 0.0; // This is the previous loop's servo position

    void updatePosition() {
        double currentAngle = turretServo1.getCurrentPosition();
        double deltaAngle = currentAngle - previousAngle;

        if (deltaAngle > Math.PI) deltaAngle -= 2 * Math.PI;
        else if (deltaAngle < -Math.PI) deltaAngle += 2 * Math.PI;

        totalAngle += deltaAngle;
        previousAngle = currentAngle;
    }

    // TODO lowkey dont know what method to use bc it's an angle so

    public Command farBlue() {
        return new ParallelGroup(
                new SetPosition(turretServo1, 0),
                new SetPosition(turretServo1, 0)
        );
    }

    public Command closeBlue() {
        return new ParallelGroup(
                new SetPosition(turretServo1, 0),
                new SetPosition(turretServo2, 0)
        );
    }

    public Command farRed() {
        return new ParallelGroup(
                new SetPosition(turretServo1, 0),
                new SetPosition(turretServo2, 0)
        );
    }

    public Command closeRed() {
        return new ParallelGroup(
                new SetPosition(turretServo1, 0),
                new SetPosition(turretServo2, 0)
        );
    }



    @Override
    public void initialize() {
        turretServo1 = new ServoEx("turret1");
        turretServo2 = new ServoEx("turret2");
    }

    @Override
    public void periodic() {

    }
}
