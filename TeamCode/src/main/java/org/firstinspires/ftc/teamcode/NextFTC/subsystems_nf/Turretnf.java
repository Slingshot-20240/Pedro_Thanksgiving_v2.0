package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.opencv.core.Mat;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
public class Turretnf implements Subsystem {

    public static double distance;
    // P is pretty much 1/range 
    public static PIDCoefficients turretPIDCoefficients =
            new PIDCoefficients(0.02, 0.0, 0);

    public static final Turretnf INSTANCE = new Turretnf();
    private Turretnf() {}

    private CRServoEx axonTurretServo = new CRServoEx("turretServo", 0.01);

    public AnalogInput turretEncoder;



    public ControlSystem turretPIDController = ControlSystem.builder()
            .posPid(turretPIDCoefficients)
            .build();
    public double power;
    public static Double targetTurretAng = 0.0;
    private static double offset = 4.0;
    private static final double GEAR_RATIO = 3.25;

    // Got MIN and MAX from 180/ Gear Ratio and then added some tolerance
    private static final double TURRET_MIN_DEG = -50;
    private static final double TURRET_MAX_DEG =  50;
    public static boolean AUTO_AIM = true;

    Pose goal = new Pose(144,144);




    @Override
    public void initialize() {
        turretEncoder = ActiveOpMode.hardwareMap().get(AnalogInput.class, "turretAnalog");

    }
    @Override
    public void periodic() {
        if (AUTO_AIM) {
            turretLoop();
        }
        turretPIDController.setGoal(new KineticState(targetTurretAng));
        // (-) Power because gear rotates other gear opposite
        power = -turretPIDController.calculate(new KineticState(getTurretDegrees()));
        axonTurretServo.setPower(power);
    }

    public void turretLoop(){

        double dx = 144 - PedroComponent.follower().getPose().getX();
        double dy = 144 - PedroComponent.follower().getPose().getY();

        //Arc tangent of dy/dx 
        double goalFieldDeg = Math.toDegrees(Math.atan2(dy, dx));
        double headingDeg   = Math.toDegrees(PedroComponent.follower().getHeading());

        // turret relative to robot forward 0 degree facing to goal side
        double desired = normalizeAngle(goalFieldDeg - headingDeg);
        targetTurretAng = clamp(desired, TURRET_MIN_DEG, TURRET_MAX_DEG);

    }
    //SHHHHH
    public void velocityBasedTurret(){
        Vector robot2Goal = new Vector(
                144-PedroComponent.follower().getPose().getX(),
                144-PedroComponent.follower().getPose().getY()
        );
        Vector velocity = PedroComponent.follower().getVelocity();
        double heading = PedroComponent.follower().getHeading();
        double coordTheta = velocity.getTheta() - robot2Goal.getTheta();
        double perpComponent = Math.sin(coordTheta) * velocity.getMagnitude();

        distance = Math.hypot(144-PedroComponent.follower().getPose().getX(), 144-PedroComponent.follower().getPose().getY());
    }


    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // Normalize angle to 180 so easy to work with 
    public double normalizeAngle(double angDeg) {
        double ang = angDeg;
        if (ang < 0.0) ang += 360.0;
        if (ang > 180.0) ang -= 360.0;
        return ang;
    }


    private double getTurretDegrees() {
        //yeah so voltage is what feedback get from servo and its pretty much a linear graph so all you need to do is divide by max voltage which is 3.3. And then i just multiply by 360 to go into degrees you could do radians but i dont like radians
        double servoDeg = (turretEncoder.getVoltage() / 3.3) * 360.0;

        // convert to turret degrees
        return normalizeAngle(servoDeg) / GEAR_RATIO;
    }
    public double turretDegrees(){
        return getTurretDegrees();
    }

    public double getPower(){
        return power;
    }

}