package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.utilities.TelemetryDebug;

public class Shooter {
    private Motor topFlywheelMotor;
    private Motor bottomFlywheelMotor;
    private MotorGroup flywheel;
    private ServoEx pitchServo;
    private Limelight3A limelight;
    private TelemetryDebug debug;
    private PIDFController velocityController = new PIDFController(0.002, 0.00005, 0.000003, 0.00022);

    private double distanceFromTarget;

    public Shooter (HardwareMap hardwareMap, boolean isRed, TelemetryDebug debug) {
        topFlywheelMotor = new Motor(hardwareMap, "topMotor", 28, 6000);
        bottomFlywheelMotor = new Motor(hardwareMap, "bottomMotor", 28, 6000);

        flywheel = new MotorGroup(topFlywheelMotor, bottomFlywheelMotor);
        flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        flywheel.setRunMode(Motor.RunMode.RawPower);

        pitchServo = new ServoEx(hardwareMap, "pitchServo");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(isRed ? 0 : 1);
        limelight.start();

        this.debug = debug;
    }

    public void accelerate() {
        limelight.start();
        LLResult result = limelight.getLatestResult();

        double rawDistance = 87.78147 * (Math.pow(result.getTa(), -0.41445)); // This should work just fine
        distanceFromTarget = distanceFromTarget == 0 ? rawDistance : (0.6 * rawDistance) + (0.4 * distanceFromTarget);

        debug.createWatcher("Distance", distanceFromTarget);

        double currentRPM = toRPM(flywheel.getCorrectedVelocity());
        double targetRPM = 9.77 * distanceFromTarget + 1820; // TODO: Tune this or recalculate regression line from scratch
        double power = velocityController.calculate(currentRPM, targetRPM);

        debug.createWatcher("Target RPM", targetRPM);
        debug.createWatcher("Flywheel Power", power);

        flywheel.set(power);

        double targetPitch = 0; // TODO: Calculate a regression line based on certain points

        debug.createWatcher("Target Pitch", targetPitch);

        pitchServo.set(targetPitch);

    }

    public void idle () {
        limelight.pause();
        flywheel.set(0);
    }

    public double toRPM(double ticksPerSecond) {
        // 28 is the Ticks Per Revolution (TPR) for the 1:1 5203 series motor
        final double TICKS_PER_REV = 28.0;
        return (ticksPerSecond * 60.0) / TICKS_PER_REV;
    }




}
