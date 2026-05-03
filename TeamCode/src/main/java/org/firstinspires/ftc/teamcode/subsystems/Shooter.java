package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.TelemetryDebug;

/**
 * Shooter subsystem featuring dual flywheel motors and adjustable pitch.
 * <p>
 * This class uses a Limelight 3A to determine target distance via the "Target Area" (Ta) metric.
 * Employs a PIDF controller to maintain flywheel velocity and a linear
 * regression to map target area to optimal RPM and pitch angles.
 * </p>
 */
public class Shooter {
    private DcMotorEx topFlywheelMotor;
    private DcMotorEx bottomFlywheelMotor;
    private Servo pitchServo;
    private Limelight3A limelight;
    private TelemetryDebug debug;
    private double filteredTa;

    // TODO: Recalculate PID Constants. These values should be close to final
    private PIDFController velocityController = new PIDFController(0.002, 0.00005, 0.000003, 0.00022);

    /**
     * Constructs a new Shooter subsystem.
     * @param hardwareMap The hardware map to retrieve motor, servo, and camera devices.
     * @param isRed       Boolean to determine the alliance; sets Limelight pipeline to 0 (Red) or 1 (Blue).
     * @param debug       Instance of TelemetryDebug for real-time data logging.
     */
    public Shooter (HardwareMap hardwareMap, boolean isRed, TelemetryDebug debug) {
        // Initialize Motors
        topFlywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher0");
        topFlywheelMotor.setDirection(DcMotorEx.Direction.REVERSE);

        bottomFlywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher1");

        pitchServo = hardwareMap.get(Servo.class, "elevationServo");

        // Initialize Limelight and change pipeline based on alliance
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(isRed ? 1 : 0);
        limelight.start();

        // Needed for custom debugging class
        this.debug = debug;
    }

    /**
     * Executes the shooting sequence by calculating and applying motor power and servo pitch.
     * <p>
     * If a valid Limelight target is visible, the method:
     * <ol>
     * <li>Applies a low-pass filter to the Target Area (Ta) to smooth out distance estimates.</li>
     * <li>Calculates current RPM from motor velocity.</li>
     * <li>Uses linear regression to find the target RPM and pitch based on {@code filteredTa}.</li>
     * <li>Updates the PIDF controller and sets hardware outputs.</li>
     * </ol>
     * If no target is valid, the flywheel power is set to 0.
     * </p>
     */
    public void accelerate() {
        limelight.start();
        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
            // Collect and low pass filter Ta
//            filteredTa = filteredTa == 0 ? result.getTa() : (0.6 * result.getTa()) + (0.4 * filteredTa);

            filteredTa = result.getTa();

            // Calculate RPM and pitch values
            double currentRPM = toRPM(topFlywheelMotor.getVelocity());
            double targetRPM =  // TODO: Calculate linear regression (e.g targetRPM = -2.7 * filteredTa + 1800)
            3574.97926*(Math.pow(filteredTa, -0.156948));
            double power = velocityController.calculate(currentRPM, targetRPM);

            double targetPitch =  // TODO: Calculate linear regression (e.g targetPitch = 1.4 * filteredTa - 1.73)
            (-0.0475945*(filteredTa*filteredTa))+(0.11103*filteredTa)+0.387717;


            // Telemetry
            debug.createWatcher("Ta", filteredTa);
            debug.createWatcher("Target RPM", targetRPM);
            debug.createWatcher("Flywheel Power", power);
            debug.createWatcher("Error", targetRPM - currentRPM);
            debug.createWatcher("Target Pitch", targetPitch);

            // Set calculated values
            topFlywheelMotor.setPower(power);
            bottomFlywheelMotor.setPower(power);
            pitchServo.setPosition(Math.min(targetPitch, 0.7));
        } else {
            topFlywheelMotor.setPower(0);
            bottomFlywheelMotor.setPower(0);
        }
    }

    /**
     * Shuts down the shooter subsystem.
     * Pauses the Limelight sensor and cuts power to the flywheel motors.
     */
    public void idle () {
        limelight.pause();
        topFlywheelMotor.setPower(0);
        bottomFlywheelMotor.setPower(0);
    }

    private double toRPM(double ticksPerSecond) {
        // 28 is the Ticks Per Revolution (TPR) for the 1:1 5203 series motor
        final double TICKS_PER_REV = 28.0;
        return (ticksPerSecond * 60.0) / TICKS_PER_REV;
    }
}
