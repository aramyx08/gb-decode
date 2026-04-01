package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServoGroup;

import org.firstinspires.ftc.teamcode.utilities.TelemetryDebug;

/**
 * Turret subsystem powered by dual Continuous Rotation (CR) servos.
 * <p>
 * Utilizes a Limelight 3A camera to automatically track and
 * align with targets using a PIDF control loop based on the target's horizontal offset (Tx).
 * </p>
 */
public class Turret {
    private CRServoEx leftTurretServo;
    private CRServoEx rightTurretServo;
    private CRServoGroup turret;
    private Limelight3A limelight;
    private TelemetryDebug debug;

    final PIDFController limelightPIDF = new PIDFController(0.012, 0.014, 0.00015, 0.005); // These values should work fine

    /**
     * Constructs a new Turret subsystem.
     * @param hardwareMap The hardware map to retrieve servo and camera devices.
     * @param isRed       Boolean to determine the alliance; sets Limelight pipeline to 0 (Red) or 1 (Blue).
     * @param debug       Instance of TelemetryDebug for real-time data logging.
     */
    public Turret (HardwareMap hardwareMap, boolean isRed, TelemetryDebug debug) {
        // Initialize Servos
        leftTurretServo = new CRServoEx(hardwareMap, "leftTurretServo");
        rightTurretServo = new CRServoEx(hardwareMap, "rightTurretServo");

        // Groups servos into controllable group
        turret = new CRServoGroup(leftTurretServo, rightTurretServo);

        // Initialize Limelight and change pipeline based on alliance
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(isRed ? 0 : 1);
        limelight.start();

        // Needed for custom debugging class
        this.debug = debug;
    }

    /**
     * Activates the Limelight tracking logic.
     * <p>
     * If a valid target is detected, the turret calculates the necessary power using
     * the {@code limelightPIDF} to center the target (Tx = 0). Power is clamped
     * for safety and to prevent oscillations. If no target is found, the turret stops.
     * </p>
     */
    public void aim() {
        limelight.start();
        // Retrieve limelight data
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Calculate and safely limit target power based on Tx
            double power = limelightPIDF.calculate(result.getTx(), 0);
            power = Math.max(-0.7, Math.min(0.7, power));

            // Telemetry
            debug.createWatcher("Tx", result.getTx());
            debug.createWatcher("Turret Power", power);

            turret.set(power);
        } else {
            turret.set(0);
        }
    }

    /**
     * Puts the turret into an idle state.
     * Stops servo movement and pauses Limelight to reduce processing overhead.
     */
    public void idle() {
        turret.set(0);
        limelight.pause();
    }
}
