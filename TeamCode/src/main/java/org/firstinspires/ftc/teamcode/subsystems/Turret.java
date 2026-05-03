package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utilities.TelemetryDebug;

/**
 * Turret subsystem powered by dual Continuous Rotation (CR) servos.
 * <p>
 * Utilizes a Limelight 3A camera to automatically track and
 * align with targets using a PIDF control loop based on the target's horizontal offset (Tx).
 * </p>
 */
public class Turret {
    private CRServo leftTurretServo;
    private CRServo rightTurretServo;
//    private CRServoEx turret;
    private Limelight3A limelight;
    private TelemetryDebug debug;

    final PIDFController limelightPIDF = new PIDFController(0.009, 0.014, 0.0003 , 0.0); // These values should work fine

    /**
     * Constructs a new Turret subsystem.
     * @param hardwareMap The hardware map to retrieve servo and camera devices.
     * @param isRed       Boolean to determine the alliance; sets Limelight pipeline to 0 (Red) or 1 (Blue).
     * @param debug       Instance of TelemetryDebug for real-time data logging.
     */
    public Turret (HardwareMap hardwareMap, boolean isRed, TelemetryDebug debug) {
        // Initialize Servos
//        leftTurretServo = new CRServoEx(hardwareMap, "azimuthServo0");
//        turret = new CRServoEx(hardwareMap, "azimuthServo1");

        leftTurretServo = hardwareMap.get(CRServo.class, "azimuthServo0");
        rightTurretServo = hardwareMap.get(CRServo.class, "azimuthServo1");

        leftTurretServo.setDirection(CRServo.Direction.REVERSE);
        rightTurretServo.setDirection(CRServo.Direction.REVERSE);



        // Groups servos into controllable group
//        turret = new CRServoGroup(leftTurretServo, rightTurretServo);

        // Initialize Limelight and change pipeline based on alliance
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(isRed ? 1 : 0);
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
            power = Math.max(-0.6, Math.min(0.6, power));

            // Telemetry
            debug.createWatcher("Tx", result.getTx());
            debug.createWatcher("Turret Power", power);

            rightTurretServo.setPower(power);
            leftTurretServo.setPower(power);
        } else {
            rightTurretServo.setPower(0);
            leftTurretServo.setPower(0);
        }
    }

    /**
     * Puts the turret into an idle state.
     * Stops servo movement and pauses Limelight to reduce processing overhead.
     */
    public void idle() {
        rightTurretServo.setPower(0);
        leftTurretServo.setPower(0);
        limelight.pause();
    }
}
