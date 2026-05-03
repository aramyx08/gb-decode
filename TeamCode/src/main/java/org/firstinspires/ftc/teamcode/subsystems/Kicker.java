package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.utilities.TelemetryDebug;

public class Kicker {
    private final double highPosition1 = 0.2;
    private final double lowPosition1 = 1;
    private final double highPosition2 = 0.2;
    private final double lowPosition2 = 1;
    private final double highPosition3 = 0.9;
    private final double lowPosition3 = 0;

    private Servo kickerOne;
    private Servo kickerTwo;
    private Servo kickerThree;
    private State currentState = State.IDLE;
    private ElapsedTime timer = new ElapsedTime();
    private TelemetryDebug debug;

    public Kicker(HardwareMap hardwareMap, TelemetryDebug debug) {
        kickerOne = hardwareMap.get(Servo.class, "lifter0");
        kickerTwo = hardwareMap.get(Servo.class, "lifter1");
        kickerThree = hardwareMap.get(Servo.class, "lifter2");

        kickerOne.setDirection(Servo.Direction.REVERSE);

        this.debug = debug;

        resetKickers();
    }

    /**
     * Call this once to begin the kicking sequence.
     */
    public void startSequence() {
        if (currentState == State.IDLE) {
            currentState = State.KICKER_1_UP;
            timer.reset();
        }
    }

    /**
     * This method must be called repeatedly in your OpMode loop.
     */
    public void update() {
        switch (currentState) {
            case IDLE:
                break;

            case KICKER_1_UP:
                kickerOne.setPosition(highPosition1);
                if (timer.seconds() >= 0.2) {
                    currentState = State.KICKER_1_DOWN;
                    timer.reset();
                }
                break;

            case KICKER_1_DOWN:
                kickerOne.setPosition(lowPosition1);
                // 0.8s gap before the next one starts
                if (timer.seconds() >= 0.4) {
                    currentState = State.KICKER_2_UP;
                    timer.reset();
                }
                break;

            case KICKER_2_UP:
                kickerTwo.setPosition(highPosition2);
                if (timer.seconds() >= 0.2) {
                    currentState = State.KICKER_2_DOWN;
                    timer.reset();
                }
                break;

            case KICKER_2_DOWN:
                kickerTwo.setPosition(lowPosition2);
                if (timer.seconds() >= 0.4) {
                    currentState = State.KICKER_3_UP;
                    timer.reset();
                }
                break;

            case KICKER_3_UP:
                kickerThree.setPosition(highPosition3);
                if (timer.seconds() >= 0.2) {
                    currentState = State.KICKER_3_DOWN;
                    timer.reset();
                }
                break;

            case KICKER_3_DOWN:
                kickerThree.setPosition(lowPosition3);
                // Sequence finished, go back to IDLE
                if (timer.seconds() >= 0.4) {
                    currentState = State.IDLE;
                }
                break;
        }

        debug.createWatcher("Kicker State", currentState);
    }

    private void resetKickers() {
        kickerOne.setPosition(lowPosition1);
        kickerTwo.setPosition(lowPosition2);
        kickerThree.setPosition(lowPosition3);
    }

    public boolean isBusy() {
        return currentState != State.IDLE;
    }

    // State Machine Variables
    public enum State {
        IDLE,
        KICKER_1_UP,
        KICKER_1_DOWN,
        KICKER_2_UP,
        KICKER_2_DOWN,
        KICKER_3_UP,
        KICKER_3_DOWN
    }
}
