package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.utilities.TelemetryDebug;

public class Kicker {
    private final double highPosition = 0.8;
    private final double lowPosition = 0;
    private ServoEx kickerOne;
    private ServoEx kickerTwo;
    private ServoEx kickerThree;
    private State currentState = State.IDLE;
    private ElapsedTime timer = new ElapsedTime();
    private TelemetryDebug debug;

    public Kicker(HardwareMap hardwareMap, TelemetryDebug debug) {
        kickerOne = new ServoEx(hardwareMap, "lifter0");
        kickerTwo = new ServoEx(hardwareMap, "lifter1");
        kickerThree = new ServoEx(hardwareMap, "lifter2");

        kickerOne.setInverted(true);

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
                resetKickers();
                break;

            case KICKER_1_UP:
                kickerOne.set(highPosition);
                if (timer.seconds() >= 0.2) {
                    currentState = State.KICKER_1_DOWN;
                    timer.reset();
                }
                break;

            case KICKER_1_DOWN:
                kickerOne.set(lowPosition);
                // 0.8s gap before the next one starts
                if (timer.seconds() >= 0.8) {
                    currentState = State.KICKER_2_UP;
                    timer.reset();
                }
                break;

            case KICKER_2_UP:
                kickerTwo.set(highPosition);
                if (timer.seconds() >= 0.2) {
                    currentState = State.KICKER_2_DOWN;
                    timer.reset();
                }
                break;

            case KICKER_2_DOWN:
                kickerTwo.set(lowPosition);
                if (timer.seconds() >= 0.8) {
                    currentState = State.KICKER_3_UP;
                    timer.reset();
                }
                break;

            case KICKER_3_UP:
                kickerThree.set(highPosition);
                if (timer.seconds() >= 0.2) {
                    currentState = State.KICKER_3_DOWN;
                    timer.reset();
                }
                break;

            case KICKER_3_DOWN:
                kickerThree.set(lowPosition);
                // Sequence finished, go back to IDLE
                if (timer.seconds() >= 0.8) {
                    currentState = State.IDLE;
                }
                break;
        }

        debug.createWatcher("Kicker State", currentState);
    }

    private void resetKickers() {
        kickerOne.set(lowPosition);
        kickerTwo.set(lowPosition);
        kickerThree.set(lowPosition);
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
