package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.utility.ButtonState;


@Config
public class Scorer {
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final Servo servoRotationBox;
    private final Servo servoRotationBeamLeft;
    private final Servo servoRotationBeamRight;
    private final Servo servoHoldUpper;
    private final Servo servoHoldLower;

    public enum HolderState {
        HOLD,
        RELEASE
    }
    public HolderState holderUpperState;
    public HolderState holderLowerState;
    public static double holdPos = 0.5;
    public static double releasePos = 0;

    public enum RotationState {
        TAKE,
        MOVE,
        DEPLOY
    }
    public RotationState rotationBoxState;
    public RotationState rotationBeamState;

    public static double takeBoxPos = 0.75;
    public static double moveBoxPos = 0.839;
    public static double deployBoxPos = 0.22;
    public static double takeBeamPos = 0.23;
    public static double moveBeamPos = 0.268;
    public static double deployBeamPos = 0.782;



    private ButtonState dpadDownState = ButtonState.RELEASED;
    private ButtonState dpadUpState = ButtonState.RELEASED;
    private int counterUp = -1;
    private int counterDown = -1;
    public static int delayDown = 40;
    public static int delayUp = 10;

    public Scorer(LinearOpMode linearOpMode) {
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;

        servoRotationBox = hardwareMap.get(Servo.class, "servo_rotation_box");

        servoRotationBeamLeft = hardwareMap.get(Servo.class, "servo_rotation_beam_left");
        servoRotationBeamRight = hardwareMap.get(Servo.class, "servo_rotation_beam_right");
        servoRotationBeamRight.setDirection(Servo.Direction.REVERSE);

        servoHoldLower = hardwareMap.get(Servo.class, "servo_hold_lower");
        servoHoldUpper = hardwareMap.get(Servo.class, "servo_hold_upper");
        servoHoldUpper.setDirection(Servo.Direction.REVERSE);


        rotationBoxState = RotationState.TAKE;
        rotationBeamState = RotationState.TAKE;
        holderUpperState = HolderState.RELEASE;
        holderLowerState = HolderState.RELEASE;

        telemetry.addLine("Scorer: Initialized");
    }

    public void opControl() {
        if (gamepad.a && rotationBeamState == RotationState.DEPLOY) { // Intake
            rotationBeamState = RotationState.MOVE;
            rotationBoxState = RotationState.MOVE;
            holderLowerState = HolderState.HOLD;
            holderUpperState = HolderState.HOLD;
            counterDown = delayDown;
        } else if (gamepad.y && rotationBeamState == RotationState.TAKE) { // Deploy
            rotationBeamState = RotationState.MOVE;
            rotationBoxState = RotationState.MOVE;
            holderLowerState = HolderState.HOLD;
            holderUpperState = HolderState.HOLD;
            counterUp = delayUp;
        }

        // HoldLower Button switch
        switch (dpadDownState) {
            case PRESSED:
                switch (holderLowerState) {
                    case RELEASE: holderLowerState = HolderState.HOLD;
                        break;
                    case HOLD: holderLowerState = HolderState.RELEASE;
                        break;
                }
                if (gamepad.dpad_down) {
                    dpadDownState = ButtonState.HELD;
                } else {
                    dpadDownState = ButtonState.RELEASED;
                }
                break;
            case HELD:
                if (!gamepad.dpad_down) {
                    dpadDownState = ButtonState.RELEASED;
                }
                break;
            case RELEASED:
                if (gamepad.dpad_down) {
                    dpadDownState = ButtonState.PRESSED;
                }
                break;
        }

        // HoldUpper Button switch
        switch (dpadUpState) {
            case PRESSED:
                switch (holderUpperState) {
                    case RELEASE: holderUpperState = HolderState.HOLD;
                        break;
                    case HOLD: holderUpperState = HolderState.RELEASE;
                        break;
                }
                if (gamepad.dpad_up) {
                    dpadUpState = ButtonState.HELD;
                } else {
                    dpadUpState = ButtonState.RELEASED;
                }
                break;
            case HELD:
                if (!gamepad.dpad_up) {
                    dpadUpState = ButtonState.RELEASED;
                }
                break;
            case RELEASED:
                if (gamepad.dpad_up) {
                    dpadUpState = ButtonState.PRESSED;
                }
                break;
        }

        switch (rotationBoxState) {
            case TAKE: servoRotationBox.setPosition(takeBoxPos);
                break;
            case DEPLOY: servoRotationBox.setPosition(deployBoxPos);
                break;
            case MOVE: servoRotationBox.setPosition(moveBoxPos);
                break;
        }

        switch (rotationBeamState) {
            case TAKE:
                servoRotationBeamLeft.setPosition(takeBeamPos);
                servoRotationBeamRight.setPosition(takeBeamPos);
                break;
            case DEPLOY:
                servoRotationBeamLeft.setPosition(deployBeamPos);
                servoRotationBeamRight.setPosition(deployBeamPos);
                break;
            case MOVE:
                servoRotationBeamLeft.setPosition(moveBeamPos);
                servoRotationBeamRight.setPosition(moveBeamPos);
                break;
        }

        switch (holderLowerState) {
            case HOLD: servoHoldLower.setPosition(holdPos);
                break;
            case RELEASE: servoHoldLower.setPosition(releasePos);
                break;
        }

        switch (holderUpperState) {
            case HOLD: servoHoldUpper.setPosition(holdPos);
                break;
            case RELEASE: servoHoldUpper.setPosition(releasePos);
                break;
        }

        counterUp -= 1;

        if (counterUp == 0) {
            rotationBeamState = RotationState.DEPLOY;
            rotationBoxState = RotationState.DEPLOY;
        }

        counterDown -= 1;

        if (counterDown == 0) {
            rotationBeamState = RotationState.TAKE;
            rotationBoxState = RotationState.TAKE;
            holderUpperState = HolderState.RELEASE;
            holderLowerState = HolderState.RELEASE;
        }

        telemetry.addLine("---------------");
        telemetry.addLine("Scorer:");
        telemetry.addData("upper_hold", holderUpperState);
        telemetry.addData("lower_hold", holderLowerState);
        telemetry.addData("beam", rotationBeamState);
        telemetry.addData("box", rotationBoxState);
        telemetry.addData("counter_up", counterUp);
        telemetry.addData("counter_down", counterDown);

    }
}
