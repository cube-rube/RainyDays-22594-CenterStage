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
    private final Lift lift;
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

    public static double takeBoxPos = 0.87; // Позиция коробки при захвате
    public static double moveBoxPos = 0.86; // Позиция коробки при перекиде
    public static double deployBoxPos = 0.32; // Позиция коробки при выгрузке
    public static double takeBeamPos = 0.02; // Позиция перекида при захвате
    public static double moveBeamPos = 0.3 - 0.21; // Позиция перекида между
    public static double deployBeamPos = 0.68; // Позиция перекида при выгрузке
    public static double deployBeamAutoPos = 1;
    public static double deployBoxAutoPos = 0.4;

    private ButtonState dpadDownState = ButtonState.RELEASED;
    private ButtonState dpadUpState = ButtonState.RELEASED;
    private int counterClose = -1;
    private int counterLift = -1;
    private int counterDown = -1;
    public static int delayDown = 21;
    public static int delayClose = 10;
    public static int delayLift = 20;
    public static int liftPos = 200;

    public Scorer(LinearOpMode linearOpMode, Lift lift) {
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;
        this.lift = lift;

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

    public void close_upper() {
        servoHoldUpper.setPosition(holdPos);
    }

    public void open_upper() {
        servoHoldUpper.setPosition(releasePos);
    }

    public void close_lower() {
        servoHoldLower.setPosition(holdPos);
    }

    public void open_lower() {
        servoHoldLower.setPosition(releasePos);
    }

    public void deploy() {
        servoRotationBox.setPosition(0.38);
        servoRotationBeamLeft.setPosition(0.915 - 0.21);
        servoRotationBeamRight.setPosition(0.915 - 0.21);
    }
    public void deployAuto() {
        servoRotationBox.setPosition(deployBoxAutoPos);
        servoRotationBeamLeft.setPosition(deployBeamAutoPos);
        servoRotationBeamRight.setPosition(deployBeamAutoPos);
    }

    public void move() {
        servoRotationBox.setPosition(moveBoxPos);
        servoRotationBeamLeft.setPosition(moveBeamPos);
        servoRotationBeamRight.setPosition(moveBeamPos);
    }

    public void take() {
        servoRotationBox.setPosition(takeBoxPos);
        servoRotationBeamLeft.setPosition(takeBeamPos);
        servoRotationBeamRight.setPosition(takeBeamPos);
    }

    public void opControl() {
        if (gamepad.a && rotationBeamState == RotationState.DEPLOY) { // Intake
            rotationBeamState = RotationState.MOVE;
            rotationBoxState = RotationState.MOVE;
            holderLowerState = HolderState.HOLD;
            holderUpperState = HolderState.HOLD;
            counterDown = delayDown;
            lift.setReference(0);
        } else if (gamepad.y && rotationBeamState == RotationState.TAKE) { // Deploy
            holderLowerState = HolderState.HOLD;
            holderUpperState = HolderState.HOLD;
            counterClose = delayClose;
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
                if (gamepad.right_trigger > 0) {
                    dpadDownState = ButtonState.HELD;
                } else {
                    dpadDownState = ButtonState.RELEASED;
                }
                break;
            case HELD:
                if (gamepad.right_trigger == 0) {
                    dpadDownState = ButtonState.RELEASED;
                }
                break;
            case RELEASED:
                if (gamepad.right_trigger > 0) {
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
                if (gamepad.right_bumper) {
                    dpadUpState = ButtonState.HELD;
                } else {
                    dpadUpState = ButtonState.RELEASED;
                }
                break;
            case HELD:
                if (!gamepad.right_bumper) {
                    dpadUpState = ButtonState.RELEASED;
                }
                break;
            case RELEASED:
                if (gamepad.right_bumper) {
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

        counterClose -= 1;
        if (counterClose == 0) {
            rotationBeamState = RotationState.DEPLOY;
            rotationBoxState = RotationState.DEPLOY;
            if (lift.getReference() == 0) {
                lift.setReference(liftPos);
            }
            counterLift = delayLift;
        }

        counterLift -= 1;
        if (counterLift == 0) {
            if (lift.getReference() == liftPos) {
                lift.setReference(0);
            }
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
        telemetry.addData("counter_up", counterClose);
        telemetry.addData("counter_down", counterDown);

    }
}
