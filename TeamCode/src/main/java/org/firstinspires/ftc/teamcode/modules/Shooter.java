package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.utility.ButtonState;


// TODO: Сделать
@Config
public class Shooter {
    private final LinearOpMode linearOpMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final Servo servoHold;
    private final Servo servoRaise;

    private double up_pos = 0.6;
    private double down_pos = 0.45;
    public static double hold_pos = 0.55;
    public static double release_pos = 1;
    private ButtonState leftTriggerState = ButtonState.RELEASED;
    private ButtonState leftBumperState = ButtonState.RELEASED;
    private ButtonState rightStickButtonState = ButtonState.RELEASED;

    private enum PositionState {
        UP,
        DOWN,
        HOLDING
    }
    private PositionState position = PositionState.DOWN;
    private int counter = 0;

    public void start_shooter_pos(){
        servoRaise.setPosition(down_pos);
    }



    public Shooter(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;

        servoHold = hardwareMap.get(Servo.class, "servo_launch");
        servoRaise = hardwareMap.get(Servo.class, "servo_angle");

        telemetry.addLine("Shooter: Initialized");
    }

    public void tele() {
        switch (leftBumperState) {
            case PRESSED:
                servoRaise.setPosition(servoRaise.getPosition() + 0.05);
                if (gamepad.left_bumper) {
                    leftBumperState = ButtonState.HELD;
                } else {
                    leftBumperState = ButtonState.RELEASED;
                }
                break;
            case HELD:
                if (!gamepad.left_bumper) {
                    leftBumperState = ButtonState.RELEASED;
                }
                break;
            case RELEASED:
                if (gamepad.left_bumper) {
                    leftBumperState = ButtonState.PRESSED;
                }
                break;
        }
        switch (leftTriggerState) {
            case PRESSED:
                servoRaise.setPosition(servoRaise.getPosition() - 0.05);
                if (gamepad.left_trigger > 0) {
                    leftTriggerState = ButtonState.HELD;
                } else {
                    leftTriggerState = ButtonState.RELEASED;
                }
                break;
            case HELD:
                if (gamepad.left_trigger == 0) {
                    leftTriggerState = ButtonState.RELEASED;
                }
                break;
            case RELEASED:
                if (gamepad.left_trigger > 0) {
                    leftTriggerState = ButtonState.PRESSED;
                }
                break;
        }
        switch (rightStickButtonState) {
            case PRESSED:
                switch (position) {
                    case UP:
                        position = PositionState.HOLDING;
                        break;
                    case DOWN:
                        position = PositionState.UP;
                        break;
                    case HOLDING:
                        position = PositionState.DOWN;
                        break;
                }
                if (gamepad.dpad_up) {
                    rightStickButtonState = ButtonState.HELD;
                } else {
                    rightStickButtonState = ButtonState.RELEASED;
                }
                break;
            case HELD:
                if (!gamepad.dpad_up) {
                    rightStickButtonState = ButtonState.RELEASED;
                }
                break;
            case RELEASED:
                if (gamepad.dpad_up) {
                    rightStickButtonState = ButtonState.PRESSED;
                }
                break;
        }
        switch (position) {
            case DOWN:
                servoRaise.setPosition(down_pos);
                break;
            case UP:
                servoRaise.setPosition(up_pos);
                position = PositionState.HOLDING;
                break;
            case HOLDING:
                servoRaise.setPosition(servoRaise.getPosition());
                break;
        }
        if (gamepad.b) {
            counter += 1;
            if (counter > 5) {
                servoHold.setPosition(release_pos);
            }
        } else {
            servoHold.setPosition(hold_pos);
            counter = 0;
        }
        telemetry.addLine("---------------");
        telemetry.addLine("Shooter:");
        telemetry.addData("right_stick", rightStickButtonState);
        telemetry.addData("left_bumper", leftBumperState);
        telemetry.addData("left_trigger", leftTriggerState);
    }

    public void teleWithPos() {
        switch (leftBumperState) {
            case PRESSED:
                servoRaise.setPosition(servoRaise.getPosition() + 0.05);
                if (gamepad.right_stick_button) {
                    leftBumperState = ButtonState.HELD;
                } else {
                    leftBumperState = ButtonState.RELEASED;
                }
                break;
            case HELD:
                if (!gamepad.dpad_up) {
                    leftBumperState = ButtonState.RELEASED;
                }
                break;
            case RELEASED:
                if (gamepad.dpad_up) {
                    leftBumperState = ButtonState.PRESSED;
                }
                break;
        }
        switch (leftTriggerState) {
            case PRESSED:
                servoRaise.setPosition(servoRaise.getPosition() - 0.05);
                if (gamepad.left_stick_button) {
                    leftTriggerState = ButtonState.HELD;
                } else {
                    leftTriggerState = ButtonState.RELEASED;
                }
                break;
            case HELD:
                if (!gamepad.left_stick_button) {
                    leftTriggerState = ButtonState.RELEASED;
                }
                break;
            case RELEASED:
                if (gamepad.left_stick_button) {
                    leftTriggerState = ButtonState.PRESSED;
                }
                break;
        }
        if (gamepad.left_trigger > 0 && gamepad.left_bumper) {
            servoHold.setPosition(release_pos);
        }
        else {
            servoHold.setPosition(hold_pos);
        }
    }
}
