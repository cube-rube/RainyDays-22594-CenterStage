package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class Deploy {
    private final LinearOpMode linearOpMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final Servo servoRotationBox;
    private final Servo servoRotationBeam;
    private final Servo servoHoldUpper;
    private final Servo servoHoldLower;

    public enum HolderState {
        HOLD,
        RELEASE
    }
    private final static double holdUpperPos = 0;
    private final static double releaseUpperPos = 1;
    private final static double holdLowerPos = 0;
    private final static double releaseLowerPos = 1;

    public HolderState holderUpperState = HolderState.RELEASE;
    public HolderState holderLowerState = HolderState.RELEASE;
    public enum RotationState {
        TAKE,
        DEPLOY
    }
    private final static double takeBoxPos = 0;
    private final static double deployBoxPos = 1;
    private final static double takeBeamPos = 0;
    private final static double deployBeamPos = 1;
    public RotationState rotationBoxState = RotationState.TAKE;
    public RotationState rotationBeamState = RotationState.TAKE;
    public RotationState rotationState = RotationState.TAKE;
    private enum ButtonState {
        PRESSED,
        HELD,
        RELEASED
    }
    private ButtonState aState = ButtonState.RELEASED;
    private ButtonState bState = ButtonState.RELEASED;
    private ButtonState xState = ButtonState.RELEASED;
    private ButtonState yState = ButtonState.RELEASED;

    public Deploy(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;

        servoRotationBox = hardwareMap.get(Servo.class, "servo_rotation_box");
        servoRotationBeam = hardwareMap.get(Servo.class, "servo_rotation_beam");
        servoHoldUpper = hardwareMap.get(Servo.class, "servo_hold_upper");
        servoHoldLower = hardwareMap.get(Servo.class, "servo_hold_lower");
    }

    public void tele() {
        switch (aState) {
            case PRESSED:
                switch (holderLowerState) {
                    case RELEASE:
                        holderLowerState = HolderState.HOLD;
                        break;
                    case HOLD:
                        holderLowerState = HolderState.RELEASE;
                }
                if (gamepad.a) {
                    aState = ButtonState.HELD;
                } else {
                    aState = ButtonState.RELEASED;
                }
            case HELD:
                if (!gamepad.a) {
                    aState = ButtonState.RELEASED;
                }
            case RELEASED:
                if (gamepad.a) {
                    aState = ButtonState.PRESSED;
                }
        }
        switch (bState) {
            case PRESSED:
                switch (holderUpperState) {
                    case RELEASE:
                        holderUpperState = HolderState.HOLD;
                        break;
                    case HOLD:
                        holderUpperState = HolderState.RELEASE;
                }
                if (gamepad.b) {
                    bState = ButtonState.HELD;
                } else {
                    bState = ButtonState.RELEASED;
                }
            case HELD:
                if (!gamepad.b) {
                    bState = ButtonState.RELEASED;
                }
            case RELEASED:
                if (gamepad.b) {
                    bState = ButtonState.PRESSED;
                }
        }
        switch (xState) {
            case PRESSED:
                switch (rotationBoxState) {
                    case TAKE:
                        rotationBoxState = RotationState.DEPLOY;
                        break;
                    case DEPLOY:
                        rotationBoxState = RotationState.TAKE;
                }
                if (gamepad.x) {
                    xState = ButtonState.HELD;
                } else {
                    xState = ButtonState.RELEASED;
                }
            case HELD:
                if (!gamepad.x) {
                    xState = ButtonState.RELEASED;
                }
            case RELEASED:
                if (gamepad.x) {
                    xState = ButtonState.PRESSED;
                }
        }
        switch (yState) {
            case PRESSED:
                switch (rotationBeamState) {
                    case TAKE:
                        rotationBeamState = RotationState.DEPLOY;
                        break;
                    case DEPLOY:
                        rotationBeamState = RotationState.TAKE;
                }
                if (gamepad.y) {
                    yState = ButtonState.HELD;
                } else {
                    yState = ButtonState.RELEASED;
                }
            case HELD:
                if (!gamepad.y) {
                    yState = ButtonState.RELEASED;
                }
            case RELEASED:
                if (gamepad.y) {
                    yState = ButtonState.PRESSED;
                }
        }
        switch (holderLowerState) {
            case HOLD:
                servoHoldLower.setPosition(holdLowerPos);
            case RELEASE:
                servoHoldLower.setPosition(releaseLowerPos);
        }
        switch (holderUpperState) {
            case HOLD:
                servoHoldUpper.setPosition(holdUpperPos);
            case RELEASE:
                servoHoldUpper.setPosition(releaseUpperPos);
        }
        switch (rotationBoxState) {
            case TAKE:
                servoRotationBox.setPosition(takeBoxPos);
            case DEPLOY:
                servoRotationBox.setPosition(deployBoxPos);
        }
        switch (rotationBeamState) {
            case TAKE:
                servoRotationBeam.setPosition(takeBeamPos);
            case DEPLOY:
                servoRotationBeam.setPosition(deployBeamPos);
        }
    }

    // Вот эту функцию надо редачить, вращение коробки и балки на Y, верхняя держалка пикселя на B, нижняя на A
    public void teleOneButtonRotation() {
        switch (yState) {
            case PRESSED:
                switch (rotationState) {
                    case TAKE:
                        rotationState = RotationState.DEPLOY;
                        break;
                    case DEPLOY:
                        rotationState = RotationState.TAKE;
                }
                if (gamepad.y) {
                    yState = ButtonState.HELD;
                } else {
                    yState = ButtonState.RELEASED;
                }
            case HELD:
                if (!gamepad.y) {
                    yState = ButtonState.RELEASED;
                }
            case RELEASED:
                if (gamepad.y) {
                    yState = ButtonState.PRESSED;
                }
        }
        switch (aState) {
            case PRESSED:
                switch (holderLowerState) {
                    case RELEASE:
                        holderLowerState = HolderState.HOLD;
                        break;
                    case HOLD:
                        holderLowerState = HolderState.RELEASE;
                }
                if (gamepad.a) {
                    aState = ButtonState.HELD;
                } else {
                    aState = ButtonState.RELEASED;
                }
            case HELD:
                if (!gamepad.a) {
                    aState = ButtonState.RELEASED;
                }
            case RELEASED:
                if (gamepad.a) {
                    aState = ButtonState.PRESSED;
                }
        }
        switch (bState) {
            case PRESSED:
                switch (holderUpperState) {
                    case RELEASE:
                        holderUpperState = HolderState.HOLD;
                        break;
                    case HOLD:
                        holderUpperState = HolderState.RELEASE;
                }
                if (gamepad.b) {
                    bState = ButtonState.HELD;
                } else {
                    bState = ButtonState.RELEASED;
                }
            case HELD:
                if (!gamepad.b) {
                    bState = ButtonState.RELEASED;
                }
            case RELEASED:
                if (gamepad.b) {
                    bState = ButtonState.PRESSED;
                }
        }
        switch (holderLowerState) {
            case HOLD:
                servoHoldLower.setPosition(holdLowerPos);
            case RELEASE:
                servoHoldLower.setPosition(releaseLowerPos);
        }
        switch (holderUpperState) {
            case HOLD:
                servoHoldUpper.setPosition(holdUpperPos);
            case RELEASE:
                servoHoldUpper.setPosition(releaseUpperPos);
        }
        switch (rotationState) {
            case TAKE:
                servoRotationBox.setPosition(takeBoxPos);
                servoRotationBeam.setPosition(takeBeamPos);
            case DEPLOY:
                servoRotationBox.setPosition(deployBoxPos);
                servoRotationBeam.setPosition(deployBeamPos);
        }
    }

    public void testing() {
        telemetry.addData("Deploy: ", "rotation_beam %4.2f", servoRotationBox.getPosition());
        telemetry.addData("Deploy: ", "rotation_box %4.2f", servoRotationBox.getPosition());
        telemetry.addData("Deploy: ", "hold_upper  %4.2f", servoHoldUpper.getPosition());
        telemetry.addData("Deploy: ", "hold_lower  %4.2f", servoHoldLower.getPosition());
    }
}
