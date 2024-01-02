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
    private final Servo servoRotationBeamLeft;
    private final Servo servoRotationBeamRight;
    private final Servo servoHoldUpper;
    private final Servo servoHoldLower;

    public enum HolderState {
        HOLD,
        RELEASE
    }
    public static double holdUpperPos = 0.9;
    public static double releaseUpperPos = 0.5;
    public static double holdLowerPos = 1;
    public static double releaseLowerPos = 0.7;

    public HolderState holderUpperState = HolderState.RELEASE;
    public HolderState holderLowerState = HolderState.RELEASE;
    public enum RotationState {
        TAKE,
        TRANSPORT,
        TRANSPORT_DOWN,
        DEPLOY
    }
    public static double takeBoxPos = 0.94;
    public static double deployBoxPos = 0.47;
    public static double takeBeamPos = 0;
    public static double deployBeamPos = 1;
    public static double transportBeamPos = 0;
    public static double transportDownBeamPos = 0.09;
    public static double transportDownBoxPos = 0.96;

    public RotationState rotationBoxState;
    public RotationState rotationBeamState;
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
    private ButtonState dpadDownState = ButtonState.RELEASED;
    private ButtonState dpadUpState = ButtonState.RELEASED;
    public enum DriveState {
        ENABLED,
        DISABLED
    }
    public DriveState driveState = DriveState.ENABLED;

    public Deploy(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;

        servoRotationBox = hardwareMap.get(Servo.class, "servo_rotation_box");
        servoRotationBeamLeft = hardwareMap.get(Servo.class, "servo_rotation_beam1");
        servoRotationBeamRight = hardwareMap.get(Servo.class, "servo_rotation_beam2");
        servoHoldUpper = hardwareMap.get(Servo.class, "servo_hold_upper");
        servoHoldLower = hardwareMap.get(Servo.class, "servo_hold_lower");
        servoRotationBeamLeft.setDirection(Servo.Direction.REVERSE);

        rotationBoxState = RotationState.TAKE;
        rotationBeamState = RotationState.TRANSPORT;
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
                servoRotationBeamLeft.setPosition(takeBeamPos);
            case DEPLOY:
                servoRotationBeamLeft.setPosition(deployBeamPos);
        }
    }

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
                servoRotationBeamLeft.setPosition(takeBeamPos);
            case DEPLOY:
                servoRotationBox.setPosition(deployBoxPos);
                servoRotationBeamLeft.setPosition(deployBeamPos);
        }
    }

    public void rotationPositions() {
        if (gamepad.a) { // Intake
            rotationBeamState = RotationState.TAKE;
            rotationBoxState = RotationState.TAKE;
            holderLowerState = HolderState.RELEASE;
            driveState = DriveState.DISABLED;
        }
        /*
        if (gamepad.b) { // Transport
            rotationBeamState = RotationState.TRANSPORT;
            rotationBoxState = RotationState.TAKE;
            driveState = DriveState.ENABLED;
        }

         */
        if (gamepad.y && holderLowerState == HolderState.HOLD) { // Deploy
            rotationBeamState = RotationState.DEPLOY;
            rotationBoxState = RotationState.DEPLOY;
            driveState = DriveState.ENABLED;
        }

        // HoldLower Box Servo switch
        switch (dpadDownState) {
            case PRESSED:
                switch (holderLowerState) {
                    case RELEASE:
                        holderLowerState = HolderState.HOLD;
                        break;
                    case HOLD:
                        holderLowerState = HolderState.RELEASE;
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

        // HoldUpper Box Servo switch
        switch (dpadUpState) {
            case PRESSED:
                switch (holderUpperState) {
                    case RELEASE:
                        holderUpperState = HolderState.HOLD;
                        break;
                    case HOLD:
                        holderUpperState = HolderState.RELEASE;
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
        /*
        switch (rotationBoxState) {
            case TAKE:
                servoRotationBox.setPosition(takeBoxPos);
            case DEPLOY:
                servoRotationBox.setPosition(deployBoxPos);
            case TRANSPORT_DOWN:
                servoRotationBox.setPosition(transportDownBoxPos);
        }

         */

        if (rotationBoxState == RotationState.TAKE) {
            servoRotationBox.setPosition(takeBoxPos);
        } else if (rotationBoxState == RotationState.DEPLOY) {
            servoRotationBox.setPosition(deployBoxPos);
        } else if (rotationBoxState == RotationState.TRANSPORT_DOWN) {
            servoRotationBox.setPosition(transportDownBoxPos);
        }
        /*
        switch (rotationBeamState) {
            case TAKE:
                servoRotationBeamLeft.setPosition(takeBeamPos);
                servoRotationBeamRight.setPosition(takeBeamPos);
            case DEPLOY:
                servoRotationBeamLeft.setPosition(deployBeamPos);
                servoRotationBeamRight.setPosition(deployBeamPos);
            case TRANSPORT:
                servoRotationBeamLeft.setPosition(transportBeamPos);
                servoRotationBeamRight.setPosition(transportBeamPos);
            case TRANSPORT_DOWN:
                servoRotationBeamLeft.setPosition(transportDownBeamPos);
                servoRotationBeamRight.setPosition(transportDownBeamPos);

        }

         */

        if (rotationBeamState == RotationState.TAKE) {
            servoRotationBeamLeft.setPosition(takeBeamPos);
            servoRotationBeamRight.setPosition(takeBeamPos);
        } else if (rotationBeamState == RotationState.DEPLOY) {
            servoRotationBeamLeft.setPosition(deployBeamPos);
            servoRotationBeamRight.setPosition(deployBeamPos);
        } else if (rotationBeamState == RotationState.TRANSPORT) {
            servoRotationBeamLeft.setPosition(transportBeamPos);
            servoRotationBeamRight.setPosition(transportBeamPos);
        } else if (rotationBeamState == RotationState.TRANSPORT_DOWN) {
            servoRotationBeamLeft.setPosition(transportDownBeamPos);
            servoRotationBeamRight.setPosition(transportDownBeamPos);
        }
        /*
        switch (holderLowerState) {
            case HOLD:
                servoHoldLower.setPosition(holdLowerPos);
            case RELEASE:
                servoHoldLower.setPosition(releaseLowerPos);
        }

         */

        if (holderLowerState == HolderState.HOLD) {
            servoHoldLower.setPosition(holdLowerPos);
        } else if (holderLowerState == HolderState.RELEASE) {
            servoHoldLower.setPosition(releaseLowerPos);
        }
        /*
        switch (holderUpperState) {
            case HOLD:
                servoHoldUpper.setPosition(holdUpperPos);
            case RELEASE:
                servoHoldUpper.setPosition(releaseUpperPos);
        }

         */

        if (holderUpperState == HolderState.HOLD) {
            servoHoldUpper.setPosition(holdUpperPos);
        } else if (holderUpperState == HolderState.RELEASE) {
            servoHoldUpper.setPosition(releaseUpperPos);
        }
        telemetry.addLine("----------------------");
        telemetry.addData("Upper_Hold: ", holderUpperState);
        telemetry.addData("Lower_Hold: ", holderLowerState);
        telemetry.addLine("----------------------");
    }

    public void auto() {
        servoRotationBeamLeft.setPosition(transportBeamPos);
        servoRotationBox.setPosition(takeBoxPos);
    }

    public void testDoubleBeamServos() {
        if (gamepad.a) {
            servoRotationBeamLeft.setPosition(takeBeamPos);
            // servoRotationBeamRight.setPosition(takeBeamPos);
        }
    }

    public boolean getDriveState() {
        return driveState == DriveState.ENABLED;
    }

    public void testing() {
        servoRotationBeamLeft.setPosition(deployBeamPos);
        servoRotationBox.setPosition(deployBoxPos);
        servoHoldLower.setPosition(holdLowerPos);
        servoHoldUpper.setPosition(holdUpperPos);
    }
}
