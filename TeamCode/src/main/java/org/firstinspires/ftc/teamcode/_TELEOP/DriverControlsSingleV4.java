/**
 Driver Controls
 Drive train:
 Left joystick: translational motion
 Right joystick: rotational motion

 Intake:-
 Triangle: Horizontal slides out
 Cross: Horizontal slides in
 Circle: Intake baby arm out
 Square: Intake baby arm in
 Right bumper: Open intake claw (automatic close when not pressed)
 Right trigger pressed: rotate wrist clockwise 20 degrees
 Left trigger pressed: rotate wrist counterclockwise 20 degrees
 Both triggers pressed: set wrist to perpendicular (ready for transfer)

 Outtake:-
 Dpad up: Vertical slides max up, arm in position to score in bucket-mode
 Dpad left: Vertical slides & arm align to score specimen in specimen-mode
 Dpad right: Vertical slides move slightly up from dpad left to score specimen
 Dpad down: Vertical slides all the way down, arm goes to pick up from wall (wall-mode) default to prevent collisions inside robot
 Share (clicked): Toggle between all 4 arm modes in this cyclic order: wall-mode, transfer-mode, specimen-mode, bucket-mode
 Left bumper: Open outtake claw (automatic close when not pressed)

 Autonomous transfer:-
 Options: Needs to be pressed and held throughout duration of transfer, all other controls will stop working

 Manual adjusts with pressing touchpad:
 Bottom left: Adjust of horizontal slides inwards slightly
 Bottom right: Adjust of horizontal slides outwards slightly
 Top left: Adjust of vertical slides downwards slightly
 Top right: Adjust of vertical slides upwards slightly
 **/

package org.firstinspires.ftc.teamcode._TELEOP;

import android.widget.Button;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._CONFIG.HwareV2;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp
public class DriverControlsSingleV4 extends LinearOpMode {
    boolean outtakeClawManual = true;
    HwareV2 robot = new HwareV2();
    Boolean H_RUNNOTON = true;
    Boolean V_RUNNOTON = true;

    double SPEEDCONTROL = 1;
    double TURNCONTROL = 1;

    int intakeWait = 0;

    double wristPos = 0.5;

    String armMode = "wall";
    boolean autoClick = false;
    boolean smartModeOn = false;
    float lastRightTrigger = 0;
    float lastLeftTrigger = 0;
    boolean shareClicked = false;
    String intakeMode = "transfer";


    public void changeHorizontalPosition(int pos1, int pos2, double pow) {
        if(pos1 < 440 && pos2 < 440 && pos1 > -5 && pos2 > -5) {
            for (int i = 0; i < intakeWait; i++) {

            }
            intakeWait = 0;
            H_RUNNOTON = false;
            robot.horizontal1.setTargetPosition(pos1);
            robot.horizontal2.setTargetPosition(pos2);
            robot.horizontal1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.horizontal2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.horizontal1.setPower(pow);
            robot.horizontal2.setPower(pow);
        }
    }

    public void changeVerticalPosition(int pos1, int pos2, double pow) {
        V_RUNNOTON = false;
        //if(pos1 < 5 && pos2 > -5 && pos1 > -1082 && pos2 < 1082) {
            robot.vertical1.setTargetPosition(pos1);
            robot.vertical2.setTargetPosition(pos2);
            robot.vertical1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.vertical2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.vertical1.setPower(pow);
            robot.vertical2.setPower(pow);
        //}
    }

    public void setWrist(){
        robot.intakeWrist.setPosition(wristPos);
    }

    public void openIntakeClaw(){
        robot.intakeClaw.setPosition(0.2);
    }

    public void closeIntakeClaw(){
        robot.intakeClaw.setPosition(0.5);
    }

    public void openOuttakeClaw(){
        robot.outtakeClaw.setPosition(0.08);
    }

    public void closeOuttakeClaw(){
        robot.outtakeClaw.setPosition(0.25);
    }

    // INTAKE PORTION
    public void rotateIntakeToGround(boolean adjusting) {
        if(adjusting) {
            robot.rightElbow.setPosition(0.2);
            robot.leftElbow.setPosition(0.8);
            intakeMode = "adjusting";
        } else {
            robot.rightElbow.setPosition(0.1);
            robot.leftElbow.setPosition(0.9);
            intakeMode = "picking";
        }
    }

    public void rotateIntakeToTransfer() {
        robot.rightElbow.setPosition(0.66);
        robot.leftElbow.setPosition(0.34);
        intakeWait = 0;
        intakeMode = "transfer";
    }

    public void rotateIntakeToMove() {
        robot.rightElbow.setPosition(0.55);
        robot.leftElbow.setPosition(0.45);
        intakeWait = 0;
    }

    public void rotateOuttakeToPickWall() {
        robot.rightNeck.setPosition(0.75);
        robot.leftNeck.setPosition(0.25);
        robot.jaw.setPosition(0.1);
        armMode = "wall";
    }

    public void rotateOuttakeToScoreBucket() {

        robot.rightNeck.setPosition(0.42);
        robot.leftNeck.setPosition(0.58);
        robot.jaw.setPosition(0.9);
        armMode = "specimen";
    }

    public void rotateOuttakeToScoreBack() {
        robot.rightNeck.setPosition(0.67);
        robot.leftNeck.setPosition(0.33);
        robot.jaw.setPosition(0.1);
        armMode = "back";
    }

    public void rotateOuttakeToScoreFront() {
        robot.rightNeck.setPosition(0);
        robot.leftNeck.setPosition(1);
        robot.jaw.setPosition(0.2);
        armMode = "front";
    }

    public void rotateOuttakeToTransfer() {
        robot.rightNeck.setPosition(0);
        robot.leftNeck.setPosition(1);
        robot.jaw.setPosition(0.708);
        armMode = "transfer";
    }

    public void rotateOuttakeToHangSpecimen() {
        robot.rightNeck.setPosition(0.82);
        robot.leftNeck.setPosition(0.18);
        robot.jaw.setPosition(0.555);
    }

    public void rumblePads(int times) {
        gamepad1.rumbleBlips(times);
        gamepad1.rumbleBlips(times);
    }



    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize(hardwareMap);
//        robot.backRight.setInverted(false);
//        robot.frontRight.setInverted(false);

        MecanumDrive drive = new MecanumDrive(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);

        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx armOp = new GamepadEx(gamepad1);

        SampleMecanumDrive driveBase = new SampleMecanumDrive(hardwareMap);
        driveBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double x, y, xy;

        // Auto Trajectory Builder
        TrajectorySequence pickUp = driveBase.trajectorySequenceBuilder(driveBase.getPoseEstimate())
                .forward(.01)
                .addDisplacementMarker(() -> {
                    changeHorizontalPosition(0, 0, 1);
                    closeIntakeClaw();
                    rotateIntakeToTransfer();
                    robot.intakeWrist.setPosition(0.158);
                    openOuttakeClaw();
                })
                .forward(.01)
                .waitSeconds(.01)
                .addDisplacementMarker(() -> {
                    rotateOuttakeToTransfer();
                })
                .forward(.01)
                .waitSeconds(.01)
                .addDisplacementMarker(() -> {
                    openIntakeClaw();
                    closeOuttakeClaw();
                }).waitSeconds(1.5)
                .forward(0.000001)
                .addDisplacementMarker(() -> {
                    rotateOuttakeToScoreBack();

                }).waitSeconds(.3)
                .forward(0.000001)
                .addDisplacementMarker(() -> {
                    outtakeClawManual = true;
                    rumblePads(1);
                })
                .build();


        // Servo Presets
        rotateOuttakeToTransfer();
        rotateIntakeToTransfer();
        closeOuttakeClaw();
        wristPos = 0.158; // Reset Wrist Position
        setWrist();

        waitForStart();


        while(opModeIsActive()) {

            // Automatic Transfer

            /** DRIVER CONTROLS **/

            x = armOp.getLeftX() * SPEEDCONTROL;
            y = armOp.getLeftY() * SPEEDCONTROL;
            xy = armOp.getRightX() * TURNCONTROL;

            y = gamepad1.square ? 0 : y; //Perfect straight
            x = gamepad1.cross ? 0 : x; //Perfect straight

            if (armOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
                SPEEDCONTROL = .4;
                TURNCONTROL = .4;
            } else if (armOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
                TURNCONTROL = 0.4;
                SPEEDCONTROL = 1;
            } else {
                SPEEDCONTROL = 1;
                TURNCONTROL = 1;
            }

            /** ARM CONTROLS **/

//            // Horizontal Control
//            if (armOp.getLeftY() != 0) {
//                H_RUNNOTON = true;
//                robot.horizontal1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.horizontal2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                if (robot.horizontal1.getCurrentPosition() < 440 && robot.horizontal1.getCurrentPosition() > -6) {
//                    robot.horizontal1.setPower(armOp.getLeftY() / 2);
//                    robot.horizontal2.setPower(armOp.getLeftY() / 2);
//
//                } else {
//                    robot.horizontal1.setPower(0);
//                    robot.horizontal2.setPower(0);
//                }
//            } else {
//                if (H_RUNNOTON) {
//                    robot.horizontal1.setPower(0);
//                    robot.horizontal2.setPower(0);
//                }
//            }

            // Horizontal Slide Controls
            if (armOp.getButton(GamepadKeys.Button.A)) { // Cross
                rotateIntakeToMove();
                changeHorizontalPosition(0, 0, .6);
                rotateOuttakeToPickWall();
            } else if (armOp.getButton(GamepadKeys.Button.Y)) { // Triangle
                rotateIntakeToMove();
                changeHorizontalPosition(360, 360, .6);
                rotateOuttakeToPickWall();
            }

            // Horizontal Intake Controls
            if (armOp.getButton(GamepadKeys.Button.B)) { // Circle
                rotateIntakeToGround(true);
            }

            if (gamepad1.options) { // Square
                rotateIntakeToTransfer();
            }


            // Arm Vertical Control

//            if (armOp.getRightY() != 0) {
//                V_RUNNOTON = true;
//                robot.vertical1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                robot.vertical2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//                if (robot.vertical2.getCurrentPosition() < 4400 && robot.vertical2.getCurrentPosition() > -5) {
//                    robot.vertical1.setPower(armOp.getRightY());
//                    robot.vertical2.setPower(-armOp.getRightY());
//
//                } else {
//
//                    robot.vertical1.setPower(0);
//                    robot.vertical2.setPower(0);
//                }
//
//            } else {
//                if (V_RUNNOTON) {
//                    robot.vertical1.setPower(0);
//                    robot.vertical2.setPower(0);
//                }
//            }

            // Arm Presets DPAD
            if (armOp.getButton(GamepadKeys.Button.DPAD_UP)) {
                changeVerticalPosition(-950, 950, 1);
                rotateOuttakeToScoreBack();
            }

            if (armOp.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                changeVerticalPosition(0, 0, 0.4);
                if(armMode.equals("transfer") || armMode.equals("wall")){
                rotateOuttakeToPickWall();
                }
            }

            if (armOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                if(!(armMode.equals("back") || armMode.equals("front"))) {
                    rotateOuttakeToScoreBack();
                }
                changeVerticalPosition(-700, 700, 1);
            }

            if (armOp.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                if(!(armMode.equals("back") || armMode.equals("front"))) {
                    rotateOuttakeToScoreBack();
                }
                changeVerticalPosition(-360, 348, 1);

            }

            // Slight Adjustment Controls, Vertical and Horizontal
            if(gamepad1.touchpad) {
                if(gamepad1.touchpad_finger_1_x < 0 && gamepad1.touchpad_finger_1_y < 0){
                    changeHorizontalPosition(robot.horizontal1.getCurrentPosition() - 10, robot.horizontal2.getCurrentPosition() - 10, 0.3);
                }else if(gamepad1.touchpad_finger_1_x > 0 && gamepad1.touchpad_finger_1_y < 0){
                    changeHorizontalPosition(robot.horizontal1.getCurrentPosition() + 10, robot.horizontal2.getCurrentPosition() + 10, 0.3);
                }else if(gamepad1.touchpad_finger_1_x < 0 && gamepad1.touchpad_finger_1_y > 0){
                    changeVerticalPosition(robot.vertical1.getCurrentPosition() + 10, robot.vertical2.getCurrentPosition() + 10, 0.3);
                }else if(gamepad1.touchpad_finger_1_x > 0 && gamepad1.touchpad_finger_1_y > 0){
                    changeVerticalPosition(robot.vertical1.getCurrentPosition() - 10, robot.vertical2.getCurrentPosition() - 10, 0.3);
                }
            }

            // Mode Selection Cycler
            if (gamepad1.share) {
                if(!shareClicked) {
                    //Share (clicked): Toggle between all 4 arm modes in this cyclic order: wall-mode, transfer-mode, specimen-mode, bucket-mode
                    if (armMode.equals("wall")) {
                        rotateOuttakeToTransfer();
                    }
                    else if (armMode.equals("transfer")) {
                        rotateOuttakeToScoreFront();
                    }
                    else if (armMode.equals("front")) {
                        rotateOuttakeToScoreBack();
                    }
                    else if (armMode.equals("back")) {
                        rotateOuttakeToPickWall();
                    }
                    shareClicked = true;
                }else {
                    shareClicked = true;
                }
            }else{
                shareClicked = false;
            }

            if (armOp.getButton(GamepadKeys.Button.X)) {
                outtakeClawManual = false;
                driveBase.followTrajectorySequence(pickUp);
            }

            // Wrist Adjustment
            if (Math.abs(gamepad1.left_trigger) > 0 && Math.abs(gamepad1.right_trigger) > 0) {
                wristPos = 0.158;
                setWrist();
            } else if (Math.abs(gamepad1.left_trigger) > 0 ) {
                wristPos -= Math.abs(gamepad1.left_trigger) / 100;
                if (wristPos < 0.158) {
                    wristPos = 0.158;
                }
                setWrist();
            } else if (Math.abs(gamepad1.right_trigger) > 0 ) {
                wristPos += Math.abs(gamepad1.right_trigger) / 100;
                if (wristPos > 0.503) {
                    wristPos = 0.503;
                }
                setWrist();
            }

            // Claw Motion
            if (armOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                openIntakeClaw();
                if (intakeMode.equals("adjusting")) {
                    rotateIntakeToGround(false);
                }
            } else { closeIntakeClaw(); }
            if(outtakeClawManual) {

                drive.driveRobotCentric(
                        x,
                        y,
                        xy
                );

                if (armOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                    openOuttakeClaw();
                } else {
                    closeOuttakeClaw();
                }
            }
            // Rumble
            if (Math.abs(robot.vertical1.getCurrentPosition() - robot.vertical1.getTargetPosition()) <= 5) {
                //rumblePads(1);
                if (robot.vertical1.getTargetPosition() == 0) {
                    robot.vertical1.setPower(0);
                    robot.vertical2.setPower(0);
                }
            }

            if (Math.abs(robot.horizontal1.getCurrentPosition() - robot.horizontal1.getTargetPosition()) <= 5) {
                if (robot.horizontal1.getTargetPosition() == 0) {
                    robot.horizontal1.setPower(0);
                    robot.horizontal2.setPower(0);
                }
            }



            autoClick = gamepad1.options;
            lastRightTrigger = gamepad1.right_trigger;
            lastLeftTrigger = gamepad1.left_trigger;


            telemetry.addLine("-- DRIVE TEAM --");
            telemetry.addLine("| Drive Speed: " + SPEEDCONTROL + "% |"); // Drive
            telemetry.addData("| Arm mode: ", armMode); // Arm Mode
            telemetry.addData("| gamepad1.share : ", gamepad1.share); // Arm Mode
            telemetry.addData("| shareClicked : ", shareClicked); // Arm Mode



            telemetry.addData("| Arm Horizontal Manual Mode: ", H_RUNNOTON);
            telemetry.addData("| Arm Vertical Manual Mode: ", H_RUNNOTON);

            telemetry.addLine("\n-- DEVELOPER --");
            telemetry.addLine("LIFT ENCODERS");
            telemetry.addData("| H1 Encoder: ", Integer.toString(robot.horizontal1.getCurrentPosition()));
            telemetry.addData("| H2 Encoder: ", Integer.toString(robot.horizontal2.getCurrentPosition()));
            telemetry.addData("| V1 Encoder: ", Integer.toString(robot.vertical1.getCurrentPosition()));
            telemetry.addData("| V2 Encoder: ", Integer.toString(robot.vertical2.getCurrentPosition()));

            telemetry.addLine("\nOUTTAKE SERVO POSITIONS");
            telemetry.addData("| Right Elbow: ", Double.toString(robot.rightElbow.getPosition()));
            telemetry.addData("| Left Elbow: ", Double.toString(robot.leftElbow.getPosition()));
            telemetry.addData("| Right Neck: ", Double.toString(robot.rightNeck.getPosition()));
            telemetry.addData("| Left Neck: ", Double.toString(robot.leftNeck.getPosition()));
            telemetry.addData("| Wrist: ", Double.toString(robot.intakeWrist.getPosition()));
            telemetry.addData("| Intake Claw: ", Double.toString(robot.intakeClaw.getPosition()));

            telemetry.addLine("\nTOUCHPAD ADJUSTMENTS");
            telemetry.addData("| Touchpad Fing 1: ", Boolean.toString(gamepad1.touchpad_finger_1));
            telemetry.addData("| Touchpad Fing 2: ", Boolean.toString(gamepad1.touchpad_finger_2));
            telemetry.addData("| Touchpad Fing 1x: ", Float.toString(gamepad1.touchpad_finger_1_x));
            telemetry.addData("| Touchpad Fing 1y: ", Float.toString(gamepad1.touchpad_finger_1_y));
            telemetry.addData("| Touchpad Fing 1 Press: ", Boolean.toString(gamepad1.touchpad));

            telemetry.update();

        }
        while (!isStopRequested() && opModeIsActive());

    }

}