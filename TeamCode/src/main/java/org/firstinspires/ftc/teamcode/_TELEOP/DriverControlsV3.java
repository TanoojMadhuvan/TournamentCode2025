//Hello
package org.firstinspires.ftc.teamcode._TELEOP;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._CONFIG.Hware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@TeleOp
public class DriverControlsV3 extends LinearOpMode {
    Boolean RUNNOTON = true;
    Boolean RUNNOTONV = true;

    boolean strafeonly = false;
    double SPEEDCONTROL = 1;
    double TURNCONTROL = 1;
    String vMSG = "PICK";
    String hMSG = "PICK";
    Hware robot = new Hware();
    int intakeWait = 0;

    //double wristPos = 0.158;
    double wristPos = 0.5;

    String armMode = "wall";

    public void changeHorizontalPosition(int pos1, int pos2, double pow) {
        if(pos1 < 440 && pos2 < 440 && pos1 > -5 && pos2 > -5) {


            for (int i = 0; i < intakeWait; i++) {

            }
            intakeWait = 0;
            RUNNOTON = false;
            robot.horizontal1.setTargetPosition(pos1);
            robot.horizontal2.setTargetPosition(pos2);
            robot.horizontal1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.horizontal2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.horizontal1.setPower(pow);
            robot.horizontal2.setPower(pow);
        }
    }

    public void changeVerticalPosition(int pos1, int pos2, double pow) {
        if(pos1 < 5 && pos2 > -5 && pos1 > -1082 && pos2 < 1082) {
            robot.vertical1.setTargetPosition(pos1);
            robot.vertical2.setTargetPosition(pos2);
            robot.vertical1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.vertical2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.vertical1.setPower(pow);
            robot.vertical2.setPower(pow);
        }
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

    public void changeVerticalPosition(int Position1, int Position2) {
        RUNNOTONV = false;
        robot.vertical1.setTargetPosition(Position1);
        robot.vertical1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.vertical2.setTargetPosition(Position2);
        robot.vertical2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // INTAKE PORTION
    public void rotateIntakeToGround(boolean adjusting) {
        if(adjusting) {
            robot.rightElbow.setPosition(0.159);
            robot.leftElbow.setPosition(0.841);
        }else{
            robot.rightElbow.setPosition(0);
            robot.leftElbow.setPosition(1);
        }
    }

    public void rotateIntakeToTransfer() {
        robot.rightElbow.setPosition(0.573);
        robot.leftElbow.setPosition(0.426);

    }

    public void rotateIntakeToMove() {
        robot.rightElbow.setPosition(0.573);
        robot.leftElbow.setPosition(0.426);
        intakeWait = 0;
    }

    //Pick from wall: r: 0.873, l:0.127
    //Transfer: r: 0, l: 1

    public void rotateOuttakeToPickWall() {
        robot.rightNeck.setPosition(0.75);
        robot.leftNeck.setPosition(0.25);
        robot.jaw.setPosition(0.53);
        armMode = "wall";
    }

    public void rotateOuttakeToScoreBucket() {
        robot.rightNeck.setPosition(0.67);
        robot.leftNeck.setPosition(0.33);
        robot.jaw.setPosition(0.55);
        armMode = "bucket";
    }

    public void rotateOuttakeToScoreBack() {
        robot.rightNeck.setPosition(0.42);
        robot.leftNeck.setPosition(0.58);
        robot.jaw.setPosition(0.4);
        armMode = "specimen";
    }

    public void rotateOuttakeToScoreFront() {
        robot.rightNeck.setPosition(0.05);
        robot.leftNeck.setPosition(0.95);
        robot.jaw.setPosition(0.488);
    }

    public void rotateOuttakeToTransfer() {
        robot.rightNeck.setPosition(0.03);
        robot.leftNeck.setPosition(0.97);
        robot.jaw.setPosition(0.75);
        armMode = "transfer";
    }

    public void rotateOuttakeToHangSpecimen() {
        robot.rightNeck.setPosition(0.82);
        robot.leftNeck.setPosition(0.18);
        robot.jaw.setPosition(0.555);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initialize(hardwareMap);

        GamepadEx driverOp;
        GamepadEx armOp;
        SampleMecanumDrive driveR = new SampleMecanumDrive(hardwareMap);
        driveR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MecanumDrive drive = new MecanumDrive(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight);
        armOp = new GamepadEx(gamepad2);
        driverOp = new GamepadEx(gamepad1);
        double x,y,xy;

        robot.backRight.setInverted(true);
        robot.frontRight.setInverted(true);

        rotateOuttakeToPickWall();
        rotateIntakeToTransfer();
        wristPos = 0.158;
        setWrist();

        waitForStart();

        boolean autoClick = false;
        float lastRightTrigger = 0;
        float lastLeftTrigger = 0;
        int autoCount = 0;
        boolean shareClicked = false;


        while(opModeIsActive()) {

            if(gamepad1.options){
                if((!autoClick && gamepad1.options) || autoCount > 0){
                    if(autoCount == 0) {
                        autoCount = 1;
                    }
                    if(autoCount == 10) {
                        SampleMecanumDrive smartDrive = new SampleMecanumDrive(hardwareMap);
                        smartDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        TrajectorySequence pickUp = smartDrive.trajectorySequenceBuilder(smartDrive.getPoseEstimate())
                                .addDisplacementMarker(() -> {
                                    closeIntakeClaw();
                                    rotateIntakeToTransfer();
                                    rotateOuttakeToPickWall();
                                    openOuttakeClaw();
                                    robot.intakeWrist.setPosition(0.158);
                                }).waitSeconds(.1)
                                .forward(0.001)
                                .addDisplacementMarker(() -> {
                                    changeHorizontalPosition(0, 0, 1);
                                    rotateOuttakeToTransfer();
                                })
                                //.waitSeconds(.2)
//                .lineToConstantHeading(new Vector2d(0, 0.02))
//                .addDisplacementMarker(()->{
//                    rotateOuttakeToTransfer();
//                })
                                .waitSeconds(0.01)
                                .forward(0.001)
                                .addDisplacementMarker(() -> {
                                    closeOuttakeClaw();
                                }).waitSeconds(1)
                                .forward(0.001)
                                .addDisplacementMarker(() -> {
                                    openIntakeClaw();
                                }).waitSeconds(0.2)
                                .forward(0.001)
                                .addDisplacementMarker(() -> {
                                    rotateOuttakeToScoreBack();
                                })
                                .build();
                        smartDrive.followTrajectorySequence(pickUp);
                        autoCount = 0;
                    } else if (autoCount > 0) {
                        autoCount ++;
                    }
                }
            }else {

                // DRIVER

                x = driverOp.getLeftX() * SPEEDCONTROL;
                y = driverOp.getLeftY() * SPEEDCONTROL;
                xy = driverOp.getRightX() * TURNCONTROL;

                y = gamepad1.square ? 0 : y; //Perfect straight
                x = gamepad1.cross ? 0 : x; //Perfect straight

                if (gamepad1.left_trigger > 0.5) {SPEEDCONTROL = .4;TURNCONTROL=.4;} else if(gamepad1.right_trigger>0.5) {TURNCONTROL = 0.4; SPEEDCONTROL = 1;} else {SPEEDCONTROL=1; TURNCONTROL=1;}

                drive.driveRobotCentric(
                        x,
                        y,
                        xy
                );

                // ARM


                if (gamepad1.cross) {
                    rotateIntakeToMove();
                    changeHorizontalPosition(0, 0, 1);

                    hMSG = "DROP";

                } else if (gamepad1.triangle) {
                    rotateIntakeToMove();
                    changeHorizontalPosition(439, 439, 1);

                    hMSG = "DROP";
                }

                if (gamepad1.circle) {
                    rotateIntakeToGround(false);
                }

                if (gamepad1.square) {
                    rotateIntakeToTransfer();
                }

                if(gamepad1.share && !shareClicked){
                    //Share (clicked): Toggle between all 4 arm modes in this cyclic order: wall-mode, transfer-mode, specimen-mode, bucket-mode
                    if(armMode.equals("wall")){

                        rotateOuttakeToTransfer();
                    }else if(armMode.equals("transfer")){

                        rotateOuttakeToScoreBack();
                    }else if(armMode.equals("specimen")){

                        rotateOuttakeToScoreBucket();
                    }else if(armMode.equals("bucket")){

                        rotateOuttakeToPickWall();
                    }
                }

                //Port3: wrist
                //Port4: elbow1
                //Port5: intake claw
                //Port2: elbow2

                if (gamepad1.dpad_up) {
                    changeVerticalPosition(-1079, 1081, 1);
                    rotateOuttakeToScoreBucket();
                }

                if (gamepad1.dpad_down) {
                    changeVerticalPosition(0, 0, 1);
                    rotateOuttakeToPickWall();
                }


                if (Math.abs(gamepad1.left_trigger) > 0 && Math.abs(gamepad1.right_trigger) > 0) {
                    wristPos = 0.158;
                    setWrist();
                } else if (Math.abs(gamepad1.left_trigger) > 0 && Math.abs(lastLeftTrigger) < 0.0001) {
                    wristPos -= Math.abs(gamepad1.left_trigger) / 10;
                    if (wristPos < 0.158) {
                        wristPos = 0.158;
                    }
                    setWrist();
                } else if (Math.abs(gamepad1.right_trigger) > 0 && Math.abs(lastRightTrigger) < 0.0001) {
                    wristPos += Math.abs(gamepad1.right_trigger) / 10;
                    if (wristPos > 0.503) {
                        wristPos = 0.503;
                    }
                    setWrist();
                }


                if (gamepad1.right_bumper) {
                    openIntakeClaw();

                    if (Math.abs(robot.rightElbow.getPosition() - 0.25) < 0.1) {
                        sleep(300);
                        rotateIntakeToGround(false);
                    }

                } else {
                    closeIntakeClaw();
                }


                if (gamepad1.dpad_right) {
                    rotateOuttakeToScoreBack();
                    changeVerticalPosition(-360,348,1);
                }

                if (gamepad1.dpad_left) {
                    rotateOuttakeToScoreBack();
                    changeVerticalPosition(-170,164,1);
                }

                if (gamepad1.left_bumper) {
                    openOuttakeClaw();
                } else {
                    closeOuttakeClaw();
                }

               if(gamepad1.touchpad){
                   if(gamepad1.touchpad_finger_1_x < 0 && gamepad1.touchpad_finger_1_y < 0){
                       changeHorizontalPosition(robot.horizontal1.getCurrentPosition() - 10, robot.horizontal2.getCurrentPosition() - 10, 0.3);
                   }else if(gamepad1.touchpad_finger_1_x > 0 && gamepad1.touchpad_finger_1_y < 0){
                       changeHorizontalPosition(robot.horizontal1.getCurrentPosition() + 10, robot.horizontal2.getCurrentPosition() + 10, 0.3);
                   }else if(gamepad1.touchpad_finger_1_x < 0 && gamepad1.touchpad_finger_1_y > 0){
                       changeVerticalPosition(robot.vertical1.getCurrentPosition() + 10, robot.vertical2.getCurrentPosition() - 10, 0.3);
                   }else if(gamepad1.touchpad_finger_1_x > 0 && gamepad1.touchpad_finger_1_y > 0){
                       changeVerticalPosition(robot.vertical1.getCurrentPosition() - 10, robot.vertical2.getCurrentPosition() + 10, 0.3);
                   }
               }

            }



            autoClick = gamepad1.options;
            lastRightTrigger = gamepad1.right_trigger;
            lastLeftTrigger = gamepad1.left_trigger;
            shareClicked = gamepad1.share;

            telemetry.addData("H1 Encoder: ", (Integer.toString(robot.horizontal1.getCurrentPosition())));
            telemetry.addData("H2 Encoder: ", (Integer.toString(robot.horizontal2.getCurrentPosition())));
            telemetry.addData("V1 Encoder: ", (Integer.toString(robot.vertical1.getCurrentPosition())));
            telemetry.addData("V2 Encoder: ", (Integer.toString(robot.vertical2.getCurrentPosition())));
            telemetry.addData("Right Elbow: ", (Double.toString(robot.rightElbow.getPosition())));
            telemetry.addData("Left Elbow: ", (Double.toString(robot.leftElbow.getPosition())));
            telemetry.addData("Right Neck: ", (Double.toString(robot.rightNeck.getPosition())));
            telemetry.addData("Left Neck: ", (Double.toString(robot.leftNeck.getPosition())));

            telemetry.addData("Wrist: ", (Double.toString(robot.intakeWrist.getPosition())));
            telemetry.addData("Intake claw: ", (Double.toString(robot.intakeClaw.getPosition())));

            telemetry.addData("Jaw: ", (Double.toString(robot.jaw.getPosition())));
            telemetry.addData("Outtake claw: ", (Double.toString(robot.outtakeClaw.getPosition())));
            telemetry.addData("Touchpad fing 1: ", (Boolean.toString(gamepad1.touchpad_finger_1)));
            telemetry.addData("Touchpad fing 2: ", (Boolean.toString(gamepad1.touchpad_finger_2)));
            telemetry.addData("Touchpad fing 1x: ", (Float.toString(gamepad1.touchpad_finger_1_x)));
            telemetry.addData("Touchpad fing 2x: ", (Float.toString(gamepad1.touchpad_finger_1_y)));
            telemetry.addData("Touchpad fing 1 press: ", (Boolean.toString(gamepad1.touchpad)));

            telemetry.addData("Arm mode: ", (armMode));



            telemetry.update();



        }

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


        while (!isStopRequested() && opModeIsActive());
    }

}
