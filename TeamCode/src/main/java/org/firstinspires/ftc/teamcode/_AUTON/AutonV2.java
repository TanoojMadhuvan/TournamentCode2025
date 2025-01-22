package org.firstinspires.ftc.teamcode._AUTON;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode._CONFIG.Hware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import kotlin.text.MatchGroup;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AutonV2 extends LinearOpMode {

    Boolean RUNNOTON = true;
    Boolean RUNNOTONV = true;

    double SPEEDCONTROL = 1;
    double TURNCONTROL = 1;
    String vMSG = "PICK";
    String hMSG = "PICK";
    Hware robot;

    public void changeHorizontalPosition(int Position) {
        RUNNOTON = false;
        robot.horizontal1.setTargetPosition(Position);
        robot.horizontal2.setTargetPosition(Position);
        robot.horizontal1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.horizontal2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void changeVerticalPosition(int Position1, int Position2) {
        RUNNOTONV = false;
        robot.vertical1.setTargetPosition(Position1);
        robot.vertical1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.vertical2.setTargetPosition(Position2);
        robot.vertical2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // INTAKE PORTION


    public void openPickUpClaw() {
//        robot.claw.setPosition(0);
    }

    public void closePickUpClaw() {

//        robot.claw.setPosition(1);
    }

    // OUTAKE PORTION
    public void transferRotateToDrop() {
//        robot.vRot1.setPosition(0.3);
//        robot.vRot2.setPosition(0.7);
//        robot.clawRot.setPosition(0);
    }

    public void transferRotateToPick() {
//        robot.vRot1.setPosition(.9);
//        robot.vRot2.setPosition(.1);
//        robot.clawRot.setPosition(1);
    }

    public void transferOpenClaw() {

//        robot.vClaw.setPosition(0);
    }

    public void transferCloseClaw() {

//        robot.vClaw.setPosition(1);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot = new Hware();
        robot.initialize(hardwareMap);

        Pose2d startPose = new Pose2d(0,0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);


        transferCloseClaw();
        openPickUpClaw();
        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(0, ()->{
                    transferCloseClaw();
                })
                .addTemporalMarker(0.2, ()->{
                    changeVerticalPosition(-4093,4071);
                    robot.vertical1.setPower(.8);
                    robot.vertical2.setPower(-.8);

                    transferRotateToDrop();
                })
                //.splineToConstantHeading(new Vector2d(0.1, 0), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-24, -16), Math.toRadians(180))
                .build();

        TrajectorySequence vDOWN = drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(.5)
                .addDisplacementMarker(()->{
                    changeVerticalPosition(-450,450);
                    robot.vertical1.setPower(.8);
                    robot.vertical2.setPower(-.8);
                    transferRotateToDrop();
                })

                .build();

        TrajectorySequence ToGiveHuman = drive.trajectorySequenceBuilder(preload.end())
                .addTemporalMarker(0, ()->{
                    changeVerticalPosition(0,0);
                    robot.vertical1.setPower(1);
                    transferRotateToDrop();
                })
                //.splineToConstantHeading(new Vector2d(0.1, 0), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-17, 23.8759, Math.toRadians(155)))
                .addDisplacementMarker(()->{
                    transferRotateToPick();
                    changeHorizontalPosition(1669);
                    robot.horizontal1.setPower(1);
                    robot.horizontal2.setPower(1);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence putInSection = drive.trajectorySequenceBuilder(ToGiveHuman.end())
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-16, 23.8759, Math.toRadians(30)))
                .build();

        TrajectorySequence getAnother = drive.trajectorySequenceBuilder(putInSection.end())
                .lineToLinearHeading(new Pose2d(-17, 32, Math.toRadians(150)))
                .build();


        TrajectorySequence getFromWall = drive.trajectorySequenceBuilder(putInSection.end())
                .lineToLinearHeading(new Pose2d(-5, 5, Math.toRadians(180)))
                .addDisplacementMarker(()->{
                    transferRotateToPick();
                    changeHorizontalPosition(10);
                    robot.horizontal1.setPower(1);
                    robot.horizontal2.setPower(1);
                })
                .build();

        TrajectorySequence cycle2 = drive.trajectorySequenceBuilder(getFromWall.end())
                .lineToLinearHeading(new Pose2d(-24, -14, Math.toRadians(0)))
                .addDisplacementMarker(()->{
                    changeVerticalPosition(-4093,4071);
                    robot.vertical1.setPower(1);
                    transferRotateToDrop();
                })
                .build();

        TrajectorySequence getFromWall2 = drive.trajectorySequenceBuilder(cycle2.end())
                .lineToLinearHeading(new Pose2d(-5, 5, Math.toRadians(180)))
                .addDisplacementMarker(()->{
                    transferRotateToPick();
                    changeHorizontalPosition(10);
                    robot.horizontal1.setPower(1);
                    robot.horizontal2.setPower(1);
                })
                .build();

        TrajectorySequence cycle3 = drive.trajectorySequenceBuilder(getFromWall2.end())
                .lineToLinearHeading(new Pose2d(-24, -10, Math.toRadians(0)))
                .addDisplacementMarker(()->{
                    changeVerticalPosition(-4093,4071);
                    robot.vertical1.setPower(1);
                    transferRotateToDrop();
                })
                .build();

        TrajectorySequence waitTime = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(.6)
                .build();
        TrajectorySequence waitTime3 = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(.7)
                .build();

        drive.followTrajectorySequence(preload);
        drive.followTrajectorySequence(vDOWN);
        drive.followTrajectorySequence(waitTime);
        transferOpenClaw();
//        drive.followTrajectorySequence(ToGiveHuman);
//        drive.followTrajectorySequence(waitTime);
//        closePickUpClaw();
//        drive.followTrajectorySequence(putInSection);
//        openPickUpClaw();
//        drive.followTrajectorySequence(waitTime);
//        drive.followTrajectorySequence(getAnother);
//        drive.followTrajectorySequence(waitTime);
//        closePickUpClaw();
//        drive.followTrajectorySequence(putInSection);
//        openPickUpClaw();
//        drive.followTrajectorySequence(waitTime);
//        drive.followTrajectorySequence(getFromWall);
//        transferRotateToDrop();
//        drive.followTrajectorySequence(waitTime3);
//        transferCloseClaw();
//        drive.followTrajectorySequence(waitTime);
//        drive.followTrajectorySequence(cycle2);
//        drive.followTrajectorySequence(waitTime3);
//        drive.followTrajectorySequence(vDOWN);
//        drive.followTrajectorySequence(waitTime);
//        transferOpenClaw();

        while (!isStopRequested() && opModeIsActive());

    }
}
