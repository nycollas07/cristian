package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "AUDIENCE SIDE DIREITO")
public class AudienceSideDireito extends LinearOpMode{
    OpenCvWebcam webcam = null; //declarar a webcam
    String linha;
    private DcMotor left_intake;
    private DcMotor right_intake;
    private Servo LeftOuttakebraço;
    private Servo RightOuttakebraço;
    private Servo LeftOuttakecaixa;
    private Servo RightOuttakecaixa;
    private Servo Dispenser;
    private DcMotor right_linear;
    private DcMotor left_linear;

    @Override
    public void runOpMode(){
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); //mapeamento da webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId); //declaração do id da webcam

        webcam.setPipeline(new Pipeline()); //definindo o pipeline

        left_intake = hardwareMap.get(DcMotor.class, "left_intake_EX3");
        right_intake = hardwareMap.get(DcMotor.class, "right_intake_C3");

        left_intake.setDirection(DcMotorSimple.Direction.REVERSE);
        right_intake.setDirection(DcMotorSimple.Direction.REVERSE);

        left_linear = hardwareMap.get(DcMotor.class, "left_linear_EX2");
        right_linear = hardwareMap.get(DcMotor.class, "right_linear_C2");

        left_linear.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftOuttakebraço = hardwareMap.get(Servo.class, "Left-Outtake-braço");
        RightOuttakebraço = hardwareMap.get(Servo.class, "Right-Outtake-braço");
        LeftOuttakecaixa = hardwareMap.get(Servo.class, "Left-Outtake-caixa");
        RightOuttakecaixa = hardwareMap.get(Servo.class, "Right-Outtake-caixa");
        Dispenser = hardwareMap.get(Servo.class, "Dispenser");

        RightOuttakecaixa.setDirection(Servo.Direction.REVERSE);
        LeftOuttakebraço.setDirection(Servo.Direction.REVERSE);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT
                ); //definindo o tamanho de tela da sua câmera
            } //logitech c920 fullHD
            @Override
            public void onError(int errorCode) {

            }
        });

        int stage = 1;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-35, -62.6, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        /////////////RIGHT//////////////////
        TrajectorySequence trajectyoryRIGHT1 = drive.trajectorySequenceBuilder(startPose)
                .forward(15)
                .lineToLinearHeading(new Pose2d(-40, -39, Math.toRadians(180)))
                .back(16)
                .forward(7)
                .build();
        TrajectorySequence trajectyoryRIGHT2 = drive.trajectorySequenceBuilder(trajectyoryRIGHT1.end())
                .forward(15)
                .build();
        TrajectorySequence trajectyoryRIGHT3 = drive.trajectorySequenceBuilder(trajectyoryRIGHT2.end())
                .strafeRight(25)
                .build();
        TrajectorySequence trajectyoryRIGHT4 = drive.trajectorySequenceBuilder(trajectyoryRIGHT3.end())
                .back(55)
                .build();
        TrajectorySequence trajectyoryRIGHT5 = drive.trajectorySequenceBuilder(trajectyoryRIGHT4.end())
                .lineToLinearHeading(new Pose2d(30, -43, Math.toRadians(180)))
                .back(8 ,  SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence trajectyoryRIGHT6 = drive.trajectorySequenceBuilder(trajectyoryRIGHT5.end())
                .forward(7)
                .build();

        TrajectorySequence trajectyoryRIGHT7 = drive.trajectorySequenceBuilder(trajectyoryRIGHT6.end())
                .strafeRight(25)
                .back(9)
                .build();

        ///////////////LEFT///////////////////
        TrajectorySequence trajectyoryLEFT1 = drive.trajectorySequenceBuilder(startPose)
                .forward(14)
                .lineToLinearHeading(new Pose2d(-36, -39, Math.toRadians(0)))
                .back(7)
                .forward(7)
                .build();
        TrajectorySequence trajectyoryLEFT2 = drive.trajectorySequenceBuilder(trajectyoryLEFT1.end())
                .back(2)
                .strafeLeft(24)
                .forward(47)
                .build();
        TrajectorySequence trajectyoryLEFT3 = drive.trajectorySequenceBuilder(trajectyoryLEFT2.end())
                .lineToLinearHeading(new Pose2d(28, -32.5, Math.toRadians(0)))
                .turn(Math.toRadians(179))
                .back(11,  SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence trajectyoryLEFT4 = drive.trajectorySequenceBuilder(trajectyoryLEFT3.end())
                .forward(7)
                .build();
        TrajectorySequence trajectyoryLEFT5 = drive.trajectorySequenceBuilder(trajectyoryLEFT4.end())
                .strafeRight(25)
                .back(9)
                .build();
        ///////////////////CENTER///////////////////////
        TrajectorySequence trajectyoryCENTER1 = drive.trajectorySequenceBuilder(startPose)
                .forward( 41)
                .build();
        TrajectorySequence backCenter = drive.trajectorySequenceBuilder(trajectyoryCENTER1.end())
                .back(1)
                .build();
        TrajectorySequence trajectyoryCENTER2 = drive.trajectorySequenceBuilder(backCenter.end())
                .turn(Math.toRadians(-90))
                .forward(47)
                .build();
        TrajectorySequence trajectyoryCENTER3 = drive.trajectorySequenceBuilder(trajectyoryCENTER2.end())
                .lineToLinearHeading(new Pose2d(35, -44.6, Math.toRadians(0)))
                .turn(Math.toRadians(179))
                .build();
        TrajectorySequence trajectyoryCENTER4 = drive.trajectorySequenceBuilder(trajectyoryCENTER3.end())
                .back(12,  SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        TrajectorySequence trajectyoryCENTER5 = drive.trajectorySequenceBuilder(trajectyoryCENTER4.end())
                .forward(7)
                .build();
        TrajectorySequence trajectyoryCENTER6 = drive.trajectorySequenceBuilder(trajectyoryCENTER5.end())
                .strafeRight(24)
                .back(9)
                .build();



        waitForStart();
        LeftOuttakebraço.setPosition(0.07);
        RightOuttakebraço.setPosition(0.11);
        LeftOuttakecaixa.setPosition(0.15);
        RightOuttakecaixa.setPosition(0.18);
        while(opModeIsActive()) {


            telemetry.addData("ENCODER", left_intake.getCurrentPosition());
            telemetry.update();

            if(stage == 1) {
                if (linha == "Esquerda"){
                    //ESQUERDA
                    webcam.stopStreaming();

                    drive.followTrajectorySequence(trajectyoryLEFT1);

                    DepositarOuttake();
                    sleep(700);
                    DepositarDispenser();
                    sleep(700);

                    ColetarOuttake();

                    drive.followTrajectorySequence(trajectyoryLEFT2);

                    DepositarOuttake();

                    drive.followTrajectorySequence(trajectyoryLEFT3);

                    linear(1030);
                    sleep(1200);

                    DepositarDispenser();
                    sleep(1000);

                    drive.followTrajectorySequence(trajectyoryLEFT4);

                    linear_descer(0);
                    ColetarOuttake();

                    drive.followTrajectorySequence(trajectyoryLEFT5);

                }else if(linha == "Meio"){
                    webcam.stopStreaming();

                    drive.followTrajectorySequence(trajectyoryCENTER1);

                    DepositarOuttake();

                    sleep(700);

                    DepositarDispenser();
                    sleep(1000);

                    sleep(500);

                    drive.followTrajectorySequence(backCenter);

                    ColetarOuttake();

                    sleep(1000);

                    drive.followTrajectorySequence(trajectyoryCENTER2);

                    drive.followTrajectorySequence(trajectyoryCENTER3);

                    DepositarOuttake();

                    drive.followTrajectorySequence(trajectyoryCENTER4);

                    linear(1030);
                    sleep(1200);

                    DepositarDispenser();
                    drive.followTrajectorySequence(trajectyoryCENTER5);

                    linear_descer(0);
                    ColetarOuttake();

                    drive.followTrajectorySequence(trajectyoryCENTER6);

                }else{
                    webcam.stopStreaming();

                    drive.followTrajectorySequence(trajectyoryRIGHT1);

                    DepositarOuttake();
                    sleep(700);
                    DepositarDispenser();
                    sleep(700);

                    drive.followTrajectorySequence(trajectyoryRIGHT2);

                    ColetarOuttake();

                    drive.followTrajectorySequence(trajectyoryRIGHT3);
                    drive.followTrajectorySequence(trajectyoryRIGHT4);

                    DepositarOuttake();

                    drive.followTrajectorySequence(trajectyoryRIGHT5);

                    linear(1030);
                    sleep(1200);

                    DepositarDispenser();
                    sleep(1000);

                    drive.followTrajectorySequence(trajectyoryRIGHT6);

                    linear_descer(0);
                    ColetarOuttake();

                    drive.followTrajectorySequence(trajectyoryRIGHT7);
                }
                stage = 2;
            }

        }

    }
    private void linear( int rotations) {
        left_linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_linear.setTargetPosition(rotations);
        right_linear.setTargetPosition(rotations);
        left_linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_linear.setPower(0.8);
        left_linear.setPower(0.8);
        while (left_linear.isBusy()) {
        }
    }
    private void intake( int rotations) {
        left_intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_intake.setTargetPosition(rotations);
        right_intake.setTargetPosition(rotations);
        left_intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_intake.setPower(1);
        left_intake.setPower(1);
        while (left_intake.isBusy()) {
        }
    }
    private void intakeVoltar( int rotations) {
        left_intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_intake.setTargetPosition(rotations);
        right_intake.setTargetPosition(rotations);
        left_intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_intake.setPower(-0.6);
        left_intake.setPower(-0.6);
        while (left_intake.isBusy()) {
        }
    }
    private void linear_descer( int rotations) {
        left_linear.setTargetPosition(rotations);
        right_linear.setTargetPosition(rotations);
        left_linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_linear.setPower(-0.8);
        left_linear.setPower(-0.8);
        while (left_linear.isBusy()) {
        }
    }
    private void DepositarDispenser(){
        Dispenser.setPosition(0);
        sleep(300);
        Dispenser.setPosition(0.5);
    }
    private void DepositarOuttake(){
        LeftOuttakebraço.setPosition(0.81);
        RightOuttakebraço.setPosition(0.97);
        LeftOuttakecaixa.setPosition(0.75);
        RightOuttakecaixa.setPosition(0.8);
    }
    private void ColetarOuttake(){
        LeftOuttakebraço.setPosition(0.07);
        RightOuttakebraço.setPosition(0.11);
        LeftOuttakecaixa.setPosition(0.15);
        RightOuttakecaixa.setPosition(0.18);
    }

    class Pipeline extends OpenCvPipeline{  //contrução do piperline
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat midCrop;
        double leftavgfin;
        double rightavgfin;
        double midavgfin;
        Mat output = new Mat();
        Scalar rectColor = new Scalar(0.0, 0.0, 255.0);
        //função proscess frame, irá detectar a localização geral de um objeto
        public Mat processFrame(Mat input){
            // e dividir o nosso espaço de vizualização em 3 quadrantes
            //parametros de altura e largura de cada quadrante
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2HSV); //converter o feedback da camera para YCbCr, deixa o desempenho melhor por otimiza-lo

            Rect leftRect = new Rect(1, 0, 550, 400);
            Rect rightRect = new Rect(1280, 0, 550, 400);
            Rect midRect = new Rect(640, 0, 550, 400);

            input.copyTo(output);
            Imgproc.rectangle(output, leftRect, rectColor, 2);
            Imgproc.rectangle(output, rightRect, rectColor, 2); //esse código permite voce vizualizar os
            //quadrantes na sua driver hub, muito util para testes e verificação
            Imgproc.rectangle(output, midRect, rectColor, 2);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            midCrop = YCbCr.submat(midRect);

            Core.extractChannel(leftCrop, leftCrop, 1);
            Core.extractChannel(rightCrop, rightCrop, 1);
            Core.extractChannel(midCrop, midCrop, 1);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar midavg = Core.mean(midCrop);

            leftavgfin = leftavg.val[0];  //linhas de escaneamento
            rightavgfin = rightavg.val[0];
            midavgfin = midavg.val[0];

            telemetry.update();

            if (leftavgfin > rightavgfin && leftavgfin > midavgfin){ // linhas para printar em qual quadrante o
                //objeto está sendo identificado
                telemetry.addLine("Esquerda");
                linha = "Esquerda";
            } else if (rightavgfin > midavgfin){
                telemetry.addLine("Direita");
                linha = "Direita";
            } else{
                telemetry.addLine("Meio");
                linha = "Meio";
            }

            return (output);
        }
    }

}