package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;


public class RobotHardware {

  public static CRServo passoffServo;
  public static CRServo passOffServo2;
  public static BNO055IMU imu;
  public static Servo wristServo;
  public static Servo toggleServo;
  public static Servo outtakePivotServo;
  public static DcMotor backLeftMotor;
  public static DcMotor backRightMotor;
  public static DcMotor frontLeftMotor;
  public static DcMotor frontRightMotor;
  public static DcMotor leftLiftMotor;
  public static DcMotor rightLiftMotor;
  public static DcMotor intakeMotor;
  public static DistanceSensor propDetectionLeft;
  public static DistanceSensor propDetectionRight_DistanceSensor;
  public static Servo intakePivotServo;

  public static void initialize(HardwareMap hardwareMap) {
    passoffServo = hardwareMap.get(CRServo.class, "passoffServo");
    passOffServo2 = hardwareMap.get(CRServo.class, "passOffServo2");
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    wristServo = hardwareMap.get(Servo.class, "wristServo");
    toggleServo = hardwareMap.get(Servo.class, "toggleServo");
    outtakePivotServo = hardwareMap.get(Servo.class, "outtakePivotServo");
    backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
    backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
    frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
    leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftMotor");
    rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLiftMotor");
    intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    propDetectionLeft = hardwareMap.get(DistanceSensor.class, "propDetectionLeft");
    propDetectionRight_DistanceSensor = hardwareMap.get(DistanceSensor.class, "propDetectionRight");
    intakePivotServo = hardwareMap.get(Servo.class, "intakePivotServo");
  }


}