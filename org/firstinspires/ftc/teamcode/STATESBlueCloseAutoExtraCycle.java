package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "STATESBlueCloseAutoExtraCycle (Blocks to Java)")
public class STATESBlueCloseAutoExtraCycle extends LinearOpMode {

  // this is a test
  private CRServo passoffServo;
  private CRServo passOffServo2;
  private BNO055IMU imu;
  private Servo wristServo;
  private Servo toggleServo;
  private Servo outtakePivotServo;
  private DcMotor backLeftMotor;
  private DcMotor backRightMotor;
  private DcMotor frontLeftMotor;
  private DcMotor frontRightMotor;
  private DcMotor leftLiftMotor;
  private DcMotor rightLiftMotor;
  private DcMotor intakeMotor;
  private DistanceSensor propDetectionLeft;
  private DistanceSensor propDetectionRight_DistanceSensor;
  private Servo intakePivotServo;

  String passoffConveyor2;
  int rowNumber;
  double VSPower;
  double OldTargetAngle;
  double newRampTime;
  double errorAngle;
  int error2;
  int FLStartingEncoderPosition;
  float MeasuredAngle;
  double TargetAngle;
  double currentPower;
  int numTicks;
  int FRStartingEncoderPosition;
  double desiredWristAngle0;
  double callRowHeightStartTime;
  double passoffConveyerStartTime;
  double lastRampTime;
  double averageFps;
  double desiredWristAngle1;
  double desiredWristAngle2;
  double desiredWristAngle3;
  double desiredWristAngle4;
  double desiredWristAngle5;
  double desiredWristAngle6;
  double scoringCenter;
  double scoringCentral;
  double scoringSecondary;
  double callRowHeightStartTime2;
  int rowNumber2;
  double VSPower2;
  int row1;
  int row0;
  int row2;
  int row3;
  int row4;
  int row5;
  int row6;
  int row7;
  int row8;
  int row9;
  int row10;
  int row11;
  int row12;
  int outtakeReady4PickupHeight;
  double rampPowerInterval;
  String detectedProp;
  double GyroAdjustment;

  /**
   * RampUp/Down
   */
  private void rampUpRampDownAng(double desiredAngle, double maxDrivePower, double RampPow, int RampAngle) {
    double ROC;

    ROC = (maxDrivePower * 2) / desiredAngle;
    if (Math.abs(errorAngle) >= Math.abs(TargetAngle) - RampAngle) {
      if (currentPower < maxDrivePower) {
        currentPower = currentPower + RampPow;
      }
    } else if (Math.abs(errorAngle) <= RampAngle + 5) {
      if (currentPower > 0.1) {
        currentPower = currentPower - RampPow * 2.5;
      }
    } else {
      if (currentPower > 0.1) {
        currentPower = currentPower;
      }
    }
  }

  /**
   * Describe this function...
   */
  private void yield() {
    if (getRuntime() > callRowHeightStartTime) {
      callRowHeight(rowNumber, VSPower);
      callRowHeightStartTime = 0;
    }
    if (getRuntime() > callRowHeightStartTime2) {
      callRowHeight(rowNumber2, VSPower2);
      callRowHeightStartTime2 = 0;
    }
    if (getRuntime() > passoffConveyerStartTime) {
      passoffConveyor(passoffConveyor2);
      passoffConveyerStartTime = 0;
    }
  }

  /**
   * Describe this function...
   */
  private void passoffConveyor(String passoffConveyor2) {
    if (passoffConveyor2.equals("In")) {
      passoffServo.setDirection(CRServo.Direction.REVERSE);
      passOffServo2.setDirection(CRServo.Direction.FORWARD);
      passoffServo.setPower(1);
      passOffServo2.setPower(0.8);
    } else if (passoffConveyor2.equals("Out")) {
      passoffServo.setDirection(CRServo.Direction.FORWARD);
      passoffServo.setDirection(CRServo.Direction.REVERSE);
      passoffServo.setPower(1);
      passOffServo2.setPower(0.8);
    } else if (passoffConveyor2.equals("Rest")) {
      passoffServo.setDirection(CRServo.Direction.REVERSE);
      passoffServo.setDirection(CRServo.Direction.FORWARD);
      passoffServo.setPower(0);
      passOffServo2.setPower(0);
    } else if (passoffConveyor2.equals("getReady")) {
      passoffServo.setDirection(CRServo.Direction.FORWARD);
      passoffServo.setDirection(CRServo.Direction.REVERSE);
      passoffServo.setPower(1);
      passOffServo2.setPower(0.8);
      sleep(150);
      passoffServo.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void initialize() {
    int INTAKE_TICKS_PER_REVOLUTION;
    int liftRightPosition;
    int liftLeftPosition;
    int baseHeight;
    boolean detectTapeLeft;
    boolean detectTapeRight;

    imu.initialize(new BNO055IMU.Parameters());
    // Verify all initalizations before "TargetAngle=0"
    passoffConveyerStartTime = 0;
    callRowHeightStartTime = 0;
    callRowHeightStartTime2 = 0;
    INTAKE_TICKS_PER_REVOLUTION = 28;
    TargetAngle = 0;
    GyroAdjustment = 0.02;
    liftRightPosition = 0;
    liftLeftPosition = 0;
    baseHeight = 0;
    row0 = 0;
    row1 = 225;
    row2 = 310;
    row3 = 400;
    row4 = 480;
    row5 = 565;
    row6 = 645;
    row7 = 725;
    outtakeReady4PickupHeight = 150;
    // Disabled because does not go to that height
    // The stuff for wrist function
    wristServo.setPosition(0.5105);
    // The stuff for toggle function
    scoringCenter = 0.497;
    scoringSecondary = 0.5125;
    scoringCentral = 0.475;
    toggleServo.setPosition(scoringCenter);
    desiredWristAngle0 = 0.5105;
    desiredWristAngle1 = 0.605;
    desiredWristAngle2 = 0.575;
    desiredWristAngle3 = 0.545;
    desiredWristAngle4 = 0.4875;
    desiredWristAngle5 = 0.452;
    desiredWristAngle6 = 0.415;
    // The stuff for pivot function
    outtakePivotServo.setPosition(0.485);
    backLeftMotor.setTargetPosition(0);
    backRightMotor.setTargetPosition(0);
    frontLeftMotor.setTargetPosition(0);
    frontRightMotor.setTargetPosition(0);
    leftLiftMotor.setTargetPosition(0);
    rightLiftMotor.setTargetPosition(0);
    intakeMotor.setTargetPosition(0);
    leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    rightLiftMotor.setDirection(DcMotor.Direction.REVERSE);
    backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    FourbarPosition("Rest");
    detectTapeLeft = false;
    detectTapeRight = false;
  }

  /**
   * Describe this function...
   */
  private void callRowHeight(double rowNumber, double VSPower) {
    // Make subfunction as a safety function
    if (rowNumber == 1) {
      leftLiftMotor.setTargetPosition(row1);
      rightLiftMotor.setTargetPosition(row1);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 0) {
      leftLiftMotor.setTargetPosition(row0);
      rightLiftMotor.setTargetPosition(row0);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 2) {
      rightLiftMotor.setTargetPosition(row2);
      leftLiftMotor.setTargetPosition(row2);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 3) {
      rightLiftMotor.setTargetPosition(row3);
      leftLiftMotor.setTargetPosition(row3);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 4) {
      rightLiftMotor.setTargetPosition(row4);
      leftLiftMotor.setTargetPosition(row4);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 5) {
      rightLiftMotor.setTargetPosition(row5);
      leftLiftMotor.setTargetPosition(row5);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 6) {
      rightLiftMotor.setTargetPosition(row6);
      leftLiftMotor.setTargetPosition(row6);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 7) {
      rightLiftMotor.setTargetPosition(row7);
      leftLiftMotor.setTargetPosition(row7);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 8) {
      rightLiftMotor.setTargetPosition(row8);
      leftLiftMotor.setTargetPosition(row8);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 9) {
      rightLiftMotor.setTargetPosition(row9);
      leftLiftMotor.setTargetPosition(row9);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 10) {
      rightLiftMotor.setTargetPosition(row10);
      leftLiftMotor.setTargetPosition(row10);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 11) {
      rightLiftMotor.setTargetPosition(row11);
      leftLiftMotor.setTargetPosition(row11);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 12) {
      rightLiftMotor.setTargetPosition(row12);
      leftLiftMotor.setTargetPosition(row12);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 0.5) {
      rightLiftMotor.setTargetPosition(outtakeReady4PickupHeight);
      leftLiftMotor.setTargetPosition(outtakeReady4PickupHeight);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 0.75) {
      rightLiftMotor.setTargetPosition(110);
      leftLiftMotor.setTargetPosition(110);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    } else if (rowNumber == 0.25) {
      rightLiftMotor.setTargetPosition(77);
      leftLiftMotor.setTargetPosition(77);
      rightLiftMotor.setPower(VSPower);
      leftLiftMotor.setPower(VSPower);
    }
  }

  /**
   * We need to add the sensor inputs
   */
  private void propDetection() {
    double leftDetect;
    double midDetect;
    double rightDetect;

    // We need to add the sensor inputs
    leftDetect = propDetectionLeft.getDistance(DistanceUnit.CM);
    midDetect = propDetectionRight_DistanceSensor.getDistance(DistanceUnit.CM);
    rightDetect = propDetectionRight_DistanceSensor.getDistance(DistanceUnit.CM);
    if (leftDetect < 13) {
      detectedProp = "Left";
    } else if (rightDetect < 13) {
      detectedProp = "Right";
    } else {
      detectedProp = "Mid";
    }
  }

  /**
   * Describe this function...
   */
  private void TurnW_ramp(int Angle, double Power) {
    double scaleFactor;
    double normalMeasuredAngle;

    OldTargetAngle = TargetAngle;
    currentPower = 0.05;
    scaleFactor = 250 * (Power / Angle);
    if (OldTargetAngle + Angle > 180) {
      TargetAngle = (OldTargetAngle + Angle) - 360;
    } else if (OldTargetAngle + Angle < -180) {
      TargetAngle = OldTargetAngle + Angle + 360;
    } else {
      TargetAngle = OldTargetAngle + Angle;
    }
    FLStartingEncoderPosition = frontLeftMotor.getCurrentPosition();
    FRStartingEncoderPosition = frontRightMotor.getCurrentPosition();
    frontLeftMotor.setTargetPosition((int) (FLStartingEncoderPosition + 10.6 * Angle));
    frontRightMotor.setTargetPosition((int) (FRStartingEncoderPosition - 10.6 * Angle));
    if (Angle == 0) {
    } else if (Angle > 0) {
      // Turning Right
      while (ReturnMeasuredAngle() < TargetAngle && opModeIsActive()) {
        telemetry.addData("Angle", ReturnMeasuredAngle());
        telemetry.addData("Desired Angle", TargetAngle);
        telemetry.addData("current power", currentPower);
        angleError();
        telemetry.addData("error", errorAngle);
        telemetry.update();
        rampUpRampDownAng(TargetAngle, Power, 0.03, 30);
        backLeftMotor.setPower(currentPower);
        backRightMotor.setPower(-currentPower);
        frontLeftMotor.setPower(currentPower);
        frontRightMotor.setPower(-currentPower);
      }
    } else {
      // Turning Left
      while (ReturnMeasuredAngle() > TargetAngle && opModeIsActive()) {
        telemetry.addData("Angle", ReturnMeasuredAngle());
        telemetry.addData("Desired Angle", TargetAngle);
        telemetry.addData("current power", currentPower);
        angleError();
        telemetry.addData("error", errorAngle);
        rampUpRampDownAng(TargetAngle, Power, 0.03, 30);
        telemetry.update();
        backLeftMotor.setPower(-currentPower);
        backRightMotor.setPower(currentPower);
        frontLeftMotor.setPower(-currentPower);
        frontRightMotor.setPower(currentPower);
      }
    }
    backLeftMotor.setPower(0);
    backRightMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
    telemetry.addData("Angle", ReturnMeasuredAngle());
    angleError();
    telemetry.addData("error", errorAngle);
    telemetry.update();
    if (TargetAngle > 0) {
      normalMeasuredAngle = ReturnMeasuredAngle();
      if (ReturnMeasuredAngle() < 0) {
        normalMeasuredAngle = ReturnMeasuredAngle() + 360;
      }
      // TargetAngle >0
      if (normalMeasuredAngle > TargetAngle + 1) {
        while (normalMeasuredAngle > TargetAngle + 1 && opModeIsActive()) {
          telemetry.addData("TargetAngle", TargetAngle);
          telemetry.addData("ReturMeasuredAngle", ReturnMeasuredAngle());
          telemetry.addData("Normalangle", normalMeasuredAngle);
          telemetry.update();
          frontRightMotor.setPower(0.1);
          backRightMotor.setPower(0.1);
          frontLeftMotor.setPower(-0.1);
          backLeftMotor.setPower(-0.1);
          normalMeasuredAngle = ReturnMeasuredAngle();
          if (ReturnMeasuredAngle() < 0) {
            normalMeasuredAngle = ReturnMeasuredAngle() + 360;
          }
        }
      } else if (normalMeasuredAngle < TargetAngle - 1) {
        while (normalMeasuredAngle < TargetAngle - 1 && opModeIsActive()) {
          telemetry.addData("TargetAngle", TargetAngle);
          telemetry.addData("ReturMeasuredAngle", ReturnMeasuredAngle());
          telemetry.addData("Normalangle", normalMeasuredAngle);
          telemetry.update();
          frontRightMotor.setPower(-0.1);
          backRightMotor.setPower(-0.1);
          frontLeftMotor.setPower(0.1);
          backLeftMotor.setPower(0.1);
          normalMeasuredAngle = ReturnMeasuredAngle();
          if (ReturnMeasuredAngle() < 0) {
            normalMeasuredAngle = ReturnMeasuredAngle() + 360;
          }
        }
      }
    } else {
      // TargetAngle <0
      if (normalMeasuredAngle > TargetAngle + 1) {
        // If Turned a little right
        while (normalMeasuredAngle > TargetAngle + 1 && opModeIsActive()) {
          telemetry.addData("TargetAngle", TargetAngle);
          telemetry.addData("ReturMeasuredAngle", ReturnMeasuredAngle());
          telemetry.addData("Normalangle", normalMeasuredAngle);
          telemetry.update();
          frontRightMotor.setPower(0.1);
          backRightMotor.setPower(0.1);
          frontLeftMotor.setPower(-0.1);
          backLeftMotor.setPower(-0.1);
          normalMeasuredAngle = ReturnMeasuredAngle();
          if (ReturnMeasuredAngle() > 0) {
            normalMeasuredAngle = ReturnMeasuredAngle() - 360;
          }
        }
      } else if (normalMeasuredAngle < TargetAngle - 1) {
        // If Turned a little left
        while (normalMeasuredAngle < TargetAngle - 1 && opModeIsActive()) {
          telemetry.addData("TargetAngle", TargetAngle);
          telemetry.addData("ReturMeasuredAngle", ReturnMeasuredAngle());
          telemetry.addData("Normalangle", normalMeasuredAngle);
          telemetry.update();
          frontRightMotor.setPower(-0.1);
          backRightMotor.setPower(-0.1);
          frontLeftMotor.setPower(0.1);
          backLeftMotor.setPower(0.1);
          normalMeasuredAngle = ReturnMeasuredAngle();
          if (ReturnMeasuredAngle() > 0) {
            normalMeasuredAngle = ReturnMeasuredAngle() - 360;
          }
        }
      }
    }
    backLeftMotor.setPower(0);
    backRightMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
    sleep(200);
    telemetry.addData("TargetAngle", TargetAngle);
    telemetry.addData("ReturMeasuredAngle", ReturnMeasuredAngle());
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void dropOnSpikemark() {
    callRowHeight(1, 0.5);
    sleep(1000);
    outtakePivotServo.setPosition(0.36);
    sleep(1000);
    wristServo.setPosition(0.38);
    sleep(1000);
    outtakePivotServo.setPosition(0.485);
    sleep(1000);
    callRowHeight(0.75, 0.5);
    // We deleted realesepixel, because we wanted to do different steps throughout auto
  }

  /**
   * Describe this function...
   */
  private void rampUpRampDown(int desiredPosition, double maxDrivePower, double RampDistance, double rampPower) {
    newRampTime = getRuntime() - lastRampTime;
    numTicks = numTicks + 1;
    averageFps = (newRampTime + averageFps) / numTicks;
    rampPowerInterval = (newRampTime / 0.06) * rampPower;
    if (Math.abs(error2) <= (RampDistance * 0.5 + RampDistance) * 43) {
      if (currentPower > 0.08) {
        currentPower = currentPower - rampPowerInterval * 0.95;
      } else {
        currentPower = 0.08;
      }
    } else {
      if (currentPower < 0.2) {
        currentPower = currentPower + rampPowerInterval * 2;
      } else if (currentPower < maxDrivePower) {
        currentPower = currentPower + rampPowerInterval * 1;
      }
    }
    lastRampTime = getRuntime();
  }

  /**
   * Describe this function...
   */
  private void setWristAngle(double my_0123456) {
    if (my_0123456 == desiredWristAngle0) {
      wristServo.setPosition(desiredWristAngle0);
    } else if (my_0123456 == desiredWristAngle1) {
      wristServo.setPosition(desiredWristAngle1);
    } else if (my_0123456 == desiredWristAngle2) {
      wristServo.setPosition(desiredWristAngle2);
    } else if (my_0123456 == desiredWristAngle3) {
      wristServo.setPosition(desiredWristAngle3);
    } else if (my_0123456 == desiredWristAngle4) {
      wristServo.setPosition(desiredWristAngle4);
    } else if (my_0123456 == desiredWristAngle5) {
      wristServo.setPosition(desiredWristAngle5);
    } else if (my_0123456 == desiredWristAngle6) {
      wristServo.setPosition(desiredWristAngle6);
    }
  }

  /**
   * Describe this function...
   */
  private void angleError() {
    errorAngle = TargetAngle - ReturnMeasuredAngle();
  }

  /**
   * Describe this function...
   */
  private void TurnRight(int Angle, double Power) {
    OldTargetAngle = TargetAngle;
    if (OldTargetAngle + Angle > 180) {
      TargetAngle = (OldTargetAngle + Angle) - 360;
    } else if (OldTargetAngle + Angle < -180) {
      TargetAngle = OldTargetAngle + Angle + 360;
    } else {
      TargetAngle = OldTargetAngle + Angle;
    }
    FLStartingEncoderPosition = frontLeftMotor.getCurrentPosition();
    FRStartingEncoderPosition = frontRightMotor.getCurrentPosition();
    frontLeftMotor.setTargetPosition((int) (FLStartingEncoderPosition + 10.6 * Angle));
    frontRightMotor.setTargetPosition((int) (FRStartingEncoderPosition - 10.6 * Angle));
    if (Angle == 0) {
    } else if (Angle > 0) {
      // Turning Right
      while (frontLeftMotor.getCurrentPosition() < frontLeftMotor.getTargetPosition() && opModeIsActive()) {
        backLeftMotor.setPower(Power);
        backRightMotor.setPower(-Power);
        frontLeftMotor.setPower(Power);
        frontRightMotor.setPower(-Power);
      }
    } else {
      // Turning Left
      while (frontLeftMotor.getCurrentPosition() > frontLeftMotor.getTargetPosition() && opModeIsActive()) {
        backLeftMotor.setPower(-Power);
        backRightMotor.setPower(Power);
        frontLeftMotor.setPower(-Power);
        frontRightMotor.setPower(Power);
      }
    }
    backLeftMotor.setPower(0);
    backRightMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
    sleep(100);
    if (TargetAngle > 0) {
      // TargetAngle >0
      if (ReturnMeasuredAngle() > TargetAngle + 1 || MeasuredAngle < -90) {
        while ((ReturnMeasuredAngle() > TargetAngle + 1 || MeasuredAngle < -90) && opModeIsActive()) {
          frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          frontRightMotor.setPower(0.1);
          backRightMotor.setPower(0.1);
          frontLeftMotor.setPower(-0.1);
          backLeftMotor.setPower(-0.1);
        }
      } else if (ReturnMeasuredAngle() < TargetAngle - 1 && MeasuredAngle > -90) {
        while (ReturnMeasuredAngle() < TargetAngle - 1 && opModeIsActive()) {
          frontRightMotor.setPower(-0.1);
          backRightMotor.setPower(-0.1);
          frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          frontLeftMotor.setPower(0.1);
          backLeftMotor.setPower(0.1);
        }
      }
    } else {
      // TargetAngle <0
      if (ReturnMeasuredAngle() > TargetAngle + 1 && MeasuredAngle < 90) {
        // If Turned a little right
        while (ReturnMeasuredAngle() > TargetAngle + 1 && opModeIsActive()) {
          frontRightMotor.setPower(0.1);
          backRightMotor.setPower(0.1);
          frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          frontLeftMotor.setPower(-0.1);
          backLeftMotor.setPower(-0.1);
        }
      } else if (ReturnMeasuredAngle() < TargetAngle - 1 || MeasuredAngle > 90) {
        // If Turned a little left
        while ((ReturnMeasuredAngle() < TargetAngle - 1 || MeasuredAngle > 90) && opModeIsActive()) {
          frontRightMotor.setPower(-0.1);
          backRightMotor.setPower(-0.1);
          frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          frontLeftMotor.setPower(0.1);
          backLeftMotor.setPower(0.1);
        }
      }
    }
    frontRightMotor.setPower(0);
    backRightMotor.setPower(0);
    frontLeftMotor.setPower(0);
    backLeftMotor.setPower(0);
    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  /**
   * Describe this function...
   */
  private void runIntake(String runIntakeInOut) {
    if (runIntakeInOut.equals("In")) {
      intakeMotor.setPower(1);
      intakeMotor.setTargetPosition(2000);
    } else if (runIntakeInOut.equals("Out")) {
      intakeMotor.setPower(-1);
      intakeMotor.setTargetPosition(-2000);
    } else if (runIntakeInOut.equals("Rest")) {
      intakeMotor.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void error() {
    error2 = frontLeftMotor.getTargetPosition() - frontLeftMotor.getCurrentPosition();
  }

  /**
   * Describe this function...
   */
  private void DriveFWDwRamp(double Distance, double Power, double rampPower, double RampDistance) {
    FLStartingEncoderPosition = frontLeftMotor.getCurrentPosition();
    FRStartingEncoderPosition = frontRightMotor.getCurrentPosition();
    currentPower = 0.1;
    frontLeftMotor.setTargetPosition((int) (FLStartingEncoderPosition + 43 * Distance));
    frontRightMotor.setTargetPosition((int) (FRStartingEncoderPosition + 43 * Distance));
    lastRampTime = getRuntime();
    if (Distance > 0) {
      // Running Forward
      while (frontLeftMotor.getCurrentPosition() < frontLeftMotor.getTargetPosition()) {
        yield();
        telemetry.addData("current position", frontLeftMotor.getCurrentPosition());
        telemetry.addData("current power", currentPower);
        telemetry.addData("RMPpowerint", rampPowerInterval);
        error();
        telemetry.addData("NumTicks", numTicks);
        telemetry.addData("DesiredPower", Power);
        telemetry.addData("error", error2);
        telemetry.addData("averageFps", averageFps);
        telemetry.addData("time", newRampTime);
        telemetry.addData("Target Angle", TargetAngle);
        telemetry.addData("measured angle", ReturnMeasuredAngle());
        rampUpRampDown(frontLeftMotor.getTargetPosition(), Power, RampDistance, rampPower);
        telemetry.update();
        if (TargetAngle > 0) {
          // If we are moving forward and aiming angle>0
          if (ReturnMeasuredAngle() > TargetAngle || ReturnMeasuredAngle() < -90) {
            backLeftMotor.setPower(currentPower - GyroAdjustment);
            frontLeftMotor.setPower(currentPower - GyroAdjustment);
            backRightMotor.setPower(currentPower);
            frontRightMotor.setPower(currentPower);
          } else {
            backLeftMotor.setPower(currentPower);
            frontLeftMotor.setPower(currentPower);
            backRightMotor.setPower(currentPower - GyroAdjustment);
            frontRightMotor.setPower(currentPower - GyroAdjustment);
          }
        } else {
          // If we are moving forward and aiming angle<0
          if (ReturnMeasuredAngle() > TargetAngle && ReturnMeasuredAngle() < 90) {
            backLeftMotor.setPower(currentPower - GyroAdjustment);
            frontLeftMotor.setPower(currentPower - GyroAdjustment);
            backRightMotor.setPower(currentPower);
            frontRightMotor.setPower(currentPower);
          } else {
            backLeftMotor.setPower(currentPower);
            frontLeftMotor.setPower(currentPower);
            backRightMotor.setPower(currentPower - GyroAdjustment);
            frontRightMotor.setPower(currentPower - GyroAdjustment);
          }
        }
      }
      lastRampTime = 0;
      numTicks = 0;
    } else {
      // Running Backward
      while (frontLeftMotor.getCurrentPosition() > frontLeftMotor.getTargetPosition()) {
        yield();
        telemetry.addData("current position", frontLeftMotor.getCurrentPosition());
        telemetry.addData("current power", currentPower);
        telemetry.addData("NumTicks", numTicks);
        error();
        telemetry.addData("error", error2);
        rampUpRampDown(frontLeftMotor.getTargetPosition(), Power, RampDistance, rampPower);
        telemetry.update();
        if (TargetAngle > 0) {
          // If we are moving backward and aiming angle>0
          if (ReturnMeasuredAngle() > TargetAngle && ReturnMeasuredAngle() < -90) {
            backLeftMotor.setPower(-currentPower);
            frontLeftMotor.setPower(-currentPower);
            backRightMotor.setPower(-currentPower + GyroAdjustment);
            frontRightMotor.setPower(-currentPower + GyroAdjustment);
          } else {
            backLeftMotor.setPower(-currentPower + GyroAdjustment);
            frontLeftMotor.setPower(-currentPower + GyroAdjustment);
            backRightMotor.setPower(-currentPower);
            frontRightMotor.setPower(-currentPower);
          }
        } else {
          // If we are moving backwardz and aiming angle<0
          if (ReturnMeasuredAngle() > TargetAngle && ReturnMeasuredAngle() < 90) {
            backLeftMotor.setPower(-currentPower);
            frontLeftMotor.setPower(-currentPower);
            backRightMotor.setPower(-currentPower + GyroAdjustment);
            frontRightMotor.setPower(-currentPower + GyroAdjustment);
          } else {
            backLeftMotor.setPower(-currentPower + GyroAdjustment);
            frontLeftMotor.setPower(-currentPower + GyroAdjustment);
            backRightMotor.setPower(-currentPower);
            frontRightMotor.setPower(-currentPower);
          }
        }
      }
      lastRampTime = 0;
      numTicks = 0;
    }
    backLeftMotor.setPower(0);
    backRightMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
    error();
    telemetry.addData("Target Angle", TargetAngle);
    telemetry.addData("measured angle", ReturnMeasuredAngle());
    telemetry.addData("current position", frontLeftMotor.getCurrentPosition());
    telemetry.addData("error", error2);
    telemetry.addData("averageFps", averageFps);
    telemetry.addData("done", 1);
    telemetry.update();
    frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition());
    frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition());
  }

  /**
   * We need to change the heights-these are the ones from last year
   */
  private void FourbarPosition(String IntakeRest12) {
    // Remember to change the servo after configuring robot!
    if (IntakeRest12.equals("Intake")) {
      intakePivotServo.setPosition(0.51);
    } else if (IntakeRest12.equals("Rest")) {
      intakePivotServo.setPosition(0.445);
    } else if (IntakeRest12.equals("1")) {
      intakePivotServo.setPosition(0.5);
    } else if (IntakeRest12.equals("2")) {
      intakePivotServo.setPosition(0.5);
    }
  }

  /**
   * Describe this function...
   */
  private void releasePixel(boolean central, boolean secondary, boolean center) {
    // TODO: Enter the type for variable named my_0123456
    UNKNOWN_TYPE my_0123456;

    if (center == true) {
      toggleServo.setPosition(scoringCenter);
    } else if (central == true && secondary == true) {
      if (my_0123456 == (desiredWristAngle2 || desiredWristAngle5)) {
        toggleServo.setPosition(scoringCentral);
        sleep(500);
        toggleServo.setPosition(scoringSecondary);
        sleep(500);
        toggleServo.setPosition(scoringCenter);
      } else if (my_0123456 == (desiredWristAngle3 || desiredWristAngle4)) {
        toggleServo.setPosition(scoringCentral);
        sleep(500);
        toggleServo.setPosition(scoringSecondary);
        sleep(500);
        toggleServo.setPosition(scoringCenter);
      } else {
        toggleServo.setPosition(scoringCentral);
        sleep(500);
        toggleServo.setPosition(scoringSecondary);
        sleep(500);
        toggleServo.setPosition(scoringCenter);
      }
      central = false;
      secondary = false;
    } else if (central == true) {
      toggleServo.setPosition(scoringCentral);
      sleep(500);
      toggleServo.setPosition(scoringCenter);
      central = false;
    } else if (secondary == true) {
      toggleServo.setPosition(scoringSecondary);
      sleep(500);
      toggleServo.setPosition(scoringCenter);
      secondary = false;
    }
  }

  /**
   * Describe this function...
   */
  private void StrafeRight(double Distance, double Power) {
    FLStartingEncoderPosition = frontLeftMotor.getCurrentPosition();
    FRStartingEncoderPosition = frontRightMotor.getCurrentPosition();
    frontLeftMotor.setTargetPosition((int) (FLStartingEncoderPosition + 49 * Distance));
    frontRightMotor.setTargetPosition((int) (FRStartingEncoderPosition - 49 * Distance));
    if (Distance > 0) {
      // Running to the Right
      while (frontLeftMotor.getCurrentPosition() < frontLeftMotor.getTargetPosition()) {
        backLeftMotor.setPower(-Power);
        backRightMotor.setPower(Power);
        frontLeftMotor.setPower(Power);
        frontRightMotor.setPower(-Power);
      }
    } else {
      // Running to the Left
      while (frontLeftMotor.getCurrentPosition() > frontLeftMotor.getTargetPosition()) {
        backLeftMotor.setPower(Power);
        backRightMotor.setPower(-Power);
        frontLeftMotor.setPower(-Power);
        frontRightMotor.setPower(Power);
      }
    }
    backLeftMotor.setPower(0);
    backRightMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
  }

  /**
   * Describe this function...
   */
  private double ReturnMeasuredAngle() {
    MeasuredAngle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    return MeasuredAngle;
  }

  /**
   * Describe this function...
   */
  private void pivotAngle(int pivotOrientation) {
    if (pivotOrientation == 0) {
      outtakePivotServo.setPosition(0.485);
    } else if (pivotOrientation == 1) {
      outtakePivotServo.setPosition(0.4);
    }
  }

  /**
   * Describe this function...
   */
  private void Drive_FWD(double Distance, double Power) {
    FLStartingEncoderPosition = frontLeftMotor.getCurrentPosition();
    FRStartingEncoderPosition = frontRightMotor.getCurrentPosition();
    frontLeftMotor.setTargetPosition((int) (FLStartingEncoderPosition + 43 * Distance));
    frontRightMotor.setTargetPosition((int) (FRStartingEncoderPosition + 43 * Distance));
    if (Distance > 0) {
      // Running Forward
      while (frontLeftMotor.getCurrentPosition() < frontLeftMotor.getTargetPosition()) {
        if (TargetAngle > 0) {
          // If we are moving forward and aiming angle>0
          if (ReturnMeasuredAngle() > TargetAngle || ReturnMeasuredAngle() < -90) {
            backLeftMotor.setPower(Power - GyroAdjustment);
            frontLeftMotor.setPower(Power - GyroAdjustment);
            backRightMotor.setPower(Power);
            frontRightMotor.setPower(Power);
          } else {
            backLeftMotor.setPower(Power);
            frontLeftMotor.setPower(Power);
            backRightMotor.setPower(Power - GyroAdjustment);
            frontRightMotor.setPower(Power - 0.05);
          }
        } else {
          // If we are moving forward and aiming angle<0
          if (ReturnMeasuredAngle() > TargetAngle && ReturnMeasuredAngle() < 90) {
            backLeftMotor.setPower(Power - GyroAdjustment);
            frontLeftMotor.setPower(Power - GyroAdjustment);
            backRightMotor.setPower(Power);
            frontRightMotor.setPower(Power);
          } else {
            backLeftMotor.setPower(Power);
            frontLeftMotor.setPower(Power);
            backRightMotor.setPower(Power - GyroAdjustment);
            frontRightMotor.setPower(Power - GyroAdjustment);
          }
        }
      }
    } else {
      // Running Backward
      while (frontLeftMotor.getCurrentPosition() > frontLeftMotor.getTargetPosition()) {
        if (TargetAngle > 0) {
          // If we are moving backward and aiming angle>0
          if (ReturnMeasuredAngle() > TargetAngle || MeasuredAngle < -90) {
            backLeftMotor.setPower(-Power);
            frontLeftMotor.setPower(-Power);
            backRightMotor.setPower(-Power + GyroAdjustment);
            frontRightMotor.setPower(-Power + GyroAdjustment);
          } else {
            backLeftMotor.setPower(-Power + GyroAdjustment);
            frontLeftMotor.setPower(-Power + GyroAdjustment);
            backRightMotor.setPower(-Power);
            frontRightMotor.setPower(-Power);
          }
        } else {
          // If we are moving backwardz and aiming angle<0
          if (ReturnMeasuredAngle() > TargetAngle && ReturnMeasuredAngle() < 90) {
            backLeftMotor.setPower(-Power);
            frontLeftMotor.setPower(-Power);
            backRightMotor.setPower(-Power + GyroAdjustment);
            frontRightMotor.setPower(-Power + GyroAdjustment);
          } else {
            backLeftMotor.setPower(-Power + GyroAdjustment);
            frontLeftMotor.setPower(-Power + GyroAdjustment);
            backRightMotor.setPower(-Power);
            frontRightMotor.setPower(-Power);
          }
        }
      }
    }
    backLeftMotor.setPower(0);
    backRightMotor.setPower(0);
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
    frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition());
  }

  /**
   * Describe this function...
   */
  private void compressing() {
    setWristAngle(desiredWristAngle0);
    sleep(500);
    pivotAngle(0);
    sleep(1000);
    callRowHeight(0, 0.5);
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
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

    // Put initialization blocks here.
    initialize();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        callRowHeight(1, 0.5);
        DriveFWDwRamp(36.5, 1, 0.033, 16.22);
        TurnRight(0, 0.5);
        outtakePivotServo.setPosition(0.36);
        propDetection();
        telemetry.addData("Left", propDetectionLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("Right", propDetectionRight_DistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
        if (detectedProp.equals("Left")) {
          Drive_FWD(-8, 0.5);
          TurnW_ramp(-87, 1);
          Drive_FWD(0.25, 0.3);
          // Process for drop on spikemark
          callRowHeight(0.25, 0.5);
          sleep(200);
          outtakePivotServo.setPosition(0.396);
          sleep(150);
          releasePixel(false, true, false);
          sleep(150);
          outtakePivotServo.setPosition(0.36);
          callRowHeight(1, 0.5);
          // Process ends
          StrafeRight(-12, 0.45);
          pivotAngle(1);
          TurnRight(0, 0.4);
          DriveFWDwRamp(37.8, 0.8, 0.033, 16.62);
          setWristAngle(desiredWristAngle1);
          TurnRight(0, 0.4);
          StrafeRight(4, 0.2);
          // Score on backdrop
          Drive_FWD(0.5, 0.2);
          releasePixel(true, true, false);
          Drive_FWD(-2, 0.2);
          setWristAngle(desiredWristAngle0);
          TurnRight(0, 0.4);
          pivotAngle(0);
          // Get out the way & park
          StrafeRight(34, 0.5);
          callRowHeight(0, 0.5);
          TurnRight(0, 0.4);
          DriveFWDwRamp(-105.2, 1, 0.035, 40);
          intakePivotServo.setPosition(0.5);
          callRowHeight(0.5, 0.5);
          setWristAngle(desiredWristAngle0);
          TurnRight(0, 0.5);
          passoffConveyor("In");
          runIntake("In");
          Drive_FWD(-4, 0.35);
          sleep(1000);
          runIntake("Out");
          passoffConveyor("In");
          sleep(150);
          passoffConveyor("In");
          runIntake("Rest");
          FourbarPosition("Rest");
          passoffConveyerStartTime = getRuntime() + 1;
          callRowHeightStartTime = getRuntime() + 1;
          rowNumber = 0;
          VSPower = 0.3;
          passoffConveyor2 = "Rest";
          callRowHeightStartTime2 = getRuntime() + 1.8;
          rowNumber2 = 2;
          VSPower2 = 0.15;
          DriveFWDwRamp(108, 1, 0.04, 36);
          TurnRight(0, 0.4);
          pivotAngle(1);
          StrafeRight(-30, 0.5);
          setWristAngle(desiredWristAngle1);
          TurnRight(0, 0.4);
          Drive_FWD(2, 0.5);
          releasePixel(true, true, false);
          sleep(100);
          Drive_FWD(-2, 0.5);
          compressing();
          TurnRight(90, 0.5);
          telemetry.update();
          sleep(30000);
        } else if (detectedProp.equals("Right")) {
          Drive_FWD(-6, 0.5);
          sleep(100);
          TurnW_ramp(87, 1);
          Drive_FWD(1, 0.5);
          TurnRight(0, 0.5);
          sleep(100);
          // Process for drop on spikemark
          callRowHeight(0.25, 0.5);
          sleep(500);
          outtakePivotServo.setPosition(0.396);
          sleep(250);
          releasePixel(false, true, false);
          sleep(250);
          outtakePivotServo.setPosition(0.36);
          // Process ends
          sleep(250);
          // Score on backdrop
          callRowHeight(1, 0.5);
          sleep(250);
          pivotAngle(1);
          Drive_FWD(-3, 0.5);
          TurnW_ramp(-175, 0.7);
          DriveFWDwRamp(35, 1, 0.033, 16.62);
          setWristAngle(desiredWristAngle1);
          Drive_FWD(4, 0.2);
          StrafeRight(7.5, 0.2);
          releasePixel(true, true, false);
          Drive_FWD(-2, 0.2);
          // Compress & get out the way
          compressing();
          StrafeRight(-32, 0.5);
          TurnRight(89, 0.5);
        } else {
          Drive_FWD(-6, 0.4);
          // Process for drop on spikemark
          callRowHeight(0.25, 0.5);
          sleep(200);
          outtakePivotServo.setPosition(0.396);
          sleep(150);
          releasePixel(false, true, false);
          sleep(150);
          outtakePivotServo.setPosition(0.36);
          // process ends
          // Score on backdrop
          Drive_FWD(-2, 0.5);
          callRowHeight(1, 0.5);
          TurnW_ramp(-87, 1);
          pivotAngle(1);
          callRowHeight(1, 0.4);
          DriveFWDwRamp(37.8, 0.8, 0.033, 16.62);
          setWristAngle(desiredWristAngle1);
          TurnRight(0, 0.4);
          Drive_FWD(0.5, 0.2);
          releasePixel(true, true, false);
          Drive_FWD(-2, 0.2);
          setWristAngle(desiredWristAngle0);
          TurnRight(0, 0.4);
          pivotAngle(0);
          StrafeRight(26, 0.5);
          callRowHeight(0, 0.5);
          TurnRight(0, 0.4);
          DriveFWDwRamp(-105.2, 1, 0.035, 40);
          intakePivotServo.setPosition(0.5);
          callRowHeight(0.5, 0.5);
          setWristAngle(desiredWristAngle0);
          TurnRight(0, 0.5);
          passoffConveyor("In");
          runIntake("In");
          Drive_FWD(-4, 0.35);
          sleep(1000);
          runIntake("Out");
          passoffConveyor("In");
          sleep(150);
          passoffConveyor("In");
          runIntake("Rest");
          FourbarPosition("Rest");
          passoffConveyerStartTime = getRuntime() + 1;
          callRowHeightStartTime = getRuntime() + 1;
          rowNumber = 0;
          VSPower = 0.3;
          passoffConveyor2 = "Rest";
          callRowHeightStartTime2 = getRuntime() + 1.8;
          rowNumber2 = 2;
          VSPower2 = 0.3;
          DriveFWDwRamp(108, 1, 0.04, 36);
          TurnRight(0, 0.4);
          pivotAngle(1);
          StrafeRight(-30, 0.5);
          setWristAngle(desiredWristAngle1);
          TurnRight(0, 0.4);
          Drive_FWD(2, 0.5);
          releasePixel(true, true, false);
          sleep(100);
          Drive_FWD(-2, 0.5);
          compressing();
          TurnRight(90, 0.5);
          telemetry.update();
          sleep(30000);
        }
        sleep(30000);
        telemetry.update();
      }
    }
  }
}
