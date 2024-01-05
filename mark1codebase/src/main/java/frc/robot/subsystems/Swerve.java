package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.kauailabs.navx.frc.*;

public class Swerve extends SubsystemBase {

    private double robotHeading;

    private double frontLeftPowerDraw;
    private double frontRightPowerDraw;
    private double backLeftPowerDraw;
    private double backRightPowerDraw;

    private double frontLeftFreeSpeed;
    private double frontRightFreeSpeed;
    private double backLeftFreeSpeed;
    private double backRightFreeSpeed;

    private double frontLeftDriveMotorTemp;
    private double frontLeftTurnMotorTemp;
    private double frontRightDriveMotorTemp;
    private double frontRightTurnMotorTemp;
    private double backLeftDriveMotorTemp;
    private double backLeftTurnMotorTemp;
    private double backRightDriveMotorTemp;
    private double backRightTurnMotorTemp;

    private final SwerveModule frontLeft = new SwerveModule(
            "FrontLeft",
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            "FrontRight",
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            "BackLeft",
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            "BackRight",
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);


    public  Swerve(){
        new Thread(() -> {
        try {
                Thread.sleep(1000);
                zeroHeading();
        } catch (Exception e){};
        
    }).start();
}

    
    public SwerveModulePosition[] getModulePositions()
    {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()};
    }

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    private final SwerveDriveOdometry odo = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions());

    public void zeroHeading(){
        gyro.reset();
    }

    public void resetYaw()
    {
        gyro.zeroYaw();
    }

    public void calibrateGyro()
    {
        gyro.calibrate();
    }

    public double getHeading(){

        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Pose2d getOdometryMeters() {
        return (odo.getPoseMeters());
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void stopModules(){

        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleState( SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);

    }

    public Pose2d getPose(){
        return odo.getPoseMeters();
    }

    public void resetPose(Pose2d pose){
        odo.resetPosition(getOdometryAngle(), getModulePositions(), pose);
    }

    public void resetPose(Pose2d pose, Rotation2d rot){
        odo.resetPosition(rot, getModulePositions(), pose);
    }

    public Rotation2d getOdometryAngle(){
        return(Rotation2d.fromDegrees(gyro.getYaw()));
    }

    public double getRobotDegrees(){
        double rawValue = gyro.getAngle() % 360;
        if (rawValue < 0.0){
            return(rawValue + 360);
        } else {
            return(rawValue);
        }
    }

    public void resetAllEncoders(){
        frontLeft.restEncoders();;
        frontRight.restEncoders();
        backLeft.restEncoders();
        backRight.restEncoders();
    }

    public double getRumble()
    {
        return gyro.getRawAccelX();
    }

    public double getRollChange(){
        return gyro.getRawGyroY();
    }

    public double getRoll(){
        return gyro.getRoll();
    }

    public double getForce() {
        return gyro.getRawAccelX() * 9.8 * DriveConstants.kRobotWeight;
    }

    public double[] getModuleMotorTemps() {
        return new double[] { frontLeft.getDriveMotorTemp(), frontRight.getDriveMotorTemp(),
                backLeft.getDriveMotorTemp(), backRight.getDriveMotorTemp(),
                frontLeft.getTurnMotorTemp(), frontRight.getTurnMotorTemp(), backLeft.getTurnMotorTemp(),
                backRight.getTurnMotorTemp() };
    }

    public double[] getModulePowerDraws() {
        return new double[] { frontLeft.getPowerDraw(), frontRight.getPowerDraw(), backLeft.getPowerDraw(),
                backRight.getPowerDraw() };
    }

    public double[] getModuleDriveVelocities() {
        return new double[] { frontLeft.getDriveVelocity(), frontRight.getDriveVelocity(), backLeft.getDriveVelocity(),
                backRight.getDriveVelocity() };
    }

    @Override
    public void periodic()
    {
        odo.update(getOdometryAngle(), getModulePositions());

        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Field Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("ROBOT DEGREES NAVX", getRobotDegrees());
        SmartDashboard.putString("ODOMETRY", odo.getPoseMeters().toString());
        SmartDashboard.putString("Raw R2d ROBOT DEG", getOdometryAngle().toString());

        SmartDashboard.putBoolean("Gyro Calibrating", gyro.isCalibrating());
        SmartDashboard.putBoolean("Magnetic Issues", gyro.isMagneticDisturbance());
        SmartDashboard.putBoolean("Magnetic Calibartion", gyro.isMagnetometerCalibrated());

        SmartDashboard.putNumber("Robot Acceleration X", gyro.getRawAccelX());
        SmartDashboard.putNumber("Robot Acceleration Y", gyro.getRawAccelY());
        SmartDashboard.putNumber("Robot Force X Newtons", 57.0 * 9.8 * gyro.getRawAccelX());
        SmartDashboard.putNumber("Robot Force X Pounds", (57.0 * 9.8 * gyro.getRawAccelX()) / 4.45);

        SmartDashboard.putNumber("RAW ROLL", getRoll());
        SmartDashboard.putNumber("RAW Y", getRollChange());

        SmartDashboard.putNumber("Turning Position:",frontLeft.getTurningPosition());

        frontLeft.update();
        frontRight.update();
        backLeft.update();
        backRight.update();

        // System.out.println(backRight.getAbsoluteEncoder());
        // System.out.println(DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad);

        // System.out.println(backLeft.getState());

        robotHeading = getHeading();

        frontLeftPowerDraw = frontLeft.getPowerDraw();
        frontRightPowerDraw = frontRight.getPowerDraw();
        backLeftPowerDraw = backLeft.getPowerDraw();
        backRightPowerDraw = backRight.getPowerDraw();

        frontLeftFreeSpeed = frontLeft.getDriveVelocity();
        frontRightFreeSpeed = frontRight.getDriveVelocity();
        backLeftFreeSpeed = backLeft.getDriveVelocity();
        backRightFreeSpeed = backRight.getDriveVelocity();

        frontLeftDriveMotorTemp = frontLeft.getDriveMotorTemp();
        frontRightDriveMotorTemp = frontRight.getDriveMotorTemp();
        backLeftDriveMotorTemp = backLeft.getDriveMotorTemp();
        backRightDriveMotorTemp = backRight.getDriveMotorTemp();

        frontLeftTurnMotorTemp = frontLeft.getTurnMotorTemp();
        frontRightTurnMotorTemp = frontRight.getTurnMotorTemp();
        backLeftTurnMotorTemp = backLeft.getTurnMotorTemp();
        backRightTurnMotorTemp = backRight.getTurnMotorTemp();
    }
}
