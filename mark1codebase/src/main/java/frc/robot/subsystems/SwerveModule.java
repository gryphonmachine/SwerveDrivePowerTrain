package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {

    private final String moduleName;
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveCoder;
    private final RelativeEncoder turnCoder;

    private final PIDController turnPidController;

    private final CANCoder canCoder;
    private final boolean canReverse;
    private final double canOffset;

    public SwerveModule(String name, int driveMotorID, int turnMotorID, boolean driveReverse, boolean turnReverse, int canCoderID, double canCoderOffset, boolean canCoderReverse ){

        this.moduleName = name;
        this.canReverse = canCoderReverse;
        this.canOffset = canCoderOffset;
        canCoder = new CANCoder(canCoderID);

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        driveMotor.setInverted(driveReverse);
        turnMotor.setInverted(turnReverse);

        turnPidController = new PIDController(0.4,0.1,0.01);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);

        driveCoder = driveMotor.getEncoder();
        turnCoder = turnMotor.getEncoder();

        driveCoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveCoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turnCoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turnCoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        restEncoders();
    }

    public void update()
    {
        SmartDashboard.putNumber(moduleName + "Absolute Position" , canCoder.getAbsolutePosition());
    }



    public double getDrivePosition() {
        return driveCoder.getPosition();
    }

    public double getTurningPosition() {
        return getAbsoluteEncoder();
    }

    public double getDriveVelocity() {
        return driveCoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turnCoder.getVelocity();
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsoluteEncoder()));
    }

    public double getAbsoluteEncoder(){

        double angle = Math.toRadians(canCoder.getAbsolutePosition());
        // angle *= (2*Math.PI)/180;
        angle -= canOffset;

        return angle*(canReverse ? -1.0 : 1.0);
    }

    public void restEncoders(){
            
        driveCoder.setPosition(0);
        turnCoder.setPosition(getAbsoluteEncoder());
        }

    public double getPowerDraw() {
        return RobotController.getBatteryVoltage() * (driveMotor.getOutputCurrent() + turnMotor.getOutputCurrent());
    }

    public double getDriveMotorTemp() {
        return driveMotor.getMotorTemperature();
    }

    public double getTurnMotorTemp() {
        return turnMotor.getMotorTemperature();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsoluteEncoder()));
    }

    public void setDesiredState(SwerveModuleState state) {

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turnMotor.set(turnPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        
        // SmartDashboard.putString("Swerve[" + canCoder.configGetSensorDirection() null)
    }

    public void stop()
    {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
