// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.module.ModuleDescriptor.Builder;
import java.util.function.Supplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Camera;
import frc.robot.Constants;
import frc.robot.Camera.CameraType;
import static frc.robot.Constants.*;


public class VisionSubsystem extends SubsystemBase {


    private Translation2d robotToTag;
    private Translation2d cameraToTag;
    private double alpha;
    private Field2d field;


    private NetworkTable Table;
    private double camToTagYaw;
    private double camToTagPitch;
    private double id;
    private double height;
    private double dist;
    private Translation2d originToRobot;
    private Translation2d origintoTag;
    private Translation2d robotToTagRR;
    public Pose2d pose;
    private Translation2d robotToTagFC;


    private Camera camera;
    private AHRS gyro;


    public VisionSubsystem(Camera camera) {
        super();
        this.camera = camera;
        Table = NetworkTableInstance.getDefault().getTable(camera.getTableName());
        field = new Field2d();


        gyro = new AHRS(NavXComType.kMXP_SPI);
        gyro.reset();
        SmartDashboard.putData("Vision", this);
        SmartDashboard.putData("Field", field);
        
    }

    public Field2d getField2d() {
        return field;
    }

    @Override
    public void periodic() {
        camToTagPitch = Table.getEntry("ty").getDouble(0.0);
        camToTagYaw = (-Table.getEntry("tx").getDouble(0.0)) + camera.getYaw();
        id = getTagId();

        if (Table.getEntry("tv").getDouble(0.0) != 0) {
            if (id > 0 && id < TAG_HEIGHT.length) {
                pose = new Pose2d(getOriginToRobot(), getAngle());
                field.setRobotPose(pose);
            }
        } else {
            pose = null;
        }
        addCommands();

        // SmartDashboard.putNumber("YAW", getAngle().getDegrees());
        // System.out.println(gyro.isConnected());


    }
    public int getTagId() {
        return (int) Table.getEntry("tid").getDouble(0.0);
    }

    public double getDistFromCamera() {
        alpha = camToTagPitch + camera.getPitch();
        height = TAG_HEIGHT[(int) id];
        // if (camera.getCameraType() == CameraType.REEF) {
        //     dist = (Math.abs(height - camera.getHeight())) * Math.tan(Math.toRadians(alpha));
        //     dist = dist / Math.cos(Math.toRadians(camToTagYaw));
        //     return Math.abs(dist);
        // }
        alpha = camToTagPitch + camera.getPitch();
        dist = (Math.abs(height - camera.getHeight())) / Math.tan(Math.toRadians(alpha));
        // dist = dist / Math.cos(Math.toRadians(camToTagYaw));
        return Math.abs(dist);
    } 

    public Translation2d getRobotToTagRR() {
        cameraToTag = new Translation2d(getDistFromCamera(),
                Rotation2d.fromDegrees(camToTagYaw));
        robotToTag = new Translation2d(camera.getRobotToCamPosition().getX(),
                camera.getRobotToCamPosition().getY()).plus(cameraToTag);
        return robotToTag;
    }

    public Translation2d getCameraToTag() {
        return new Translation2d(getDistFromCamera(),
            Rotation2d.fromDegrees(camToTagYaw));
    }

    public Translation2d getOriginToRobot() {

        origintoTag = O_TO_TAG[(int) this.id == -1 ? 0 : (int) this.id];
    
        height = TAG_HEIGHT[(int) this.id];
        if (origintoTag != null) {
        //   Get vector from robot to tag
          robotToTagRR = getRobotToTagRR();
    
        robotToTagFC = robotToTagRR.rotateBy(getAngle());
          originToRobot = origintoTag.plus(robotToTagFC.rotateBy(Rotation2d.kPi));
    
          return originToRobot;
        }
        return new Translation2d();
    }


    public Pose2d getPose() {
        return this.pose;
    }




    public boolean isSeeTag(int id, double distance) {
        return Table.getEntry("tid").getDouble(0.0) == id && getRobotToTagRR().getNorm() <= distance;
    }

    public boolean isSeeTag() {
        return Table.getEntry("tid").getDouble(0.0) > 0;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw() - Constants.gyroOffset);
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    
        // Your existing entries (fixed syntax)
        builder.addDoubleProperty("Tag ID", () -> id, null);
        builder.addDoubleProperty("Tag Dist", () -> getDistFromCamera(), null);
        builder.addDoubleProperty("Tag Yaw", () -> camToTagYaw, null);
        builder.addDoubleProperty("Tag Pitch", () -> camToTagPitch, null);
        builder.addBooleanProperty("See Tag", () -> isSeeTag(), null);
        builder.addDoubleProperty("Robot X", () -> getOriginToRobot().getX(), null);
        builder.addDoubleProperty("Robot Y", () -> getOriginToRobot().getY(), null);
    
        // Add gyro diagnostics
        builder.addBooleanProperty("Gyro Connected", () -> gyro.isConnected(), null);
        builder.addBooleanProperty("Gyro Calibrating", () -> gyro.isCalibrating(), null);
        builder.addDoubleProperty("Gyro Yaw", () -> gyro.getYaw(), null);
        
   }

}
