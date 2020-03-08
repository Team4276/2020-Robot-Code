package frc.systems.sensors;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;

public class Cameras {

	public UsbCamera mainCamera;
	// UsbCamera armCamera;

	private final int MAIN_RES_X = 128		;
	private final int MAIN_RES_Y = 96;
	private final int MAIN_FPS = 22;
	private final int MAIN_EXPOSURE = -4;


	public Cameras() {
		
		mainCamera = CameraServer.getInstance().startAutomaticCapture(0);

		mainCamera.setResolution(MAIN_RES_X, MAIN_RES_Y);
		//mainCamera.setFPS(MAIN_FPS);
		//mainCamera.setExposureAuto();
		mainCamera.setExposureManual(MAIN_EXPOSURE);
		mainCamera.setExposureHoldCurrent();
		
	}

}
