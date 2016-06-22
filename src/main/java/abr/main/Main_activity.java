/**
 * Rescue Robotics 2016 App
 * Developed by Cognitive Anteater Robotics Laboratory at University of California, Irvine
 * Controls wheeled robot through IOIO
 * Parts of code adapted from OpenCV blob follow
 * Before running, connect phone to IOIO with a bluetooth connection
 * If you would like to uncomment sections for message passing, first connect peer phones using wifi direct
 */
package abr.main;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Calendar;
import java.util.List;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.google.android.gms.common.ConnectionResult;
import com.google.android.gms.common.api.GoogleApiClient;
import com.google.android.gms.common.api.GoogleApiClient.ConnectionCallbacks;
import com.google.android.gms.common.api.GoogleApiClient.OnConnectionFailedListener;
import com.google.android.gms.location.LocationListener;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationServices;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.Result;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import ioio.lib.util.IOIOLooper;
import ioio.lib.util.IOIOLooperProvider;
import ioio.lib.util.android.IOIOAndroidApplicationHelper;
import android.graphics.Bitmap;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.media.AudioManager;
import android.media.ToneGenerator;
import android.os.Bundle;
import android.os.Environment;
import android.os.SystemClock;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.util.Log;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import abr.main.R;

public class Main_activity extends Activity implements IOIOLooperProvider, SensorEventListener, ConnectionCallbacks, OnConnectionFailedListener,
		CvCameraViewListener2 // implements IOIOLooperProvider: from IOIOActivity
{
	private final IOIOAndroidApplicationHelper helper_ = new IOIOAndroidApplicationHelper(this, this); // from IOIOActivity
	
	boolean redRobot = true; //account for different gear ratios
	int forwardSpeed;
	int turningSpeed;
	int obstacleTurningSpeed;
	
	// ioio variables
	IOIO_thread m_ioio_thread;
	
	//blob detection variables
	private CameraBridgeViewBase mOpenCvCameraView;
	private Mat mRgba;
	private Scalar mBlobColorRgba;
	private ColorBlobDetector mDetector;
	private Mat mSpectrum;
	private Scalar CONTOUR_COLOR;
	
	//app state variables
	private boolean autoMode;
	
	//variables for logging
	private Sensor mGyroscope;
	private Sensor mGravityS;
	float[] mGravityV;
	float[] mGyro;

	//location variables
	private GoogleApiClient mGoogleApiClient;
	private double curr_lat;
	private double curr_lon;
	private Location curr_loc;
	private LocationRequest mLocationRequest;
	private LocationListener mLocationListener;
	Location dest_loc;
	float distance = 0;
	
	//variables for compass
	private SensorManager mSensorManager;
	private Sensor mCompass, mAccelerometer;
	float[] mGravity;
	float[] mGeomagnetic;
	public float heading = 0;
	public float bearing;

	//ui variables
	TextView sonar1Text;
	TextView sonar2Text;
	TextView sonar3Text;
	TextView distanceText;
	TextView bearingText;
	TextView headingText;
	
	//sockets for message passing
	Boolean isClient = true;
	ServerSocket serverSocket;
	Socket socket;
	Socket clientSocket;
	DataInputStream dataInputStream;
	DataOutputStream dataOutputStream;
	
	//occupancy grid variables
	int numSweeps = 0;
	int[][] occupancyGrid;
	Location[][] gridLocations;
	Location centerLocation;
	Location topLeft;
	Location bottomRight;
	int gridSize = 2;
	int sendCounter = 0;
	int destRow = 0;
	int destCol = 0;
	
	//timers
	int pauseCounter = 0;
	int backCounter = 0;
	Long startTime;
	Long currTime;
	
	//pan/tilt
	int panVal=1500;
	int tiltVal=1500;
	boolean panningRight = false;
	boolean tiltingUp = false;
	int panInc;
	int tiltInc;
	
	// called to use OpenCV libraries contained within the app as opposed to a separate download
	static {
		if (!OpenCVLoader.initDebug()) {
			// Handle initialization error
		}
	}
	
	// called whenever the activity is created
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		
		requestWindowFeature(Window.FEATURE_NO_TITLE);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		setContentView(R.layout.main);
		
		helper_.create(); // from IOIOActivity
		
		//set up opencv camera
		mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
		mOpenCvCameraView.setCvCameraViewListener(this);
		mOpenCvCameraView.enableView();

		//initialize textviews
		sonar1Text = (TextView) findViewById(R.id.sonar1);
		sonar2Text = (TextView) findViewById(R.id.sonar2);
		sonar3Text = (TextView) findViewById(R.id.sonar3);
		distanceText = (TextView) findViewById(R.id.distanceText);
		bearingText = (TextView) findViewById(R.id.bearingText);
		headingText = (TextView) findViewById(R.id.headingText);
		
		//add functionality to autoMode button
		Button buttonAuto = (Button) findViewById(R.id.btnAuto);
		buttonAuto.setOnClickListener(new OnClickListener() {
			public void onClick(View v) {
				if (!autoMode) {
					v.setBackgroundResource(R.drawable.button_auto_on);
					autoMode = true;
					startTime = System.currentTimeMillis();
				} else {
					v.setBackgroundResource(R.drawable.button_auto_off);
					autoMode = false;
				}
			}
		});
		
		//set starting autoMode button color
		if (autoMode) {
			buttonAuto.setBackgroundResource(R.drawable.button_auto_on);
		} else {
			buttonAuto.setBackgroundResource(R.drawable.button_auto_off);
		}
		
		 //set up occupancy grid
	    numSweeps = 0;
	    occupancyGrid = new int[gridSize][gridSize];
	    topLeft = new Location(""); //define the topLeft corner of field
	    topLeft.setLatitude(33.6469238);
	    topLeft.setLongitude(-117.8430502);
	    bottomRight = new Location(""); //define the bottomRight corner of field
	    bottomRight.setLatitude(33.6468030);
	    bottomRight.setLongitude(-117.8429104);
	    centerLocation = new Location(""); //calculate the center point of the field
	    centerLocation.setLatitude((bottomRight.getLatitude()+topLeft.getLatitude())/2);
	    centerLocation.setLongitude((bottomRight.getLongitude()+topLeft.getLongitude())/2);
	    Log.i("rescue robotics","center,lat:"+centerLocation.getLatitude()+"lon:"+centerLocation.getLongitude());
	    gridLocations = calculateGridLocations(topLeft, bottomRight, gridSize);
	    
	    //set destination
	    destRow = (int)Math.floor(Math.random()*gridSize); 
	    destCol = (int)Math.floor(Math.random()*gridSize);
	    dest_loc = gridLocations[destRow][destCol];
		
		//set up location listener
		mLocationListener = new LocationListener() {
			public void onLocationChanged(Location location) {
				curr_loc = location;
				distance = location.distanceTo(dest_loc);
				bearing = location.bearingTo(dest_loc);
			}
			@SuppressWarnings("unused")
			public void onStatusChanged(String provider, int status, Bundle extras) {
			}
			@SuppressWarnings("unused")
			public void onProviderEnabled(String provider) {
			}
			@SuppressWarnings("unused")
			public void onProviderDisabled(String provider) {
			}
		};
		
		//set up compass
		mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
	    mCompass= mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
	    mAccelerometer= mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
	    mGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
	    mGravityS = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);

	    //set up sockets for communication with other robots
	    /*
	    if (isClient) {
	    	try {
	    		Object[] objects = (new FileClientAsyncTask()).execute().get(); 
	    		socket = (Socket) objects[0];
	    		dataOutputStream = (DataOutputStream) objects[1];
	    		dataInputStream = (DataInputStream) objects[2];
	    	} catch(Exception e) {
	    		Log.e("rescue robotics", e.getMessage());
	    	}  	
	    }
	    else {
	    	try {
	    		Object[] objects = (new FileServerAsyncTask()).execute().get();
	    		serverSocket = (ServerSocket) objects[0];
	    		clientSocket = (Socket) objects[1];
	    		dataInputStream = (DataInputStream) objects[2];
	    		dataOutputStream = (DataOutputStream) objects[3];
	    	} catch(Exception e) {
	    		Log.e("rescue robotics", e.getMessage());
	    	}
	    }
	    */
		// phone must be Android 2.3 or higher and have Google Play store
		// must have Google Play Services: https://developers.google.com/android/guides/setup
		buildGoogleApiClient();
		mLocationRequest = new LocationRequest();
	    mLocationRequest.setInterval(2000);
	    mLocationRequest.setFastestInterval(500);
	    mLocationRequest.setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY);
	    
	    //set speeds. adjust accordingly for your robot
	    if(redRobot){
	    	forwardSpeed = 180;
	    	turningSpeed = 100;
	    	obstacleTurningSpeed = 100;
	    } else {
	    	forwardSpeed = 110 ;
	    	turningSpeed = 80;
	    	obstacleTurningSpeed = 90;
	    }
	}
	//Method necessary for google play location services
	protected synchronized void buildGoogleApiClient() {
	    mGoogleApiClient = new GoogleApiClient.Builder(this)
	        .addConnectionCallbacks(this)
	        .addOnConnectionFailedListener(this)
	        .addApi(LocationServices.API)
	        .build();
	}
	//Method necessary for google play location services
	@Override
    public void onConnected(Bundle connectionHint) {
        // Connected to Google Play services
		curr_loc = LocationServices.FusedLocationApi.getLastLocation(mGoogleApiClient);
	    startLocationUpdates();
    }
	//Method necessary for google play location services
	protected void startLocationUpdates() {
	    LocationServices.FusedLocationApi.requestLocationUpdates(mGoogleApiClient, mLocationRequest, mLocationListener);
	}
	//Method necessary for google play location services
    @Override
    public void onConnectionSuspended(int cause) {
        // The connection has been interrupted.
        // Disable any UI components that depend on Google APIs
        // until onConnected() is called.
    }
    //Method necessary for google play location services
    @Override
    public void onConnectionFailed(ConnectionResult result) {
        // This callback is important for handling errors that
        // may occur while attempting to connect with Google.
        //
        // More about this in the 'Handle Connection Failures' section.
    }
    @Override
	public final void onAccuracyChanged(Sensor sensor, int accuracy) {
		// Do something here if sensor accuracy changes.
	}
    
    //Called whenever the value of a sensor changes
	@Override
	public final void onSensorChanged(SensorEvent event) {
		 if(m_ioio_thread != null){
			  setText("sonar1: "+m_ioio_thread.get_sonar1_reading(), sonar1Text);
			  setText("sonar2: "+m_ioio_thread.get_sonar2_reading(), sonar2Text);
			  setText("sonar3: "+m_ioio_thread.get_sonar3_reading(), sonar3Text);
			  setText("distance: "+distance, distanceText);
			  setText("bearing: "+bearing, bearingText);
			  setText("heading: "+heading, headingText);
		  }
		 
		  if (event.sensor.getType() == Sensor.TYPE_GRAVITY)
			  mGravityV = event.values;
		  if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE)
			  mGyro = event.values;
		  if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
		      mGravity = event.values;
		  if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
		      mGeomagnetic = event.values;
		  if (mGravity != null && mGeomagnetic != null) {
			  float[] temp = new float[9];
			  float[] R = new float[9];
			  //Load rotation matrix into R
			  SensorManager.getRotationMatrix(temp, null, mGravity, mGeomagnetic);
			  //Remap to camera's point-of-view
			  SensorManager.remapCoordinateSystem(temp, SensorManager.AXIS_X, SensorManager.AXIS_Z, R);
			  //Return the orientation values
			  float[] values = new float[3];
			  SensorManager.getOrientation(R, values);
			  //Convert to degrees
			  for (int i=0; i < values.length; i++) {
				  Double degrees = (values[i] * 180) / Math.PI;
				  values[i] = degrees.floatValue();
			  }
			  //Update the compass direction
			  heading = values[0]+12;
			  heading = (heading*5 + fixWraparound(values[0]+12))/6; //add 12 to make up for declination in Irvine, average out from previous 2 for smoothness
		   }
	  }

	//Scan for QR code and save information to phone
	public String scan(Mat frame) {
		Bitmap bMap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
		Utils.matToBitmap(frame, bMap);
		int[] intArray = new int[bMap.getWidth()*bMap.getHeight()];  
		//copy pixel data from the Bitmap into the 'intArray' array  
		bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());  

		LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(),intArray);

		BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
		Reader reader = new QRCodeReader();     
	    
		String text; 
		
	    try {
			Result result = reader.decode(bitmap);
			text = result.getText();
			Calendar calendar = Calendar.getInstance();
			java.util.Date now = calendar.getTime();
			java.sql.Timestamp currentTimestamp = new java.sql.Timestamp(now.getTime());
			String time = currentTimestamp.toString();
			curr_lat = curr_loc.getLatitude();
			curr_lon = curr_loc.getLongitude();
			String info = text +" ,Lat:"+curr_lat+" ,Lon:"+curr_lon+" ,Time:"+time;
			try {
			    File newFolder = new File(Environment.getExternalStorageDirectory(), "RescueRobotics");
			    if (!newFolder.exists()) {
			        newFolder.mkdir();
			    }
			    try {
			        File file = new File(newFolder, time + ".txt");
			        file.createNewFile();
			        FileOutputStream fos=new FileOutputStream(file);
	                try {
	                	byte[] b = info.getBytes();
	                    fos.write(b);
	                    fos.close();
	                    ToneGenerator toneG = new ToneGenerator(AudioManager.STREAM_ALARM, 100);
	        			toneG.startTone(ToneGenerator.TONE_CDMA_PIP, 200);
	                } catch (IOException e) {
	                	Log.e("app.main","Couldn't write to SD");
	                }
			    } catch (Exception ex) {
			    	Log.e("app.main","Couldn't write to SD");
			    }
			} catch (Exception e) {
			    Log.e("app.main","Couldn't write to SD");
			}
			Log.i("rescue robotics",text);
			return text;
		} catch (NotFoundException e) {
			e.printStackTrace();
			text = "no code found";
		} catch (ChecksumException e) {
			e.printStackTrace();
			text =  "checksum error";
		} catch (FormatException e) {
			e.printStackTrace();
			text = "format error";
		}
	    Log.i("rescue robotics",text);

		return text;
	}
	
	//Called whenever activity resumes from pause
	@Override
	public void onResume() {
		super.onResume();
	    if (mOpenCvCameraView != null)
			mOpenCvCameraView.enableView();
	    mSensorManager.registerListener(this, mCompass, SensorManager.SENSOR_DELAY_NORMAL);
	    mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
	    mSensorManager.registerListener(this, mGyroscope, SensorManager.SENSOR_DELAY_NORMAL);
	    mSensorManager.registerListener(this, mGravityS, SensorManager.SENSOR_DELAY_NORMAL);
	    if (mGoogleApiClient.isConnected()) {
	        startLocationUpdates();
	    }
	}
	
	//Called when activity pauses
	@Override
	public void onPause() {
		super.onPause();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
		mSensorManager.unregisterListener(this);
		stopLocationUpdates();
	}
	
	protected void stopLocationUpdates() {
	    LocationServices.FusedLocationApi.removeLocationUpdates(mGoogleApiClient, mLocationListener);
	}
	
	//Called when activity restarts. onCreate() will then be called
	@Override
	public void onRestart() {
		super.onRestart();
		Log.i("activity cycle","main activity restarting");
	}

	//Called when camera view starts. change bucket color here
	public void onCameraViewStarted(int width, int height) {
		mRgba = new Mat(height, width, CvType.CV_8UC4);
		mDetector = new ColorBlobDetector();
		mSpectrum = new Mat();
		mBlobColorRgba = new Scalar(255);
		CONTOUR_COLOR = new Scalar(255, 0, 0, 255);

		//To set color, find HSV values of desired color and convert each value to 1-255 scale
		//mDetector.setHsvColor(new Scalar(7, 196, 144)); // red
		mDetector.setHsvColor(new Scalar(253.796875,222.6875,195.21875));
	}
	//Called when camera view stops
	public void onCameraViewStopped() {
		mRgba.release();
	}
	//Called at every camera frame. Main controls of the robot movements are in this function
	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
		/*
		if(sendCounter > 100){
			sendGrid();
			sendCounter = 0;
		}
		getGrid();
		sendCounter++;
		*/
		
		mRgba = inputFrame.rgba();
		mDetector.process(mRgba);
		
		List<MatOfPoint> contours = mDetector.getContours();
		// Log.e("rescue robotics", "Contours count: " + contours.size());
		Imgproc.drawContours(mRgba, contours, -1, CONTOUR_COLOR);

		Mat colorLabel = mRgba.submat(4, 68, 4, 68);
		colorLabel.setTo(mBlobColorRgba);

		Mat spectrumLabel = mRgba.submat(4, 4 + mSpectrum.rows(), 70,
				70 + mSpectrum.cols());
		mSpectrum.copyTo(spectrumLabel);

		// majority of control exists here
		if (autoMode && (System.currentTimeMillis()-startTime < 600000)) { // only move if autoMode is on and time under time limit
			if(System.currentTimeMillis()-startTime > 6000000)
				autoMode = false;
			scan(mRgba);
			if(backCounter > 0){
				m_ioio_thread.set_steering(1500);
				m_ioio_thread.set_speed(1500-forwardSpeed);
				panVal = 1500;
				tiltVal = 1500;
				m_ioio_thread.set_pan(panVal);
				m_ioio_thread.set_tilt(tiltVal);
				backCounter--;
				if (backCounter == 0)
					pauseCounter = 5;
			}
			else if(pauseCounter > 0){
				m_ioio_thread.set_speed(1500);
				m_ioio_thread.set_steering(1500);
				panVal = 1500;
				tiltVal = 1500;
				m_ioio_thread.set_pan(panVal);
				m_ioio_thread.set_tilt(tiltVal);
				pauseCounter--;
				if (pauseCounter == 0){ // change destination
					if(System.currentTimeMillis()-startTime > 570000){ // go to center at 9:30
						dest_loc = centerLocation;
					}
					else {
						occupancyGrid[destRow][destCol]++;
						/*
						int minVal = Integer.MAX_VALUE;
						int minRow = 0;
						int minCol = 0;
						for(int r = destRow - 1; r < destRow + 2; r++){
							for(int c = destCol - 1; c < destCol + 1; c++){
								if(r > -1 && r < gridSize && c > -1 && c < gridSize && r!=c)
									if(occupancyGrid[r][c] < minVal){
										minVal = occupancyGrid[r][c];
										minRow = r;
										minCol = c;
									}
							}
						}
						dest_loc = gridLocations[minRow][minCol];
						*/
						destRow = (int)Math.floor(Math.random()*gridSize); 
					    destCol = (int)Math.floor(Math.random()*gridSize);
					    dest_loc = gridLocations[destRow][destCol];
					}
				}
			}
			else if(m_ioio_thread.get_sonar2_reading() < 25 && m_ioio_thread != null) {
				if(curr_loc.distanceTo(dest_loc) <= 7 && m_ioio_thread.get_sonar2_reading() < 20 && (mDetector.getMaxArea()/(mDetector.getCenterX()*mDetector.getCenterY()*4) > .01)) //bucket reached
					backCounter = 5;
				else{ //avoiding obstacle
					if(m_ioio_thread.get_sonar1_reading() < m_ioio_thread.get_sonar3_reading()){ //obstacle on left
						m_ioio_thread.set_speed(1500+forwardSpeed);
						m_ioio_thread.set_steering(1500+obstacleTurningSpeed);
					} else { //obstacle on right
						m_ioio_thread.set_speed(1500+forwardSpeed);
						m_ioio_thread.set_steering(1500-obstacleTurningSpeed);
					}
						
				}
				panVal = 1500;
				tiltVal = 1500;
				m_ioio_thread.set_pan(panVal);
				m_ioio_thread.set_tilt(tiltVal);
			}
			else if(curr_loc.distanceTo(dest_loc) > 7) { // follow compass
				float bearingMod = bearing%360;
				float headingMod = heading%360;
				
				m_ioio_thread.set_speed(1500+forwardSpeed);
				if (bearingMod >= headingMod) {
					if (bearingMod - headingMod <= 180)
						m_ioio_thread.set_steering(1500+turningSpeed);
					else
						m_ioio_thread.set_steering(1500-turningSpeed);
					}
				else {
					if (headingMod - bearingMod <= 180)
						m_ioio_thread.set_steering(1500-turningSpeed);
					else
						m_ioio_thread.set_steering(1500+turningSpeed);
				}
				panVal = 1500;
				tiltVal = 1500;
				m_ioio_thread.set_pan(panVal);
				m_ioio_thread.set_tilt(tiltVal);
			} else { // follow orange bucket
				double momentX = mDetector.getMomentX();
				double momentY = mDetector.getMomentY();
				
				if(mDetector.blobsDetected() == 0){
					Log.i("rescue robotics","frame");
					if(panningRight){
						m_ioio_thread.set_pan(panVal+=30);
						if(panVal >= 2400)
							panningRight = false;
					} else {
						m_ioio_thread.set_pan(panVal-=30);
						if(panVal <= 600)
							panningRight = true;
					}
					tiltVal = 1500;
					m_ioio_thread.set_tilt(1500);
				} else {
					panInc = 40 + (int)Math.exp(.03*Math.abs(momentX));
					if(momentX > 25){
						m_ioio_thread.set_pan(panVal+=panInc);
					}
					else if(momentX < -25){
						m_ioio_thread.set_pan(panVal-=panInc);
					}
					tiltInc = 20 + (int)Math.exp(.03*Math.abs(momentY));
					if(momentY > 25){
						m_ioio_thread.set_tilt(tiltVal+=tiltInc);
					}
					else if(momentY < -25){
						m_ioio_thread.set_tilt(tiltVal-=tiltInc);
					}
				}
				
				if(panVal > 2400) panVal = 2400;
				if(panVal < 600) panVal = 600;
				if(tiltVal > 2000) tiltVal = 2000;
				if(tiltVal < 1000) tiltVal = 1000;
				
				if (mDetector.blobsDetected() > 0) {
					if(!(panVal < 1550 && panVal > 1450)){
						m_ioio_thread.set_speed(1500+forwardSpeed);
						if(panVal>1500){
							m_ioio_thread.set_steering(1500+turningSpeed);
						}
						else {
							m_ioio_thread.set_steering(1500-turningSpeed);
						}
					}
					else{
						m_ioio_thread.set_steering(1500);
						m_ioio_thread.set_speed(1500+forwardSpeed);
					}
				} else  {
					m_ioio_thread.set_steering(1500);
					m_ioio_thread.set_speed(1500);
				}
				
			}
		} else {
			m_ioio_thread.set_speed(1500);
			m_ioio_thread.set_steering(1500);
			panVal = 1500;
			tiltVal = 1500;
			m_ioio_thread.set_pan(panVal);
			m_ioio_thread.set_tilt(tiltVal);
			
		}

		return mRgba;
	}
	
	//send occupancy grid information using output stream from socket
	public void sendGrid(){
		if(dataOutputStream != null)
			try {
				dataOutputStream.writeInt(numSweeps);
				for(int i = 0; i < gridSize; i++){
					for(int j = 0; j < gridSize; j++){
						dataOutputStream.writeInt(occupancyGrid[i][j]);
					}
				}
				Log.i("rescue robotics", "grid sent");
			} catch (IOException e) {
				Log.e("rescue robotics", e.getMessage());
			}
	}

	//receive and update occupancy grid data using input stream from socket
	public void getGrid(){
		try {
			if(dataInputStream != null && dataInputStream.available() >= 4*17) {
				int receivedNumSweeps = dataInputStream.readInt();
				for(int i = 0; i < gridSize; i++){
					for(int j = 0; j < gridSize; j++){
						int receivedVal = dataInputStream.readInt();
						if((receivedNumSweeps > numSweeps) || receivedVal > occupancyGrid[i][j])
							occupancyGrid[i][j] = receivedVal;
						Log.e("rescue robotics","i:"+i+",j:"+j+",val:"+receivedVal);
					}
				}
				if(receivedNumSweeps > numSweeps)
					numSweeps = receivedNumSweeps;
			}
		} catch (IOException e) {
			Log.e("rescue robotics", e.getMessage());
		}
	}
	
	//construct the 2D matrix of Locations corresponding to the centers of grid squares
	//works only in northern hemisphere and not if your field crosses the prime meridian
	public static Location[][] calculateGridLocations(Location topLeft, Location bottomRight, int gridSize){
		Location[][] result = new Location[gridSize][gridSize];
		for(int i = 0; i < gridSize; i++){
			for(int j = 0; j < gridSize; j++){
				//calculate dot around origin
				double c = Math.sqrt(Math.pow(topLeft.getLongitude()-bottomRight.getLongitude(),2) + Math.pow(topLeft.getLatitude()-bottomRight.getLatitude(),2));
				double x = c/Math.sqrt(2);
				double lon = -x/2 + x/gridSize/2 + i * x/gridSize;
				double lat = x/2 - x/gridSize/2 - j * x/gridSize;
				//calculate theta
				double midpt_lon = (topLeft.getLongitude() + bottomRight.getLongitude()) / 2;
				double midpt_lat = (topLeft.getLatitude() + bottomRight.getLatitude()) / 2;
				double topLeftOrig_lon = topLeft.getLongitude() - midpt_lon;
				double topLeftOrig_lat = topLeft.getLatitude() - midpt_lat;
				double topLeftOrig_theta = Math.atan2(topLeftOrig_lat,topLeftOrig_lon);
				double theta = topLeftOrig_theta - 3 * Math.PI / 4;
				//rotate dot around origin
				lon = lon * Math.cos(theta) - lat * Math.sin(theta);
				lat = lon * Math.sin(theta) + lat * Math.cos(theta);
				//translate dot to original offset location
				lon = lon + midpt_lon;
				lat = lat + midpt_lat;
				Location resultLoc = new Location("");
				resultLoc.setLongitude(lon);
				resultLoc.setLatitude(lat);
				result[j][i] = resultLoc;
				Log.i("rescue robotics","lon:"+lon+",lat:"+lat);
			}
		}
		return result;
	}

	//revert any degree measurement back to the -179 to 180 degree scale
	public float fixWraparound(float deg){
		if(deg <= 180.0 && deg > -179.99)
			return deg;
		else if(deg > 180)
			return deg-360;
		else
			return deg+360;
		  
	}
	
	//determine whether 2 directions are roughly pointing in the same direction, correcting for angle wraparound
	public boolean sameDir(float dir1, float dir2){
		float dir = bearing%360;
		float headingMod = heading%360;
		//return (Math.abs((double) (headingMod - dir)) < 22.5 || Math.abs((double) (headingMod - dir)) > 337.5);
		return (Math.abs((double) (headingMod - dir)) < 2.5 || Math.abs((double) (headingMod - dir)) > 357.5);
	}
	
	//set the text of any text view in this application
	public void setText(final String str, final TextView tv) 
	{
		  runOnUiThread(new Runnable() {
			  @Override
			  public void run() {
				  tv.setText(str);
			  }
		  });
	}

	/****************************************************** functions from IOIOActivity *********************************************************************************/

	/**
	 * Create the {@link IOIO_thread}. Called by the
	 * {@link IOIOAndroidApplicationHelper}. <br>
	 * Function copied from original IOIOActivity.
	 * 
	 * @see {@link #get_ioio_data()} {@link #start_IOIO()}
	 * */
	@Override
	public IOIOLooper createIOIOLooper(String connectionType, Object extra) {
		if (m_ioio_thread == null
				&& connectionType
						.matches("ioio.lib.android.bluetooth.BluetoothIOIOConnection")) {
			m_ioio_thread = new IOIO_thread(this);
			return m_ioio_thread;
		} else
			return null;
	}

	@Override
	protected void onDestroy() {
		super.onDestroy();
		Log.i("activity cycle","main activity being destroyed");
		helper_.destroy();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	@Override
	protected void onStart() {
		super.onStart();
		Log.i("activity cycle","main activity starting");
		helper_.start();
		mGoogleApiClient.connect();
	}

	@Override
	protected void onStop() {
		Log.i("activity cycle","main activity stopping");
		super.onStop();
		helper_.stop();
		mGoogleApiClient.disconnect();
		try {
			if(socket != null)
				socket.close();
			if(serverSocket != null)
				serverSocket.close();
			if(clientSocket != null)
				clientSocket.close();
		} catch (IOException e) {
			Log.e("rescue robotics", e.getMessage());
		}
		
	}

	@Override
	protected void onNewIntent(Intent intent) {
		super.onNewIntent(intent);
			if ((intent.getFlags() & Intent.FLAG_ACTIVITY_NEW_TASK) != 0) {
			helper_.restart();
		}
	}
	
}
