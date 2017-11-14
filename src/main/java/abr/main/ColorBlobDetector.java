/* Class modified from OpenCV Sample*/
package abr.main;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import android.util.Log;

public class ColorBlobDetector {
	static {
	    if (!OpenCVLoader.initDebug()) {
	        // Handle initialization error
	    }
	}
    // Lower and Upper bounds for range checking in HSV color space
    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);
    // Minimum contour area in percent for contours filtering
    private static double mMinContourArea = 0.1;
    // Color radius for range checking in HSV color space
    private Scalar mColorRadius = new Scalar(13,41,41,0); //25,50,50
    
    private Mat mSpectrum = new Mat();
    
    private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();
    private double maxArea;	//made this an accessible variable
    private double momentX;	//added
    private double momentY;	//added
    private double centerX;	//added
    private double centerY;	//added
    private int blobsDetected;

    // Cache
    Mat mPyrDownMat = new Mat();
    Mat mHsvMat = new Mat();
    Mat mMask = new Mat();
    Mat mDilatedMask = new Mat();
    Mat mHierarchy = new Mat();
    
    public void setColorRadius(Scalar radius) {
        mColorRadius = radius;
    }

    public void setHsvColor(Scalar hsvColor) {
        double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0]-mColorRadius.val[0] : 0;
        double maxH = (hsvColor.val[0]+mColorRadius.val[0] <= 255) ? hsvColor.val[0]+mColorRadius.val[0] : 255;
        
        //Log.i("opencv",""+hsvColor.val[0]+","+hsvColor.val[1]+","+hsvColor.val[2]);
        
        mLowerBound.val[0] = minH;
        mUpperBound.val[0] = maxH;

        mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
        mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];

        mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
        mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];

        mLowerBound.val[3] = 0;
        mUpperBound.val[3] = 255;
        
        Mat spectrumHsv = new Mat(1, (int)(maxH-minH), CvType.CV_8UC3);

        for (int j = 0; j < maxH-minH; j++) {
            byte[] tmp = {(byte)(minH+j), (byte)255, (byte)255};
            spectrumHsv.put(0, j, tmp);
        }

        Imgproc.cvtColor(spectrumHsv, mSpectrum, Imgproc.COLOR_HSV2RGB_FULL, 4);
        
    }

    public Mat getSpectrum() {
        return mSpectrum;
    }

    public void setMinContourArea(double area) {
        mMinContourArea = area;
    }

    public void process(Mat rgbaImage) {
        Imgproc.pyrDown(rgbaImage, mPyrDownMat);
        Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

        Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

        Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);
        Imgproc.dilate(mMask, mDilatedMask, new Mat());

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        
        centerX = (double)mDilatedMask.cols()/2;
        centerY = (double)mDilatedMask.width()/2;
        //Log.i("opencv", "Width="+mDilatedMask.cols()+",Length="+mDilatedMask.rows());
        Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find max contour area
        //double maxArea = 0;	//declared up top now
        maxArea = 0;	//added
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > maxArea){
                maxArea = area;
                calculateMoment(wrapper);	//calculate the moment of the biggest blob
            }
        }
        //reset moment back to zero
        if(contours.size()==0){
        	momentX = 0;
        	momentY = 0;
        }
        
        // Filter contours by area and resize to fit the original image size
        mContours.clear();
        each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint contour = each.next();
            if (Imgproc.contourArea(contour) > mMinContourArea*maxArea) {
                Core.multiply(contour, new Scalar(4,4), contour);
                mContours.add(contour);
            }
        }
        blobsDetected = mContours.size();
    }
    
    public int blobsDetected(){
    	return blobsDetected;
    }

    public List<MatOfPoint> getContours() {
        return mContours;
    }
    
    //Added
    public double getMaxArea(){
    	return maxArea;
    }
    
    //Added - calculates center of mass of blob with largest area
    public void calculateMoment(MatOfPoint contour){
    	List<Point> contourPoints = contour.toList();
    	momentX = 0;
    	momentY = 0;
    	for(int i = 0; i < contourPoints.size(); i++){
    		momentX += contourPoints.get(i).x;
    		momentY += contourPoints.get(i).y;
    	}
    	momentX = momentX/contourPoints.size();
    	momentY = momentY/contourPoints.size();
    	
    	momentX = momentX-centerX;
    	momentY = momentY-centerY;
    }
    
    //Added
    public double getMomentX(){
    	return momentX;
    }
    
    //Added
    public double getMomentY(){
    	return momentY;
    }
    
    //Added
    public double getCenterX(){
    	return centerX;
    }
    
    //Added
    public double getCenterY(){
    	return centerY;
    }
    
}
