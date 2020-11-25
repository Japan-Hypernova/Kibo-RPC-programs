package jp.jaxa.iss.kibo.rpc.japan;

import org.opencv.calib3d.Calib3d;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import static org.opencv.android.Utils.bitmapToMat;
import static org.opencv.android.Utils.matToBitmap;
import static org.opencv.core.CvType.CV_32FC1;
import static org.opencv.core.CvType.CV_64F;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import static android.graphics.Bitmap.createBitmap;
import android.graphics.Bitmap;

public class YourService extends KiboRpcService {
    
    
    @Override
    protected void runPlan1(){
        //U1 P2付近から移動開始するプログラム
        api.judgeSendStart();
        moveToWrapper(10.83, -8.21, 5.27,0, 0, 0.707, -0.707);
        double airlock_center_x = 10.9924;
        double airlock_center_y = -10.1;
        double airlock_center_z = 5.3486585;
        moveToWrapper(airlock_center_x, airlock_center_y, airlock_center_z, 0, 0, 0.707, -0.707);
        double[] current_pos = {airlock_center_x,airlock_center_y,airlock_center_z};
        getARmarker(current_pos);
        api.judgeSendFinishISS();
    }
    
    
    @Override
    protected void runPlan2(){
        //U2 P3手前から移動開始するプログラム
        api.judgeSendStart();
        double airlock_center_x = 10.9924;
        double airlock_center_y = -10.1;
        double airlock_center_z = 5.3486585;
        moveToWrapper(airlock_center_x, airlock_center_y, airlock_center_z, 0, 0, 0.707, -0.707);
        double[] current_pos = {airlock_center_x,airlock_center_y,airlock_center_z};
        getARmarker(current_pos);
        api.judgeSendFinishISS();
    }
    
    
    private void getARmarker(double[] current_pos){
        //ARマーカーの取得からレーザーの照射まで
        int loop_count = 0;
        boolean isSucceeded;
        do{
            Point position = getPositionWrapper(10.9924, -10.1, 5.3486585);
            double m_pos_x = position.getX();
            double m_pos_y = position.getY();
            double m_pos_z = position.getZ();
            if (!(m_pos_x >= 10.9824 && m_pos_x <= 11.0024 && m_pos_y >= -10.11 && m_pos_y <= -10.09 && m_pos_z >= 5.338 && m_pos_z <= 5.358)) {
                moveToWrapper(10.9924, -10.1, 5.3486585, 0, 0, 0.707, -0.707);
            }
            moveToWrapper(10.9924, -10.1, 5.3486585, 0, 0, 0.707, -0.707);
            relativeMoveToWrapper(0,0, 0, 0, 0, 0.707, -0.707);
            api.flashlightControlFront(0.025f);
            sleep(2000);
            Bitmap bitmap = api.getBitmapNavCam();
            api.flashlightControlFront(0);
            isSucceeded = readARmarker(bitmap,current_pos,true);
            loop_count ++;
        } while(!isSucceeded && loop_count < 50);
    }
    
    
    private Bitmap undistortImage(Bitmap input_bitmap){
        //撮影した画像の歪み補正
        Mat input_mat = new Mat();
        bitmapToMat(input_bitmap,input_mat);
        double[] camera_matrix = {692.827528, 0.000000, 571.399891, 0.000000, 691.919547, 504.956891, 0.000000, 0.000000, 1.000000};
        double[] distortion_coefficients = {-0.312191, 0.073843, -0.000918, 0.001890, 0.000000};
        Mat camera_matrix_mat = new Mat(3,3,CV_64F);
        Mat distortion_coefficients_mat = new Mat(1,5,CV_64F);
        camera_matrix_mat.put(0,0,camera_matrix[0]);
        camera_matrix_mat.put(0,1,camera_matrix[1]);
        camera_matrix_mat.put(0,2,camera_matrix[2]);
        camera_matrix_mat.put(1,0,camera_matrix[3]);
        camera_matrix_mat.put(1,1,camera_matrix[4]);
        camera_matrix_mat.put(1,2,camera_matrix[5]);
        camera_matrix_mat.put(2,0,camera_matrix[6]);
        camera_matrix_mat.put(2,1,camera_matrix[7]);
        camera_matrix_mat.put(2,2,camera_matrix[8]);
        distortion_coefficients_mat.put(0,0,distortion_coefficients[0]);
        distortion_coefficients_mat.put(0,1,distortion_coefficients[1]);
        distortion_coefficients_mat.put(0,2,distortion_coefficients[2]);
        distortion_coefficients_mat.put(0,3,distortion_coefficients[3]);
        distortion_coefficients_mat.put(0,4,distortion_coefficients[4]);
        Mat undistorted_mat = new Mat();
        Size original_size = new Size(input_mat.width(),input_mat.height());
        Mat new_cammat = Calib3d.getOptimalNewCameraMatrix(camera_matrix_mat,distortion_coefficients_mat,original_size,1);
        Mat eye = Mat.eye(3,3,CV_32FC1);
        Mat output_map1 = new Mat();
        Mat output_map2 = new Mat();
        Imgproc.initUndistortRectifyMap(camera_matrix_mat,distortion_coefficients_mat,eye,new_cammat,original_size,CV_32FC1,output_map1,output_map2);
        Imgproc.remap(input_mat,undistorted_mat,output_map1,output_map2,Imgproc.INTER_AREA);
        Bitmap output_bitmap = createBitmap(undistorted_mat.width(),undistorted_mat.height(), Bitmap.Config.ARGB_8888);
        matToBitmap(undistorted_mat,output_bitmap);
        return output_bitmap;
    }
    
    
    private double[] calcTargetQuaternion(double[] current_pos,double ar_pixel_x,double ar_pixel_z){
        //ターゲットの姿勢を算出する
        double nav_center_x = 640;
        double nav_center_z = 480;
        double current_position_y = current_pos[1];
        double offset_x = 0.059;
        double offset_y = 0.152;
        double offset_z = 0.13;
        double pixel_transformation_constant = 0.001731;
        double pixel_transformation_constant_2 = 0.00172;
        double delta_x =  -(ar_pixel_x - nav_center_x)*pixel_transformation_constant;
        double delta_z = -(ar_pixel_z - nav_center_z)*pixel_transformation_constant_2;
        double displacement_x = delta_x -offset_x;
        double displacement_y = 10.56875 + current_position_y + offset_y;
        double displacement_z = delta_z - offset_z;
        double Z_over_y_slope = displacement_z/displacement_y;
        double zy_quaternion_x_and_y = -Z_over_y_slope/(1+Z_over_y_slope)/2;
        double unit_adjustment = 1-(zy_quaternion_x_and_y*zy_quaternion_x_and_y*2);
        double significance_x = displacement_y;
        double significance_y = displacement_y + displacement_x;
        if (displacement_x < 0) {
            significance_x = displacement_y - displacement_x;
            significance_y = displacement_y;
        }
        double y_over_x_atan = Math.atan(significance_y/significance_x);
        double y_over_x_0_to_1 = (y_over_x_atan/3.1415926*2);
        double yx_quaternion_w = -(unit_adjustment - y_over_x_0_to_1)/Math.sqrt(unit_adjustment-(2*y_over_x_0_to_1)+(2*(y_over_x_0_to_1*y_over_x_0_to_1)));
        double yx_quaternion_z = y_over_x_0_to_1*unit_adjustment/(Math.sqrt(unit_adjustment-(2*y_over_x_0_to_1)+(2*y_over_x_0_to_1*y_over_x_0_to_1)));
        double[] ans = {zy_quaternion_x_and_y,zy_quaternion_x_and_y,yx_quaternion_z,yx_quaternion_w};
        return ans;
    }
    
    
    private boolean readARmarker(Bitmap bitmap,double[] current_pos, boolean needSend) {
        //ARマーカーの読み取り
        bitmap = undistortImage(bitmap);
        int width = bitmap.getWidth();
        int height = bitmap.getHeight();
        bitmap = Bitmap.createScaledBitmap(bitmap,(int)(width),(int)(height),true);
        Mat inputImage = new Mat();
        bitmapToMat(bitmap,inputImage,false);
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        DetectorParameters parameters = DetectorParameters.create();
        Imgproc.cvtColor(inputImage, inputImage, Imgproc.COLOR_BGR2GRAY);
        Aruco.detectMarkers(inputImage, dictionary, corners, markerIds, parameters);
        String markerIDfromArr = Arrays.toString(markerIds.get(0, 0));
        if (markerIDfromArr.equals("null")) {
            return false;
        } else {
            String markerID = String.valueOf((int)(markerIds.get(0, 0)[0]));
            if(needSend){
                api.judgeSendDiscoveredAR(markerID);
                double ar_width = corners.get(0).get(0,2)[0] - corners.get(0).get(0,0)[0];
                double ar_height = corners.get(0).get(0,2)[1] - corners.get(0).get(0,0)[1];
                double AR_center_x = corners.get(0).get(0,0)[0] + (ar_width / 2);
                double AR_center_z = corners.get(0).get(0,0)[1] + (ar_height / 2);
                double[] target_qua = calcTargetQuaternion(current_pos,AR_center_x,AR_center_z);
                moveToWrapper(10.9924, -10.1, 5.3486585, target_qua[0], target_qua[1], target_qua[2], target_qua[3]);
                relativeMoveToWrapper(0, 0, 0, target_qua[0], target_qua[1], target_qua[2], target_qua[3]);
                api.laserControl(true);
            }
            return true;
        }
    }
    
    
    private void moveToWrapper(double pos_x,double pos_y,double pos_z,
                               double qua_x,double qua_y,double qua_z,
                               double qua_w){
        //指定した座標に移動させる
        final int LOOP_MAX = 20;
        final Point point = new Point(pos_x,pos_y,pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x,(float)qua_y,
                (float)qua_z,(float)qua_w);
        Result result;
        int loop_count = 0;
        do{
            result = api.moveTo(point,quaternion,true);
            Point position = getPositionWrapper(pos_x,pos_y,pos_z);
            double m_pos_x = position.getX();
            double m_pos_y = position.getY();
            double m_pos_z = position.getZ();
            if ((pos_y <= -9.4) && m_pos_x >= (pos_x - 0.01) && m_pos_x <= (pos_x + 0.01) && m_pos_y >= (pos_y - 0.01) && m_pos_y <= (pos_y + 0.01) && m_pos_z >= (pos_z - 0.01) && m_pos_z <= (pos_z + 0.01)) {
                break;
            }  else if (m_pos_y < (pos_y + 0.05)) {
                break;
            }else{
                loop_count++;
            }
        }while(!result.hasSucceeded() && loop_count < LOOP_MAX);
    }
    
    
    private void relativeMoveToWrapper(double pos_x,double pos_y,double pos_z,
                                       double qua_x,double qua_y,double qua_z,
                                       double qua_w){
        //指定した相対的な座標に移動させる
        final int LOOP_MAX = 20;
        final Point point = new Point(pos_x,pos_y,pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x,(float)qua_y,
                (float)qua_z,(float)qua_w);
        Result result;
        int loop_count = 0;
        do{
            result = api.relativeMoveTo(point,quaternion,true);
            loop_count++;
        }while(!result.hasSucceeded() && loop_count < LOOP_MAX);
    }
    
    
    private void sleep(int millis){
        //Astrobeeを停止させる
        try {
            Thread.sleep(millis);
        }catch(InterruptedException e){
        }
    }
    
    
    private Point getPositionWrapper(double def_pos_x,double def_pos_y,double def_pos_z){
        //現在位置を取得する
        int timeout_sec = 10;
        Kinematics kinematics = api.getTrustedRobotKinematics(timeout_sec);
        Point ans_point = null;
        if(kinematics != null){
            ans_point = kinematics.getPosition();
        }
        if(ans_point == null){
            ans_point = new Point(def_pos_x,def_pos_y,def_pos_z);
        }
        return ans_point;
    }
}
