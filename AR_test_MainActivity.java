package com.example.ar_test;

import androidx.appcompat.app.AppCompatActivity;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.net.Uri;
import android.os.Bundle;
import android.os.ParcelFileDescriptor;
import android.util.Log;
import android.view.View;
import android.widget.ImageView;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;
import org.opencv.core.Size;
import static org.opencv.android.Utils.bitmapToMat;
import static org.opencv.android.Utils.matToBitmap;
import static org.opencv.core.CvType.CV_32F;
import static org.opencv.core.CvType.CV_32FC1;
import static org.opencv.core.CvType.CV_64F;
import static org.opencv.core.CvType.CV_64FC1;
import static org.opencv.core.CvType.CV_64FC2;

import java.io.FileDescriptor;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;

import static android.graphics.Bitmap.createBitmap;

public class MainActivity extends AppCompatActivity {
    
    
    private static final int RESULT_PICK_IMAGE = 1000;
    private ImageView imageView;
    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        //読み込み時のエラー対策？
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i("OpenCV", "OpenCV loaded successfully");
                    Mat imageMat = new Mat();
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    }
    
    
    public void onResume() {
        //エラーハンドリング
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d("OpenCV", "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.d("OpenCV", "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }
    
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        //メイン関数
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        imageView = (ImageView)findViewById(R.id.imageView);
        findViewById(R.id.button).setOnClickListener(new View.OnClickListener(){
            @Override
            public void onClick(View v){
                //メイン画面でボタンが押されたときに、画像を読み込む
                Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT);
                intent.addCategory(Intent.CATEGORY_OPENABLE);
                intent.setType("image/*");
                startActivityForResult(intent,RESULT_PICK_IMAGE);
                //onActivityResult(RESULT_PICK_IMAGE,RESULT_OK,intent);org.opencv.core.Size;
            }

        });
    }
    
    
    @Override
    public void onActivityResult(int requestCode, int resultCode, Intent resultData) {
        //画像を読み込んだあとの処理
        super.onActivityResult(requestCode, resultCode, resultData);
        if (requestCode == RESULT_PICK_IMAGE && resultCode == RESULT_OK) {
            Uri uri = null;
            if (resultData != null) {
                uri = resultData.getData();
                Log.d("MY STRING","corners 0");

                try {
                    Bitmap bmp = getBitmapFromUri(uri);
                    //color_detect2(bmp);
                    Mat print_mat = new Mat();
                    Mat print_mat_undistort = new Mat();

                    bitmapToMat(bmp,print_mat);
                    Log.d("MY_STRING","QR = "+readQRcode_mat(print_mat));

                    print_mat_undistort = undistortImage(print_mat);

                    Bitmap print_bmp = Bitmap.createBitmap(print_mat.cols(),print_mat.rows(),Bitmap.Config.ARGB_8888);
                    matToBitmap(print_mat_undistort,print_bmp);

                    readARmarker(print_bmp);

                    imageView.setImageBitmap(print_bmp);

                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }
    
    
    private String readQRcode_mat(Mat QR_mat){
        //QRコードを読み取ってみる
        String result = "error";
        QR_mat = trimMat(QR_mat,0.4);
        QR_mat = resizeMat(QR_mat,0.6);
        //QR_mat = undistortImage(QR_mat);

        int width = QR_mat.width();
        int height = QR_mat.height();
        int[] pixels = new int[width * height];

        Bitmap QR_bitmap = createBitmap(width,height,Bitmap.Config.ARGB_8888);
        matToBitmap(QR_mat,QR_bitmap);

        QR_bitmap.getPixels(pixels, 0, width, 0, 0, width, height);

        try {
            LuminanceSource source = new RGBLuminanceSource(width,height,pixels);
            BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));
            Reader reader = new QRCodeReader();
            Map<DecodeHintType, Object> tmpHintsMap;
            tmpHintsMap = new EnumMap<DecodeHintType, Object>(DecodeHintType.class);
            tmpHintsMap.put(DecodeHintType.TRY_HARDER, Boolean.TRUE);
            com.google.zxing.Result decodeResult = reader.decode(binaryBitmap, tmpHintsMap);
            result = decodeResult.getText();
        } catch (Exception e) {
        }
        return result;
    }

    
    private Mat resizeMat(Mat input_mat,double rate){
        //Mat型の画像のリサイズを行う
        int width = input_mat.width();
        int height = input_mat.height();
        Size new_size = new Size(width * rate,height * rate);
        Mat output_mat = new Mat();
        Imgproc.resize(input_mat,output_mat,new_size);
        return output_mat;
    }
    
    
    private Mat trimMat(Mat input_mat,double rate){
        //画像の切り取りを行う
        int width = input_mat.width();
        int height = input_mat.height();
        int new_width = (int) (width * rate);
        int new_height = (int) (height * rate);
        int rect_x = (width - new_width)/2;
        int rect_y = (height - new_height)/2;
        Rect roi = new Rect(rect_x,rect_y,new_width,new_height);
        return new Mat(input_mat,roi);
    }
    
    
    private double[] undistortPoint(double x,double y){
        //UndistortPointを使って歪み補正を試みる
        double camera_matrix[] = {344.173397, 0.000000, 630.793795,
                0.000000, 344.277922, 487.033834,
                0.000000, 0.000000, 1.000000};
        double distortion_coefficients[] = {-0.152963, 0.017530, -0.001107, -0.000210,0.000000};

        /*

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
        */

        Mat camera_matrix_mat = new Mat(1,1,CV_64F);
        Mat distortion_coefficients_mat = new Mat(1,1,CV_64F);

        camera_matrix_mat.put(0,0,camera_matrix);
        distortion_coefficients_mat.put(0,0,distortion_coefficients);

        Mat input_xy_mat = new Mat(1,1,CV_64FC2);
        Mat output_xy_mat = new Mat(1,1,CV_64FC2);

        double[] input_xy = {x,y};
        input_xy_mat.put(0,0,input_xy);

        Log.d("MY_STRING","OK");

        Imgproc.undistortPoints(input_xy_mat,output_xy_mat,camera_matrix_mat,distortion_coefficients_mat);

        double[] output_array = {output_xy_mat.get(0,0)[0],output_xy_mat.get(0,0)[1]};

        Log.d("MY_STRING",output_array[0] + "," + output_array[1]);
        return  output_array;
    }
    
    
    private Mat undistortImage(Mat input_mat){
        //画像の歪みを補正する（パラメーターはシミュレーションのNavCam）
        double camera_matrix[] = {344.173397, 0.000000, 630.793795,
                0.000000, 344.277922, 487.033834,
                0.000000, 0.000000, 1.000000};
        double distortion_coefficients[] = {-0.152963, 0.017530, -0.001107, -0.000210,0.000000};

        Mat camera_matrix_mat = new Mat(3,3,CV_64FC1);
        Mat distortion_coefficients_mat = new Mat(1,5,CV_64FC1);
        camera_matrix_mat.put(0,0,camera_matrix);
        distortion_coefficients_mat.put(0,0,distortion_coefficients);

        /*
        camera_matrix_mat.put(0,0,camera_matrix[0],camera_matrix[1],camera_matrix[2],
                                            camera_matrix[3],camera_matrix[4],camera_matrix[5],
                                            camera_matrix[6],camera_matrix[7],camera_matrix[8]);
        distortion_coefficients_mat.put(0,0,distortion_coefficients[0],distortion_coefficients[1],
                                                      distortion_coefficients[2],distortion_coefficients[3]);
        */
        /*
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
        //distortion_coefficients_mat.put(0,4,distortion_coefficients[4]);
        */

        Size new_size = new Size(1280,960);
        Imgproc.resize(input_mat,input_mat,new_size);
        Mat undistorted_mat = new Mat();
        //Imgproc.undistort(input_mat,undistorted_mat,camera_matrix_mat,distortion_coefficients_mat);
        Log.d("MY_STRING",""+input_mat.width());
        Log.d("MY_STRING",""+input_mat.height());

        Size original_size = new Size(input_mat.width(),input_mat.height());
        Mat new_cammat = Calib3d.getOptimalNewCameraMatrix(camera_matrix_mat,distortion_coefficients_mat,original_size,1);
        //Mat eye = Mat.eye(3,3,CV_32FC1);
        Mat eye = new Mat();
        Mat output_map1 = new Mat();
        Mat output_map2 = new Mat();


        Imgproc.initUndistortRectifyMap(camera_matrix_mat,distortion_coefficients_mat,eye,new_cammat,original_size,CV_32FC1,output_map1,output_map2);

        Imgproc.remap(input_mat,undistorted_mat,output_map1,output_map2,Imgproc.INTER_AREA);

        return undistorted_mat;
    }
    
    
    private org.opencv.core.Point color_detect2(Bitmap bmp){
        //緑色を検出するプログラムの実験 改良版
        Mat mat1 = new Mat();
        Mat mat2 = new Mat();
        Mat mat_gray = new Mat();
        bitmapToMat(bmp,mat1);
        Imgproc.cvtColor(mat1,mat1, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(mat1,mat2,Imgproc.COLOR_BGR2HSV);
        Core.inRange(mat2, new Scalar(75, 50, 50), new Scalar(110, 255, 255), mat_gray);
        Mat kernel = new Mat();
        org.opencv.core.Point point = new org.opencv.core.Point(-1,-1);
        Imgproc.erode(mat_gray, mat_gray, kernel,point,2);
        Imgproc.dilate(mat_gray, mat_gray, kernel,point,3);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mat_gray,contours,hierarchy,Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_NONE);
        Mat contours_point = new Mat();
        int max_area_num = 0;
        double max_area = 0;
        if(contours.size() == 0){
            Log.d("my","no blue");
            org.opencv.core.Point position = new org.opencv.core.Point(-100, -100);
            return position;
        }else {
            for (int i = 0; i < contours.size(); i++) {
                double area = Imgproc.contourArea(contours.get(i));
                max_area_num = max_area > area ? max_area_num : i;
                max_area = max_area > area ? max_area : area;
            }
            contours_point = contours.get(max_area_num);
            Rect rect = Imgproc.boundingRect(contours_point);
            org.opencv.core.Point position = new org.opencv.core.Point(rect.x + (rect.width / 2.0), rect.y + (rect.height / 2.0));
            Log.d("my","x="+position.x+"y="+position.y);
            return position;
        }
    }
    
    
    private Bitmap color_detect(Bitmap bmp){
        //緑色を検出するプログラムを書いてみる
        Mat mat1 = new Mat();
        Mat mat2 = new Mat();
        Mat mat_gray = new Mat();
        bitmapToMat(bmp,mat1);
        Imgproc.cvtColor(mat1,mat1, Imgproc.COLOR_RGBA2BGR);
        Imgproc.cvtColor(mat1,mat2,Imgproc.COLOR_BGR2HSV);
        Core.inRange(mat2, new Scalar(75, 50, 50), new Scalar(110, 255, 255), mat_gray);
        Mat kernel = new Mat();
        Point point = new Point(-1,-1);
        Imgproc.erode(mat_gray, mat_gray, kernel,point,2);
        Imgproc.dilate(mat_gray, mat_gray, kernel,point,3);
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mat_gray,contours,hierarchy,Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_NONE);
        Mat contours_point = new Mat();
        int max_area_num = 0;
        double max_area = 0;
        for(int i = 0;i < contours.size();i++){
            double area = Imgproc.contourArea(contours.get(i));
            max_area_num = max_area > area ? max_area_num : i;
            max_area = max_area > area ? max_area : area;
        }
        if(max_area_num != 0) {
            contours_point = contours.get(max_area_num);
        }
        Mat label_mat = new Mat();
        Mat stats = new Mat();
        Mat centroids  = new Mat();
        Imgproc.connectedComponentsWithStats(mat_gray,label_mat,stats,centroids);
        Scalar scl = new Scalar(255,0,0);
        Rect rect = Imgproc.boundingRect(contours_point);
        Point position = new Point(rect.x + (rect.width / 2.0),rect.y + (rect.height / 2.0));
        Imgproc.drawMarker(mat_gray,position,scl,0,50,20);
        matToBitmap(mat_gray,bmp);
        return bmp;
    }
    
    
    private Bitmap fishEye(Bitmap bmp){
        //魚眼レンズの補正に当てはめられるかを確認。上手く行かなかった。
        Mat camera_param = new Mat(3,3,CV_64F);
        Mat distortion_param = new Mat(5,1,CV_64F);
        Mat mat = new Mat();
        camera_param.put(0,0,344.173397);
        camera_param.put(1,0,0.000000);
        camera_param.put(2,0,630.793795);
        camera_param.put(0,1,0.000000);
        camera_param.put(1,1,344.277922);
        camera_param.put(2,1,487.033834);
        camera_param.put(0,2,0.000000);
        camera_param.put(1,2,0.000000);
        camera_param.put(2,2,1.000000);
        distortion_param.put(0,0,-0.152963);
        distortion_param.put(1,0,0.017530);
        distortion_param.put(2,0,-0.001107);
        distortion_param.put(3,0,-0.000210);
        distortion_param.put(4,0,0.000000);;
        Log.d("s","make param");
        bitmapToMat(bmp,mat);
        Log.d("s","bmp to mat");
        bitmapToMat(bmp,mat);
        Log.d("s","undistort");
        Mat dist = new Mat();
        Imgproc.undistort(mat,dist,camera_param,distortion_param,camera_param);
        Log.d("s","mat to bitmap");
        matToBitmap(dist,bmp);
        return bmp;
    }
    
    
    private Bitmap getBitmapFromUri(Uri uri) throws IOException {
        //Bitmap型の画像を取得する
        ParcelFileDescriptor parcelFileDescriptor =
                getContentResolver().openFileDescriptor(uri, "r");
        FileDescriptor fileDescriptor = parcelFileDescriptor.getFileDescriptor();
        Bitmap image = BitmapFactory.decodeFileDescriptor(fileDescriptor);
        parcelFileDescriptor.close();
        return image;
    }
    
    
    private void readARmarker(Bitmap bitmap){
        //ARマーカーの読み取りを確認
        Mat mat = new Mat();
        bitmapToMat(bitmap,mat,false);
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        double camera_param[] = {-0.152963, 0.017530, -0.001107, -0.000210, 0.000000};
        //Mat camera_param_mat = new Mat(0,5,CvType.CV_64F,camera_param);
        Mat inputImage = mat;
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        DetectorParameters parameters = DetectorParameters.create();
        Imgproc.cvtColor(inputImage,inputImage, Imgproc.COLOR_BGR2GRAY);
        Aruco.detectMarkers(inputImage,dictionary,corners,markerIds,parameters);
        String markerIDfromArr = Arrays.toString(markerIds.get(0,0));
        if(markerIDfromArr == "null"){
            Log.d("MY STRING","NO MARKER");
        }else{
            String markerID = String.valueOf((int)(markerIds.get(0,0)[0]));
            Log.d("MY STRING","ID:"+markerID);
            double x_dif = corners.get(0).get(0,2)[0] - corners.get(0).get(0,0)[0];
            double z_dif = corners.get(0).get(0,2)[1] - corners.get(0).get(0,0)[1];
            double next_pos_x = corners.get(0).get(0,0)[0] + (x_dif / 2);
            double next_pos_y = corners.get(0).get(0,0)[1] + (z_dif / 2);

            Log.d("MY STRING","corners 0"+Arrays.toString(corners.get(0).get(0,0)));
            Log.d("MY STRING","corners 1"+Arrays.toString(corners.get(0).get(0,1)));
            Log.d("MY STRING","corners 2"+Arrays.toString(corners.get(0).get(0,2)));
            Log.d("MY STRING","corners 3"+Arrays.toString(corners.get(0).get(0,3)));
            Log.d("MY STRING","next_pos_x :" + next_pos_x + "next_pos_y :" + next_pos_y);
            //undistortPoint(next_pos_x,next_pos_y);
        }
    }
}
