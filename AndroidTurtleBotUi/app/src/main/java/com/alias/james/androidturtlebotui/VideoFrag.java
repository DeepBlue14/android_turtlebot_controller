/*
 * File:   VideoFrag.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class displays the data stream from a camera, and acts as the base UI
 *                   during most of the execution.
 *
 * See: http://stackoverflow.com/questions/5125851/enable-disable-zoom-in-android-webview
 * Reference:
 *
 * Last Modified 11/03/2015
 */


package com.alias.james.androidturtlebotui;

import android.annotation.TargetApi;
import android.graphics.Bitmap;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.speech.tts.TextToSpeech;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.webkit.WebView;
import android.widget.ImageView;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.io.IOException;
import java.io.InputStream;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;


public class VideoFrag extends Fragment implements WebView.OnTouchListener, TextToSpeech.OnInitListener
{
    private TextToSpeech tts; /** Is used for speech. */
    private ImageView cameraView; /** Contains the video feed from the robot. */
    private float fingerPressX = 0; /** Is the x coordinate of the finger press action. */
    private float fingerPressY = 0; /** Is the y coordinate of the finger press action. */
    private float fingerReleaseX = 0; /** Is the x coordinate of the finger release action. */
    private float fingerReleaseY = 0; /** Is the y coordinate of the finger release action. */
    private Socket socket; /** Socket object to handle traffic between robot and Android device. */
    private InetAddress serverAddress = null; /** The address of the robot. */
    private final int MATRIX_SIZE = 921600; /** OpenCV matrixes  retrieved from the PrimeSense via ROS are always this size */
    private Bitmap camBitmap; /** Is the bitmap with bounding box drawn around the object the robot thinks the user selected. */

    LayoutInflater inflater; /** Is an inflater object to inflate the VerifyTouch dialog UI. */


    /**
     *
     *
     * @param savedInstanceState
     */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
    }


    /**
     * Initializes the UI components related to this fragment, especially setting up the WebView
     * settings.
     *
     * @param inflater
     * @param container
     * @param savedInstanceState
     *
     * @return
     */
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState)
    {
        View view = inflater.inflate(R.layout.camera_layout, container, false);
        this.inflater = inflater;
        //dataCom.setUiStuff(getActivity(), inflater);
        tts = new TextToSpeech(getActivity(), this);
        cameraView = (ImageView) view.findViewById(R.id.web_view);
        cameraView.setOnTouchListener(this);

        startWebCamStream();

        return view;
    }


    /**
     * Is called when the user touches the video.  If the action is detected as a touch instead of
     * a slide
     * @see #isWithinMargin(float, float, float)
     * then a dialog appears asking the user if he wishes the location evaluated by the robot.
     *
     * @param v
     * @param event
     *
     * @return
     */
    @Override
    public boolean onTouch(View v, MotionEvent event)
    {
        int action = event.getAction();

        switch(action)
        {
            case MotionEvent.ACTION_DOWN:
                fingerPressX = event.getX();
                fingerPressY = event.getY();
                //System.out.println("ACTION_DOWN at: (" + event.getX() + ", " + event.getY() + ")");
                break;
            case MotionEvent.ACTION_MOVE:
                //System.out.println("ACTION_MOVE");
                break;
            case MotionEvent.ACTION_UP:
                fingerReleaseX = event.getX();
                fingerReleaseY = event.getY();
                //System.out.println("ACTION_UP at: (" + event.getX() + ", " + event.getY() + ")");
                if(isWithinMargin(fingerPressX, fingerReleaseX, 25.0f) && isWithinMargin(fingerPressY, fingerReleaseY, 25.0f) )
                {
                    //verifyTouch.initVerifyDialog(getActivity(), inflater, "|" + (int) fingerPressX + "|" + (int) fingerPressY + "|", dataCom);
                    //verifyTouch.show();
                }
                break;
            case MotionEvent.ACTION_CANCEL:
                //System.out.println("ACTION_CANCEL");
                break;
            default:
                System.out.println("default");
        }

        return false;
    }


    /**
     * Checks to see if the user event was a press or a slide.
     *
     * @param pressInt
     * @param releaseInt
     * @param margin
     *
     * @return
     */
    public boolean isWithinMargin(float pressInt, float releaseInt, float margin)
    {
        if(Math.abs(pressInt - releaseInt) - margin < 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }


    /**
     * Sets up the speech module.
     *
     * @param status
     */
    @Override
    public void onInit(int status)
    {
        if(tts == null)
        {
            tts = new TextToSpeech(getActivity(), this);
        }

        if(status == TextToSpeech.SUCCESS)
        {
            speakOut();

        } else
        {
            Log.e("TTS", "Initilization Failed!");
        }
    }


    /**
     * When called, this method invokes speak(...).
     */
    @TargetApi(Build.VERSION_CODES.LOLLIPOP)
    public void speakOut()
    {
        /*
        String text = "Testing";
        tts.speak(text, TextToSpeech.QUEUE_FLUSH, null, null);
        */
    }


    /**
     * Starts accepting the video stream.
     */
    public void startWebCamStream()
    {

        /*
        //http://stackoverflow.com/questions/12716850/android-update-textview-in-thread-and-runnable
        final Handler handler = new Handler();
        //Runnable runnable = new Runnable() {
        Thread runnable = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    serverAddress = InetAddress.getByName(UniversalDat.getIpAddressStr());
                    socket = new Socket(serverAddress, UniversalDat.getVideoPort() ); //!!!connect in a separate method--same goes for FetchLRFrames!!!
                    System.out.println("^^^Successfully connected to server^^^");
                } catch (UnknownHostException e) {
                    e.printStackTrace();
                } catch (IOException e) {
                    e.printStackTrace();
                }

                while (true)
                {
                    try
                    {
                        Thread.sleep(1000);
                    }
                    catch (InterruptedException e)
                    {
                        e.printStackTrace();
                    }
                    handler.post(new Runnable() {
                        public void run()
                        {
                            try
                            {
                                InputStream sub = socket.getInputStream(); // ???Do this once???
                                System.out.println("^^^Generating new InputStream obj");
                                // !!!???will this be true 32 & 64 bit processors???!!!
                                byte[] buffer = new byte[MATRIX_SIZE];
                                int currPos = 0;
                                int bytesRead = 0;

                                //bytesRead = sub.read(buffer, 0, buffer.length);
                                //currPos = bytesRead;
                                do
                                {
                                    bytesRead = sub.read(buffer, currPos, (buffer.length - currPos));

                                    if (bytesRead != -1 && bytesRead != 0)
                                    {
                                        currPos += bytesRead;
                                    }
                                    else
                                    {
                                        break;
                                    }

                                } while (bytesRead != -1 && bytesRead != 0);

                                System.out.println("^^^recieved image bytes @ DataCom::doInBackground(...):" + currPos);
                                Mat mat = new Mat(480, 640, CvType.CV_8UC3);
                                mat.put(0, 0, buffer);
                                camBitmap = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);
                                Utils.matToBitmap(mat, camBitmap);
                            } catch (UnknownHostException e) {
                                System.out.println("^^^Ousted");
                                e.printStackTrace();
                            } catch (IOException e) {
                                System.out.println("^^^Ousted");
                                e.printStackTrace();
                            }

                            //update ui
                            //cameraView.setImageBitmap(camBitmap);


                        }
                    });
                }
            }
        });
        runnable.start();
        */
    }


    /**
     * Stops accepting the video stream.
     */
    public void stopWebCamStream()
    {
        ;//webView.stopLoading();
    }


    /**
     * Invokes the onDestroy of the superclass, and shuts down the speech module.
     */
    @Override
    public void onDestroy()
    {
        super.onDestroy();
        tts.shutdown();
    }



    // Accessor and mutator methods


    /**
     * Mutator.
     * @see #tts
     *
     * @param tts
     */
    public void setTts(TextToSpeech tts)
    {
        this.tts = tts;
    }


    /**
     * Accessor.
     * @see #tts
     *
     * @return
     */
    public TextToSpeech getTts()
    {
        return tts;
    }


    /**
     * Mutator.
     * @see #cameraView
     *
     * @param cameraView
     */
    public void setWebView(ImageView cameraView)
    {
        this.cameraView = cameraView;
    }


    /**
     * Accessor.
     * @see #cameraView
     *
     * @return
     */
    public ImageView getWebView()
    {
        return cameraView;
    }


    /**
     * Mutator.
     * @see #fingerPressX
     *
     * @param fingerPressX
     */
    public void setFingerPressX(float fingerPressX)
    {
        this.fingerPressX = fingerPressX;
    }


    /**
     * Accessor.
     * @see #fingerPressX
     *
     * @return
     */
    public float getFingerPressX()
    {
        return fingerPressX;
    }


    /**
     * Mutator.
     * @see #fingerPressY
     *
     * @param fingerPressY
     */
    public void setFingerPressY(float fingerPressY)
    {
        this.fingerPressY = fingerPressY;
    }


    /**
     * Accessor.
     * @see #fingerPressY
     *
     * @return
     */
    public float getFingerPressY()
    {
        return fingerPressY;
    }


    /**
     * Mutator.
     * @see #fingerReleaseX
     *
     * @param fingerReleaseX
     */
    public void setFingerReleaseX(float fingerReleaseX)
    {
        this.fingerReleaseX = fingerReleaseX;
    }


    /**
     * Accessor.
     * @see #fingerReleaseX
     *
     * @return
     */
    public float getFingerReleaseX()
    {
        return fingerReleaseX;
    }


    /**
     * Mutator.
     * @see #fingerReleaseY
     *
     * @param fingerReleaseY
     */
    public void setFingerReleaseY(float fingerReleaseY)
    {
        this.fingerReleaseY = fingerReleaseY;
    }


    /**
     * Accessor.
     * @see #fingerReleaseY
     *
     * @return
     */
    public float getFingerReleaseY()
    {
        return fingerReleaseY;
    }


    /**
     * Mutator.
     * @see #inflater
     *
     * @param inflater
     */
    public void setInflater(LayoutInflater inflater)
    {
        this.inflater = inflater;
    }


    /**
     * Accessor.
     * @see #inflater
     *
     * @return
     */
    public LayoutInflater getInflater()
    {
        return inflater;
    }


    /**
     * TODO: impelement this.
     *
     * @return
     */
    public String toString()
    {
        return "^^^*** METHOD STUB ***^^^";
    }




} // End of class VideoFrag