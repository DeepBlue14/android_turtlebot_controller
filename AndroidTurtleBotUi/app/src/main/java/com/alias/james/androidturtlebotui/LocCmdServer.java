/*
 * File:   CamClient.java
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class contains a thread which will send location commands to the robot
 *                   to execute.
 *
 *
 * Last Modified 11/08/2015
 */


package com.alias.james.androidturtlebotui;

import android.graphics.Bitmap;
import android.os.Handler;
import android.widget.ImageView;



import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;

/**
 * Created by root on 11/12/15.
 */
public class LocCmdServer implements Runnable {


    //TODO: main() should notify this thread which fragment is loaded
    enum CurrentFrag {
        MAP,
        CAM,
        TELEOP
    }

    private static String locCmd = new String("000|000");
    private char DELIMITER = '|';
    private final int MESSAGE_SIZE = 8;
    private static CurrentFrag currentFrag;
    private int count=0;
    private ImageView imageView;
    private Bitmap bitmap;
    private InetAddress serverAddress = null;
    private Socket socket;
    private final int MATRIX_SIZE = 921600;
    private boolean isConnected = false;
    private boolean isIdling = false;
    Handler handler;


    public LocCmdServer() {
        handler = new Handler();
    }


    @Override
    public void run() {


        while (true) {
            if (!isIdling) {

                if (!isConnected) {
                    connect();
                }

                try {
                    if (socket != null && !locCmd.equals("666|666")) {
                        PrintWriter out = null;
                        out = new PrintWriter(new BufferedWriter(new OutputStreamWriter(socket.getOutputStream())), true);
                        out.println(locCmd);
                        System.out.println("^^^\tsent msg");
                        locCmd = "666|666";

                        /*
                        //update UI
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                imageView.setImageBitmap(bitmap);
                            }
                        });
                        */
                    } else {
                        //sub = null; //FIXME: hack
                    }

                } catch (IOException e) {
                    System.out.println("^^^kicked out :(");
                    e.printStackTrace();
                }
            } else {
                ;//spin (i.e. have the thread idle)
            }

        } // End of outer loop
    }


    public void connect() {
        System.out.println("^^^@ connect()");
        try {
            //serverAddress = InetAddress.getByName("10.0.4.6");
            serverAddress = InetAddress.getByName(UniversalDat.getIpAddressStr() );
            socket = new Socket(serverAddress, 50001);
            isConnected = true;
            System.out.println("^^^connected @ LocCmdServer");

            /*InputStream sub = socket.getInputStream();
            byte[] buffer = new byte[5];
            int currPos = 0;
            int bytesRead = 0;

            do {
                bytesRead = sub.read(buffer, currPos, (buffer.length - currPos));
                currPos += bytesRead;
            } while (bytesRead != -1 && bytesRead != 0);

            System.out.println("^^^connected successfully @ LocCmdServer and read: " + new String(buffer, "UTF-8") );
            */

        } catch (UnknownHostException e) {
            System.out.println("^^^Ousted 0");
            e.printStackTrace();
        } catch (IOException e) {
            System.out.println("^^^Ousted 1");
            e.printStackTrace();
        }catch (Exception e)
        {
            System.out.println("^^^Ousted 2");
            e.printStackTrace();
        }

    }


    private void runOnUiThread(Runnable runnable) {
        handler.post(runnable);
    }


    public void closeSocket() {
        try {
            socket.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    public void setCount(int count) {
        this.count = count;
    }


    public int getCount() {
        return count;
    }


    public static void setLocCmd(String locCmd) {
        LocCmdServer.locCmd = locCmd;
    }


    public static String getLocCmd() {
        return locCmd;
    }


    public void setImageView(ImageView imageView) {
        this.imageView = imageView;
    }


    public ImageView getImageView() {
        return imageView;
    }


    public void setBitmap(Bitmap bitmap) {
        this.bitmap = bitmap;
    }


    public Bitmap getBitmap() {
        return bitmap;
    }


    public void setServerAddress(InetAddress serverAddress) {
        this.serverAddress = serverAddress;
    }


    public InetAddress getServerAddress() {
        return serverAddress;
    }


    public void setSocket(Socket socket) {
        this.socket = socket;
    }


    public Socket getSocket() {
        return socket;
    }


    public int getMATRIX_SIZE() {
        return MATRIX_SIZE;
    }


    public void setIsConnected(boolean isConnected) {
        this.isConnected = isConnected;
    }


    public boolean getIsConnected() {
        return isConnected;
    }


    public void setIsIdling(boolean isIdling) {
        this.isIdling = isIdling;
    }


    public boolean getIsIdling() {
        return isIdling;
    }

    @Override
    protected void finalize() throws Throwable {
        if(socket != null) {
            closeSocket();
        }
        super.finalize();
    }

}