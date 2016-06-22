package abr.main;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;

import android.content.Context;
import android.os.AsyncTask;
import android.util.Log;
import android.view.View;
import android.widget.TextView;

public class FileClientAsyncTask extends AsyncTask<Void,Void,Object[]> {
	Object[] objects = new Object[3];
    @Override
    protected Object[] doInBackground(Void... params) {
    	int timeout = 10000;
	    int port = 8888;
	    InetSocketAddress socketAddress  = new InetSocketAddress("192.168.49.1", port);
	    try {
	      Socket socket = new Socket();
	      objects[0] = socket;
	      socket.bind(null);
	      socket.connect(socketAddress, timeout);
	      DataOutputStream outputStream = new DataOutputStream(socket.getOutputStream());
	      objects[1] = outputStream;
	      DataInputStream inputStream = new DataInputStream(socket.getInputStream());
	      objects[2] = inputStream;
	    } catch (IOException e) {
	      Log.e("hahaha", "IO Exception.", e);
	    }
	    return objects;
    }
}