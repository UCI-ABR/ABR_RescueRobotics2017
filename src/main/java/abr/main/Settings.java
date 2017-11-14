package abr.main;

import android.app.Activity;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.ActivityInfo;
import android.os.Bundle;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.Spinner;

public class Settings extends Activity {
    Button buttonConnect;
    Spinner currentDisplaySpinner;

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        setContentView(R.layout.settings);
        SharedPreferences settings = getSharedPreferences("Settings", 0);

        currentDisplaySpinner = (Spinner)findViewById(R.id.currentDisplaySpinner);

        buttonConnect = (Button)findViewById(R.id.buttonConnect);
        buttonConnect.setEnabled(false);
        buttonConnect.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                Intent intent = new Intent(Settings.this, Main_activity.class);
                intent.putExtra("selectedCurrentDisplay", currentDisplaySpinner.getSelectedItem().toString());
                startActivity(intent);
            }
        });
    }

    public void onResume() {
        super.onResume();
        buttonConnect.setEnabled(false);

        new Thread(new Runnable() {
            public void run() {
                try {
                    Thread.sleep(1000);
                    runOnUiThread(new Runnable() {
                        public void run() {
                            buttonConnect.setEnabled(true);
                        }
                    });
                } catch (InterruptedException e) { }
            }
        }).start();
    }
}


