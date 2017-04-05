package com.robotlifealert3.common.activities;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

import com.robotlifealert3.common.Constants;
import com.robotlifealert3.common.R;

public class Dashboard extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_dashboard);

        //find the buttons in the layout and setup click handlers
        Button startIoIoButton = (Button) findViewById(R.id.startIoIo);
        startIoIoButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(Constants.INTENT_IOIO_DEBUG_ACTIVITY);
                startActivityForIntent(intent);
            }
        });

        Button startTango = (Button) findViewById(R.id.startTango);
        startTango.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(Constants.INTENT_TANGO_AND_TENSORFLOW_ACTIVITY);
                startActivityForIntent(intent);
            }
        });
    }

    private void startActivityForIntent(Intent intent) {
        try {
            startActivity(intent);
        } catch (Exception ex){
            Toast.makeText(Dashboard.this, "Could not start Tango", Toast.LENGTH_SHORT).show();
        }
    }
}
