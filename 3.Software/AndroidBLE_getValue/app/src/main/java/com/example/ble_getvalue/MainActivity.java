/*
 * Copyright (C) 2013 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.example.ble_getvalue;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.ListActivity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.BaseAdapter;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.util.ArrayList;
import java.util.UUID;




/**
 * Activity for scanning and displaying available Bluetooth LE devices.
 */
public class MainActivity extends ListActivity {
    private LeDeviceListAdapter mLeDeviceListAdapter;
    private BluetoothAdapter mBluetoothAdapter;
    private boolean mScanning;
    private Handler mHandler;

    float temp;
    float hum;
    float tvoc;
    float eco2;

    private TextView tv_temp;
    private TextView tv_hum;
    private TextView tv_tvoc;
    private TextView tv_eco2;


    private static final int REQUEST_ENABLE_BT = 1;
    // Stops scanning after 10 seconds.
    private static final long SCAN_PERIOD = 10000;

    @SuppressLint("NewApi")
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        //getActionBar().setTitle(R.string.title);
        //setTitle(R.string.title);
        mHandler = new Handler();

        requestPermission();


        tv_temp = (TextView) findViewById(R.id.tv_temp);
        tv_hum = (TextView) findViewById(R.id.tv_hum);
        tv_tvoc = (TextView) findViewById(R.id.tv_tvoc);
        tv_eco2 = (TextView) findViewById(R.id.tv_eco2);

        // Use this check to determine whether BLE is supported on the device.  Then you can
        // selectively disable BLE-related features.
        if (!getPackageManager().hasSystemFeature(PackageManager.FEATURE_BLUETOOTH_LE)) {
            Toast.makeText(this, "Ble not supported", Toast.LENGTH_SHORT).show();
            finish();
        }

        // Initializes a Bluetooth adapter.  For API level 18 and above, get a reference to
        // BluetoothAdapter through BluetoothManager.
 final BluetoothManager bluetoothManager =
                (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        mBluetoothAdapter = bluetoothManager.getAdapter();

        // Checks if Bluetooth is supported on the device.
        if (mBluetoothAdapter == null) {
            Toast.makeText(this, "Bluetooth not supported", Toast.LENGTH_SHORT).show();
            finish();
            return;
        }
    }


    private void requestPermission() {
        //动态申请
        if (Build.VERSION.SDK_INT < 23) {
            return;
        }
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.ACCESS_COARSE_LOCATION}, 1);
        }
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, 1);
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.main, menu);
        if (!mScanning) {
            menu.findItem(R.id.menu_stop).setVisible(false);
            menu.findItem(R.id.menu_scan).setVisible(true);
            menu.findItem(R.id.menu_refresh).setActionView(null);
        } else {
            menu.findItem(R.id.menu_stop).setVisible(true);
            menu.findItem(R.id.menu_scan).setVisible(false);
            menu.findItem(R.id.menu_refresh).setActionView(
                    R.layout.actionbar_indeterminate_progress);
        }
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.menu_scan:
                mLeDeviceListAdapter.clear();
                scanLeDevice(true);
                break;
            case R.id.menu_stop:
                scanLeDevice(false);
                break;
        }
        return true;
    }

    @SuppressLint("MissingPermission")
    @Override
    protected void onResume() {
        super.onResume();

        //确保设备上已启用蓝牙功能。如果蓝牙当前未启用，
        //激发显示对话框的意图，要求用户授予启用该对话框的权限。
        if (!mBluetoothAdapter.isEnabled()) {
            if (!mBluetoothAdapter.isEnabled()) {
                Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
            }
        }

        //初始化列表视图适配器。
        mLeDeviceListAdapter = new LeDeviceListAdapter();
        setListAdapter(mLeDeviceListAdapter);
        scanLeDevice(true);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        //用户选择不启用蓝牙。
        if (requestCode == REQUEST_ENABLE_BT && resultCode == Activity.RESULT_CANCELED) {
            finish();
            return;
        }
        super.onActivityResult(requestCode, resultCode, data);
    }

    @Override
    protected void onPause() {
        super.onPause();
        scanLeDevice(false);
        mLeDeviceListAdapter.clear();
    }

    @SuppressLint({"MissingPermission", "NewApi"})
    private void scanLeDevice(final boolean enable) {
        if (enable) {
            //在预定义的扫描周期后停止扫描。
            mHandler.postDelayed(new Runnable() {

                @Override
                public void run() {
                    mScanning = false;
                    mBluetoothAdapter.stopLeScan(mLeScanCallback);
                    invalidateOptionsMenu();
                }
            }, SCAN_PERIOD);

            mScanning = true;

            //UUID AirTHD = UUID.fromString("FDA50693-A4E2-4FB1-AFCF-C6EB07647833");
            //UUID[] uuid = new UUID[1];
            //uuid[0] = AirTHD;

            mBluetoothAdapter.startLeScan(mLeScanCallback);
        } else {
            mScanning = false;
            mBluetoothAdapter.stopLeScan(mLeScanCallback);
        }
        invalidateOptionsMenu();
    }

    //用于保存通过扫描找到的设备的适配器。
    @SuppressLint("MissingPermission")
    private class LeDeviceListAdapter extends BaseAdapter {
        private ArrayList<BluetoothDevice> mLeDevices;
        private ArrayList<Integer> mRSSIs;//新加
        private ArrayList<byte[]> mRecords;//新加
        private LayoutInflater mInflator;

        public LeDeviceListAdapter() {
            super();
            mLeDevices = new ArrayList<BluetoothDevice>();
            mRSSIs = new ArrayList<Integer>();//新加
            mRecords = new ArrayList<byte[]>();//新加
            mInflator = MainActivity.this.getLayoutInflater();
        }

        public void addDevice(BluetoothDevice device,int rssi, byte[] scanRecord) {
            if(!mLeDevices.contains(device)) {
                mLeDevices.add(device);
                mRSSIs.add(rssi);//新加
                mRecords.add(scanRecord);//新加
            }
        }

        public BluetoothDevice getDevice(int position) {
            return mLeDevices.get(position);
        }

        public void clear() {
            mLeDevices.clear();
            mRecords.clear();
            mRSSIs.clear();
        }

        @Override
        public int getCount() {
            return mLeDevices.size();
        }

        @Override
        public Object getItem(int i) {
            return mLeDevices.get(i);
        }

        @Override
        public long getItemId(int i) {
            return i;
        }

        @Override
        public View getView(int i, View view, ViewGroup viewGroup) {
            ViewHolder viewHolder;
            // General ListView optimization code.
            if (view == null) {
                view = mInflator.inflate(R.layout.listitem_device, null);
                //view = mInflator.inflate(R.layout.activity_main, null);
                viewHolder = new ViewHolder();
                viewHolder.deviceAddress = (TextView) view.findViewById(R.id.device_address);
                viewHolder.deviceName = (TextView) view.findViewById(R.id.device_name);
                viewHolder.deviceBroadcastPack = (TextView) view.findViewById(R.id.device_broadcastPack);
                viewHolder.deviceRssi = (TextView) view.findViewById(R.id.device_rssi);
                viewHolder.placeholder = (TextView) view.findViewById(R.id.placeholder);


                view.setTag(viewHolder);
            } else {
                viewHolder = (ViewHolder) view.getTag();
            }

            BluetoothDevice device = mLeDevices.get(i);
            int rssi = mRSSIs.get(i);
            byte[] scanRecord = mRecords.get(i);

            //final String deviceName = device.getName();
            //提取信息
            final String deviceName = "设备名：" + device.getName();
            final String deviceAddr = "Mac地址：" + device.getAddress();
            final String broadcastPack ="广播包：" + bytesToHex(scanRecord);//此处调用了格式转换方法bytesToHex()将十六进制序列转String
            final String rssiString = "RSSI:" + String.valueOf(rssi) + "dB";//此处调用了String的格式转换方法valueOf()将数值类型转String

            String Record = bytesToHex(scanRecord);
            String uuid = "FDA50693A4E24FB1AFCFC6EB07647825";
            String uuid2 = "FDA50693A4E24FB1AFCFC6EB07647833";
            String target_uuid = Record.substring(18,18+uuid.length());
            String major = Record.substring(18+uuid.length(), 18+uuid.length()+4);
            String minor = Record.substring(18+uuid.length()+4, 18+uuid.length()+4+4);
            int Major = Integer.parseInt(major, 16);
            int Minor = Integer.parseInt(minor, 16);
            temp = (int)(Major/100)/10;
            hum = Major%100;
            tvoc = (Minor%100)*100;
            eco2 = (Minor/100)*10;

            if(uuid.equals(target_uuid) || uuid2.equals(target_uuid)){
                viewHolder.deviceName.setText("温度："+temp+"℃");
                viewHolder.deviceAddress.setText("湿度："+hum+"%");//显示设备地址
                viewHolder.deviceBroadcastPack.setText("TVOC："+tvoc+"ppm");//显示广播包
                viewHolder.deviceRssi.setText("eCO2："+eco2+"ppb");//显示RSSI
            }else{
                //隐藏
                viewHolder.deviceName.setVisibility(View.GONE);
                viewHolder.deviceAddress.setVisibility(View.GONE);
                viewHolder.deviceBroadcastPack.setVisibility(View.GONE);
                viewHolder.deviceRssi.setVisibility(View.GONE);
                viewHolder.placeholder.setVisibility(View.GONE);
            }


            /*if (deviceName != null && deviceName.length() > 0)
                viewHolder.deviceName.setText(deviceName);
            else
                viewHolder.deviceName.setText(R.string.unknown_device);
            viewHolder.deviceAddress.setText(deviceAddr);//显示设备地址
            viewHolder.deviceBroadcastPack.setText(broadcastPack);//显示广播包
            viewHolder.deviceRssi.setText(rssiString);//显示RSSI*/


            /*if(uuid.equals(target_uuid)){
                tv_temp.setText(temp+"℃");
                tv_hum.setText(hum+"%");
                tv_tvoc.setText(tvoc+"ppm");
                tv_eco2.setText(eco2+"ppb");
            }*/


            return view;

        }

    }

    //scanRecords的格式转换
    static final char[] hexArray = "0123456789ABCDEF".toCharArray();
    private static String bytesToHex(byte[] bytes) {
        char[] hexChars = new char[bytes.length * 2];
        for (int j = 0; j < bytes.length; j++) {
            int v = bytes[j] & 0xFF;
            hexChars[j * 2] = hexArray[v >>> 4];
            hexChars[j * 2 + 1] = hexArray[v & 0x0F];
        }
        return new String(hexChars);
    }

    // Device scan callback.
    private BluetoothAdapter.LeScanCallback mLeScanCallback = new BluetoothAdapter.LeScanCallback() {

                @Override
                public void onLeScan(final BluetoothDevice device, final int rssi, final byte[] scanRecord) {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mLeDeviceListAdapter.addDevice(device,rssi,scanRecord);
                            mLeDeviceListAdapter.notifyDataSetChanged();
                        }
                    });
                }
            };

    static class ViewHolder {
        TextView deviceName;
        TextView deviceAddress;
        TextView deviceBroadcastPack;    //加入广播包数据
        TextView deviceRssi;            //加入RSSI

        TextView placeholder;
    }

}