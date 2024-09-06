#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager


void setup() {
//     WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
    // it is a good practice to make sure your code sets wifi mode how you want it.

    // put your setup code here, to run once:
    Serial.begin(115200);
    
    //WiFiManager, Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wfm;
    
    wfm.setDebugOutput(false);
    // reset settings - wipe stored credentials for testing
    // these are stored by the esp library
    
    wfm.resetSettings();

    WifiManagerParameter custom_text_box("my_text" , "Enter your string here" , "default string" , 50);
    
    wfm.addParameter(&custom_text_box);
    // Automatically connect using saved credentials,
    // if connection fails, it starts an access point with the specified name ( "AutoConnectAP"),
    // if empty will auto generate SSID, if password is blank it will be anonymous AP (wm.autoConnect())
    // then goes into a blocking loop awaiting configuration and will return success result

    if(!wfm.autoConnect("ESP32TEST_AP" , "password")){
      Serial.println("Failed to connect and hit timeout");
      ESP.restart();
      delay(1000);
      
    }

//    connected
    Serial.println("WiFi connected");
    Serial.print("IP address:");
    Serial.println(WiFi.localIP());

    Serial.println("Custom text box entry:");
    Serial.println(custom_text_box.getValue());
}

void loop() {
    // put your main code here, to run repeatedly:   
}
