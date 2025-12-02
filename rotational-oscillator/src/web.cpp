/*
All of web.cpp was created by a Generative AI model and only verified by myself.
This is because the web functionality is not the main scope of my project.
Therefore, I have offloaded this task to AI.
However, all other code, unless stated otherwise, has been programmed by myself
*/

/**
 * @file web.cpp
 * * Handles all Wi-Fi, mDNS, and Web Server functionality for the ESP32.
 * * CONFIGURED AS ACCESS POINT (HOTSPOT).
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <vector>

// ----------------------------------------------------------------------------
// Type Definitions (from main file)
// ----------------------------------------------------------------------------

// Data structure for logging (MUST MATCH main.cpp)
struct DataPoint {
    uint32_t timestamp;
    float sensorValue;
    int selectedSensor;
    float error;
    float controlOutput;
    int strategyUsed;
};

// ----------------------------------------------------------------------------
// External State Variables (defined in main.cpp)
// ----------------------------------------------------------------------------

extern volatile bool running;
extern volatile int selectedStrategy;
extern volatile int selectedSensor;
extern float calibMiddle; // To display on web page
extern std::vector<DataPoint> dataLog;

// ----------------------------------------------------------------------------
// Web Server Globals
// ----------------------------------------------------------------------------

// AP Credentials (The ESP32 will CREATE this network)
const char* ssid = "ESP32-Control";
const char* password = "12345678"; // Must be at least 8 chars

// WebServer object
WebServer server(80);

// ----------------------------------------------------------------------------
// Placeholder / Stub Functions
// ----------------------------------------------------------------------------

void calibrateLowStep(int selectedSensor);
void calibrateHighStep(int selectedSensor);

// ----------------------------------------------------------------------------
// HTML Helper
// ----------------------------------------------------------------------------

// Generates the HTML page dynamically to update icons and values
String getHTML() {
    String html = "<!DOCTYPE html><html><head><title>ESP32 Control</title>";
    html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
    html += "<style>";
    html += "body { font-family: Arial, sans-serif; margin: 20px; background: #f4f4f4; }";
    html += "h1 { color: #333; text-align: center; }";
    html += ".container { max-width: 600px; margin: auto; padding: 20px; background: #fff; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }";
    html += "button, select { display: block; width: 100%; padding: 12px; margin: 10px 0; font-size: 16px; border: none; border-radius: 5px; cursor: pointer; box-sizing: border-box; }";
    html += "select { background: #eee; }";
    html += "button { color: white; }";
    html += ".start { background: #28a745; } .start:hover { background: #218838; }";
    html += ".stop { background: #dc3545; } .stop:hover { background: #c82333; }";
    html += ".calib { background: #ffc107; color: #333; } .calib:hover { background: #e0a800; }";
    html += ".download { background: #17a2b8; } .download:hover { background: #138496; }";
    html += ".btn-blue { background: #007bff; } .btn-blue:hover { background: #0056b3; }";
    
    // Status Icon CSS
    html += ".status-box { text-align: center; margin-bottom: 20px; padding: 10px; background: #eee; border-radius: 5px;}";
    html += ".status-dot { height: 15px; width: 15px; border-radius: 50%; display: inline-block; margin-right: 8px; vertical-align: middle; }";
    html += ".running { background-color: #28a745; box-shadow: 0 0 8px #28a745; }";
    html += ".stopped { background-color: #dc3545; }";
    html += "</style></head><body>";
    
    html += "<div class=\"container\">";
    html += "<h1>ESP32 Motor Control</h1>";

    // Status Section
    html += "<div class=\"status-box\">";
    if (running) {
        html += "Status: <span class=\"status-dot running\"></span> <strong>RUNNING</strong>";
    } else {
        html += "Status: <span class=\"status-dot stopped\"></span> <strong>STOPPED</strong>";
    }
    html += "<br>Current Strategy: <strong>" + String(selectedStrategy) + "</strong>";
    html += "</div>";

    // Main Controls
    html += "<form action=\"/start\" method=\"GET\"><button class=\"start\" type=\"submit\">Start Experiment</button></form>";
    html += "<form action=\"/stop\" method=\"GET\"><button class=\"stop\" type=\"submit\">Stop Experiment</button></form>";

    html += "<hr>";
    html += "<h3>Calibration</h3>";
    html += "<p>1. Move rig to LOW position:</p>";
    html += "<form action=\"/calibLow\" method=\"GET\"><button class=\"calib\" type=\"submit\">Record Low</button></form>";
    html += "<p>2. Move rig to HIGH position:</p>";
    html += "<form action=\"/calibHigh\" method=\"GET\"><button class=\"calib\" type=\"submit\">Record High</button></form>";
    html += "<p><em>Calculated Middle: " + String(calibMiddle) + "</em></p>";

    html += "<hr>";
    html += "<form action=\"/download\" method=\"GET\"><button class=\"download\" type=\"submit\">Download CSV Data</button></form>";
    
    html += "<hr>";

    // Strategy Selector with Persistence
    html += "<form action=\"/setStrategy\" method=\"GET\">";
    html += "<label for=\"strategy\">Control Strategy:</label>";
    html += "<select name=\"value\" id=\"strategy\">";
    
    String s0 = (selectedStrategy == 0) ? "selected" : "";
    String s1 = (selectedStrategy == 1) ? "selected" : "";
    String s2 = (selectedStrategy == 2) ? "selected" : "";
    String s3 = (selectedStrategy == 3) ? "selected" : "";

    html += "<option value=\"0\" " + s0 + ">Strategy 0 (Idle)</option>";
    html += "<option value=\"1\" " + s1 + ">Strategy 1 (P-Only)</option>";
    html += "<option value=\"2\" " + s2 + ">Strategy 2 (ON/OFF)</option>";
    html += "<option value=\"3\" " + s3 + ">Strategy 3 (PID)</option>";
    html += "</select>";
    html += "<button class=\"btn-blue\" type=\"submit\">Set Strategy</button>";
    html += "</form>";

    html += "</div></body></html>";
    return html;
}

// ----------------------------------------------------------------------------
// Web Server Route Handlers
// ----------------------------------------------------------------------------

void handleRoot() {
    server.send(200, "text/html", getHTML());
}

void handleStart() {
    running = true;
    server.sendHeader("Location", "/"); 
    server.send(302, "text/plain", "Starting...");
}

void handleStop() {
    running = false;
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Stopping...");
}

// New split handlers
void handleCalibLow() {
    calibrateLowStep(selectedSensor);
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Low Set");
}

void handleCalibHigh() {
    calibrateHighStep(selectedSensor);
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "High Set");
}

void handleSetStrategy() {
    if (server.hasArg("value")) {
        int strategy = server.arg("value").toInt();
        selectedStrategy = strategy;
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Strategy set.");
}

void handleDownload() {
    server.sendHeader("Content-Disposition", "attachment; filename=datalog.csv");
    // Chunked transfer to prevent RAM crash
    server.setContentLength(CONTENT_LENGTH_UNKNOWN); 
    server.send(200, "text/csv", ""); 

    String header = "timestamp,sensorValue,selectedSensor,error,controlOutput,strategyUsed\n";
    server.sendContent(header);

    String chunk = "";
    // Note: In a real-time OS, we might want a mutex here, but for this simple loop it is okay
    for (const auto& dp : dataLog) {
        chunk = String(dp.timestamp) + "," +
                String(dp.sensorValue) + "," +
                String(dp.selectedSensor) + "," +
                String(dp.error) + "," +
                String(dp.controlOutput) + "," +
                String(dp.strategyUsed) + "\n";
        server.sendContent(chunk);
    }
    
    server.sendContent(""); 
}

void handleNotFound() {
    server.send(404, "text/plain", "404: Not Found");
}

// ----------------------------------------------------------------------------
// Public Functions
// ----------------------------------------------------------------------------

void initWebServer() {
    // SET TO ACCESS POINT MODE
    WiFi.mode(WIFI_AP);
    
    // Create the network
    Serial.println("Creating Access Point...");
    WiFi.softAP(ssid, password);

    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP Created! Network Name: ");
    Serial.println(ssid);
    Serial.print("Connect to this network and visit: http://");
    Serial.println(myIP);

    // Start mDNS (optional in AP mode, IP is safer)
    if (MDNS.begin("esprig")) {
        Serial.println("mDNS responder started: http://esprig.local");
    }

    server.on("/", HTTP_GET, handleRoot);
    server.on("/start", HTTP_GET, handleStart);
    server.on("/stop", HTTP_GET, handleStop);
    server.on("/calibLow", HTTP_GET, handleCalibLow);   // New
    server.on("/calibHigh", HTTP_GET, handleCalibHigh); // New
    server.on("/setStrategy", HTTP_GET, handleSetStrategy);
    server.on("/download", HTTP_GET, handleDownload);
    server.onNotFound(handleNotFound);

    server.begin();
    Serial.println("HTTP server started");
}

void handleWebServer() {
    server.handleClient();
}