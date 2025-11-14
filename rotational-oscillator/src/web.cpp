/*
All of web.cpp was created by a Generative AI model and only verified by myself.
This is because the web functionality is not the main scope of my project.
Therefore, I have offloaded this task to AI.
However, all other code, unless stated otherwise, has been programmed by myself
*/

/**
 * @file web.cpp
 * * Handles all Wi-Fi, mDNS, and Web Server functionality for the ESP32.
 * Provides a web interface to start/stop the experiment, download data,
 * trigger calibration, and select a control strategy.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <vector>

// ----------------------------------------------------------------------------
// Type Definitions (from main file)
// ----------------------------------------------------------------------------

// Data structure for logging
struct DataPoint {
    uint32_t timestamp;
    float sensorValue;
    int selectedSensor;
    float controlOutput;
    int strategyUsed;
};

// ----------------------------------------------------------------------------
// External State Variables (defined in main.cpp)
// ----------------------------------------------------------------------------

// These variables are controlled by the web server
extern volatile bool running;
extern volatile int selectedStrategy;

// This data log is read by the web server for download
extern std::vector<DataPoint> dataLog;

// ----------------------------------------------------------------------------
// Web Server Globals
// ----------------------------------------------------------------------------

// Wi-Fi Credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// WebServer object
WebServer server(80);

// ----------------------------------------------------------------------------
// Placeholder / Stub Functions (to be implemented in main.cpp)
// ----------------------------------------------------------------------------

// Forward declaration (calibrate is declared in main.cpp)
void calibrate();

// ----------------------------------------------------------------------------
// HTML/CSS for Web UI
// ----------------------------------------------------------------------------

// We store the HTML page in PROGMEM (Flash) to save RAM
const char MAIN_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f4f4f4; }
        h1 { color: #333; }
        .container { max-width: 600px; margin: auto; padding: 20px; background: #fff; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
        button, select {
            display: block;
            width: 100%;
            padding: 12px;
            margin: 10px 0;
            font-size: 16px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            box-sizing: border-box; /* Ensures padding doesn't affect width */
        }
        select { background: #eee; }
        button { background: #007bff; color: white; }
        button:hover { background: #0056b3; }
        .start { background: #28a745; }
        .start:hover { background: #218838; }
        .stop { background: #dc3545; }
        .stop:hover { background: #c82333; }
        .download { background: #17a2b8; }
        .download:hover { background: #138496; }
        form { margin-bottom: 10px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>ESP32 Motor Control</h1>

        <form action="/start" method="GET">
            <button class="start" type="submit">Start Experiment</button>
        </form>
        <form action="/stop" method="GET">
            <button class="stop" type="submit">Stop Experiment</button>
        </form>

        <hr>

        <form action="/calibrate" method="GET">
            <button type="submit">Calibrate</button>
        </form>
        <form action="/download" method="GET">
            <button class="download" type="submit">Download CSV Data</button>
        </form>
        
        <hr>

        <form action="/setStrategy" method="GET">
            <label for="strategy">Control Strategy:</label>
            <select name="value" id="strategy">
                <option value="0">Strategy 0 (e.g., PID)</option>
                <option value="1">Strategy 1 (e.g., P-Only)</option>
                <option value="2">Strategy 2 (e.g., ON/OFF)</option>
            </select>
            <button type="submit">Set Strategy</button>
        </form>
    </div>
</body>
</html>
)rawliteral";

// ----------------------------------------------------------------------------
// Web Server Route Handlers
// ----------------------------------------------------------------------------

/**
 * @brief Serves the main HTML page.
 */
void handleRoot() {
    server.send(200, "text/html", MAIN_page);
}

/**
 * @brief Sets the 'running' flag to true and redirects to root.
 */
void handleStart() {
    running = true;
    server.sendHeader("Location", "/"); // Redirect back to home page
    server.send(302, "text/plain", "Starting...");
}

/**
 * @brief Sets the 'running' flag to false and redirects to root.
 */
void handleStop() {
    running = false;
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Stopping...");
}

/**
 * @brief Calls the calibrate() function and redirects to root.
 */
void handleCalibrate() {
    calibrate(); // Call the placeholder function
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Calibrating...");
}

/**
 * @brief Reads the 'value' query parameter and updates 'selectedStrategy'.
 */
void handleSetStrategy() {
    if (server.hasArg("value")) {
        int strategy = server.arg("value").toInt();
        selectedStrategy = strategy;
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "Strategy set.");
}

/**
 * @brief Generates a CSV from the dataLog and sends it as a file.
 */
void handleDownload() {
    // Use a String to build the CSV content.
    // For very large logs, streaming the content is better, but this is simpler.
    String csv = "";
    
    // Add CSV Header
    csv += "timestamp,sensorValue,selectedSensor,controlOutput,strategyUsed\n";

    // Add data rows
    // We lock access here if necessary, but vector read is mostly safe.
    // If dataLog can be written to by an ISR, you MUST disable interrupts here.
    for (const auto& dp : dataLog) {
        csv += String(dp.timestamp) + ",";
        csv += String(dp.sensorValue) + ",";
        csv += String(dp.selectedSensor) + ",";
        csv += String(dp.controlOutput) + ",";
        csv += String(dp.strategyUsed) + "\n";
    }

    // Send the CSV file as an attachment
    server.sendHeader("Content-Disposition", "attachment; filename=datalog.csv");
    server.send(200, "text/csv", csv);
}

/**
 * @brief Handles 404 Not Found errors.
 */
void handleNotFound() {
    server.send(404, "text/plain", "404: Not Found");
}

// ----------------------------------------------------------------------------
// Public Functions (to be called from main.cpp)
// ----------------------------------------------------------------------------

/**
 * @brief Initializes WiFi, mDNS, and the WebServer routes.
 * Call this once from your setup() function.
 */
void initWebServer() {
    // Start Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
    // Start mDNS
    if (!MDNS.begin("esprig")) {
        // shouldn't happen (hopefully)
    } else {
        MDNS.addService("http", "tcp", 80);
        // should be accesible at http://esprig.local
    }

    // Define WebServer Routes
    server.on("/", HTTP_GET, handleRoot);
    server.on("/start", HTTP_GET, handleStart);
    server.on("/stop", HTTP_GET, handleStop);
    server.on("/calibrate", HTTP_GET, handleCalibrate);
    server.on("/setStrategy", HTTP_GET, handleSetStrategy);
    server.on("/download", HTTP_GET, handleDownload);
    server.onNotFound(handleNotFound);

    // Start the server
    server.begin();
    Serial.println("HTTP server started");
}

/**
 * @brief Handles incoming web server client requests.
 * Call this in your main loop() function.
 */
void handleWebServer() {
    server.handleClient();
}