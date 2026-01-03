# ESP32 WiFi Provisioning Framework Documentation

## Overview

This document describes the WiFi provisioning framework implemented for ESP32 devices. The framework allows users to configure WiFi credentials for the device through a web interface, without requiring hardcoding of network credentials in the firmware.

## Features

1. **SoftAP Mode**: The ESP32 creates its own WiFi network (Access Point) for provisioning.
2. **Web Interface**: Users can connect to the SoftAP and access a web page to enter WiFi credentials.
3. **Credential Storage**: WiFi credentials are stored in Non-Volatile Storage (NVS) for persistent storage.
4. **Automatic Reconnection**: The device attempts to connect to previously saved networks on startup.
5. **Fallback Provisioning**: If no saved credentials are found, the device enters provisioning mode.

## Architecture

The framework consists of the following components:

### 1. WiFi Management
- SoftAP creation for provisioning
- Station mode for connecting to user WiFi
- Event handling for connection status

### 2. Web Server
- HTTP server for serving the configuration page
- REST endpoints for receiving WiFi credentials
- Static HTML page with form for SSID/password input

### 3. Storage
- NVS storage for persisting WiFi credentials
- Automatic retrieval of saved credentials on boot

### 4. Provisioning Flow
1. Device starts and tries to connect to saved WiFi
2. If no saved credentials, starts SoftAP and web server
3. User connects to SoftAP and accesses configuration page
4. User enters WiFi credentials and submits
5. Device saves credentials and attempts connection
6. On successful connection, device operates normally
7. On reboot, device automatically connects to saved network

## Implementation Details

### Configuration Parameters

The framework uses the following default parameters:

- **SoftAP SSID**: `ESP32_PROV`
- **SoftAP Password**: `12345678`
- **SoftAP IP Address**: `192.168.4.1`
- **Maximum Station Connections**: 4

These can be modified in the code by changing the following constants:
```c
#define EXAMPLE_ESP_WIFI_SSID      "ESP32_PROV"
#define EXAMPLE_ESP_WIFI_PASS      "12345678"
#define EXAMPLE_MAX_STA_CONN       4
```

### Key Functions

#### `wifi_init_softap()`
Initializes the WiFi subsystem in SoftAP+Station mode, creating a local network for provisioning.

#### `try_saved_wifi()`
Attempts to connect to previously saved WiFi credentials stored in NVS.

#### `start_webserver()`
Starts the HTTP server that serves the configuration page and handles credential submission.

#### `root_get_handler()`
Handles HTTP GET requests to the root path, serving the WiFi configuration HTML page.

#### `save_post_handler()`
Handles HTTP POST requests to `/save`, receiving and storing WiFi credentials.

#### Event Handlers
Handles WiFi and IP events such as station connections, disconnections, and IP acquisition.

### HTML Configuration Page

The configuration page is embedded in the firmware as a string constant. It includes:
- Input fields for SSID and password
- Form validation
- Responsive design for mobile devices

## Usage Flow

### Initial Setup
1. Flash the firmware to the ESP32 device
2. Power on the device
3. Connect to the WiFi network named `ESP32_PROV` with password `12345678`
4. Open a web browser and navigate to `http://192.168.4.1`
5. Enter the target WiFi network SSID and password
6. Click "Connect"
7. The device will attempt to connect to the specified network

### Subsequent Boots
1. The device automatically retrieves saved credentials from NVS
2. The device attempts to connect to the saved network
3. If successful, the device operates normally
4. If connection fails, the device re-enters provisioning mode

## Dependencies

The framework requires the following ESP-IDF components:
- `esp_wifi`: WiFi functionality
- `nvs_flash`: Non-volatile storage
- `esp_http_server`: Web server functionality
- `esp_netif`: Network interface management

## Security Considerations

1. The provisioning network uses WPA2 security by default
2. Passwords are transmitted over the local network during provisioning
3. Credentials are stored in NVS without encryption
4. In production environments, consider:
   - Using HTTPS for the configuration page
   - Implementing password encryption
   - Adding authentication to the provisioning interface

## Customization

### Changing SoftAP Settings
Modify the `EXAMPLE_ESP_WIFI_SSID` and `EXAMPLE_ESP_WIFI_PASS` constants in the source code.

### Modifying HTML Page
Edit the `config_page_html` constant in the source code to customize the appearance and functionality of the configuration page.

### Adjusting Connection Parameters
Change the `EXAMPLE_MAX_STA_CONN` constant to allow more or fewer simultaneous connections to the SoftAP.

## Troubleshooting

### Device Not Appearing in WiFi Networks
- Ensure the device is powered on
- Check serial output for error messages
- Verify the firmware was flashed correctly

### Unable to Access Configuration Page
- Ensure you're connected to the correct SoftAP network
- Verify the IP address (should be 192.168.4.1)
- Try refreshing the page or using a different browser

### Connection to Target Network Fails
- Verify SSID and password are correct
- Check if the target network is within range
- Ensure the target network supports the device's WiFi standard

### Credentials Not Persisting
- Check NVS initialization
- Verify sufficient flash storage space
- Check for NVS partition configuration issues

## API Reference

### Functions

#### `void wifi_init_softap(void)`
Initializes the WiFi system in SoftAP+Station mode.

#### `bool try_saved_wifi(void)`
Attempts to connect to saved WiFi credentials.
- Returns: `true` if credentials were found and connection attempted, `false` otherwise

#### `httpd_handle_t start_webserver(void)`
Starts the provisioning web server.
- Returns: Handle to the HTTP server instance

### Constants

#### `EXAMPLE_ESP_WIFI_SSID`
SSID for the provisioning SoftAP network.

#### `EXAMPLE_ESP_WIFI_PASS`
Password for the provisioning SoftAP network.

#### `EXAMPLE_MAX_STA_CONN`
Maximum number of simultaneous connections to the SoftAP.

## Future Enhancements

1. Add support for WiFi network scanning and selection
2. Implement secure credential transmission (HTTPS)
3. Add timeout mechanism for provisioning mode
4. Support for multiple network profiles
5. Integration with cloud services for remote provisioning
6. Enhanced security with credential encryption

## Conclusion

This WiFi provisioning framework provides a simple and effective way to configure WiFi credentials for ESP32 devices without requiring firmware modifications. The web-based approach makes it user-friendly and accessible from any device with a web browser.