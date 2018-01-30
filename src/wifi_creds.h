/*
 * wifi_creds.h
 * Description: Specify any wifi credentials in this file to be automatically
 *              added to your Particle Photon.
 * NOTE: If you specify any credentials here, all other credentials
 *       stored on your photon will be cleared.
 * NOTE: Credentials should never be stored in version control. If you plan
 *       to specify credentials here, you should not commit them to your repo.
 * Author: Patrick Blesi
 * Date: 2018-01-29
 *
 */

// Uncomment the line below if specifying credentials in this file
// #define WIFI_CREDENTIALS_SPECIFIED

const String wifiCredentials[][2] = {
  // Set wifi creds here (up to 5) (last entry will be tried first)
  // {"SSID", "password"}
};