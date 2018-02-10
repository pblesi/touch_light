# touch_light

Simple wifi touch lights that synchronize their colors from across the globe.

## What

This is a [Particle](https://www.particle.io/) project for linking [wifi touch lights](http://patrick.ble.si/wifi-touch-light) together so their colors are synchronized from anywhere in the globe. This project was developed with the intent of connecting loved ones. You can touch the lights, setting them to a specific color and communicating with all others, that you are thinking of them. Pictures and details can be found on my [blog post](http://patrick.ble.si/wifi-touch-light).

## Usage

Once you have built the touch lights, you will need to download this software, configure it, compile it, and finally flash it onto your photon.

### Configuration

There are a number of aspects of the code that can be configured.

You can control serial and wifi debug statements by toggling the below keys:

```c++
// CONFIGURATION SETTINGS START
// DEBUG SETTINGS:
#define D_SERIAL false
#define D_WIFI false
```

You will want to set NUM_PARTICLES to the number of particles that will be synchronized with each other. You will need to place the id of each particle in the `particleId` array.

```c++
#define NUM_PARTICLES 4 // number of touch lights in your group
// Number each Filimin starting at 1.
String particleId[] = {
  "",                         // 0
  "330022001547353236343033", // pblesi
  "2d0047001247353236343033", // carol
  "2a0026000b47353235303037", // cindy
  "2e003e001947353236343033"  // tammy
};
```

These ids can be found at the command line by running `particle list`.

You can specify the default color for each touch light.

```c++
int particleColors[] = {
  0,   // Green
  90,  // Magenta
  170, // Blue
  79,  // Orange
  131  // Purple
};
```

This array corresponds to the above `particleId` array, such that the first color listed in this array corresponds to the first particle id listed in `particleId` and the second entries correspond, etc.

If you wish to specify your wifi credentials via code (for example if you are giving the touch lights away as gifts and want the wifi connection set up for when the light arrives), then you can use `wifi_creds.h` to set the wifi creds of the touch light without being in range of the router you want the light to connect to.

```c++
// Uncomment the line below if specifying credentials in this file
// #define WIFI_CREDENTIALS_SPECIFIED

// ...

const credentials wifiCreds[] = {
  // Set wifi creds here (up to 5) (last entry will be tried first)
  // {.ssid="SSID", .password="password", .authType=WPA2, .cipher=WLAN_CIPHER_AES}
};
```

See Particle's [WiFi.setCredentials](https://docs.particle.io/reference/firmware/photon/#setcredentials-) reference for more details on `authType` and `cipher` options.

### Compiling

Once you have properly configured your touch light code, you can compile it by running:

```bash
$ particle compile photon
```

This should be run from within the project's root directory (the directory containing `src`)

### Flashing

Finally, you can flash your touch light once you have compiled your source into a firmware file:

```bash
$ particle flash <photon_name> <firmware.bin file>
```

If all goes well, your touch light should cycle through a rainbow of colors, fade, and then be ready to be touched to change colors!
