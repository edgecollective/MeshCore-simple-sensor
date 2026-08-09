#!/usr/bin/env python3
"""
example_temp_poster.py
======================

A minimal Python script that mimics what the MeshCore ultrasonic receiver does
when it forwards a reading to Bayou: build a small JSON object with the feed's
private key plus a couple of measurement fields, and POST it over HTTP.

The only difference: instead of pulling a real distance from a LoRa DM, we make
up a random temperature every few seconds. Bayou doesn't care where the number
came from — the field name is what tells it "this is a temperature reading in
Celsius" and drives the axis labels on the feed page.

Usage
-----

  1. Create a Bayou feed (see the tutorial page next to this file, or just
     open https://bayou.pvos.org/ and click through) and copy the public and
     private keys the server hands back.
  2. Paste them into PUBLIC_KEY / PRIVATE_KEY below.
  3. Install requests if you don't already have it:  pip install requests
  4. Run:  python3 example_temp_poster.py
  5. Open  https://bayou.pvos.org/data/<your public key>/?plot_param=temperature_c
     in a browser and watch the values arrive.

To stop, hit Ctrl-C.
"""

import random
import time

import requests


# --- Configure these two for your own feed ------------------------------------
PUBLIC_KEY  = "YOUR_PUBLIC_KEY"
PRIVATE_KEY = "YOUR_PRIVATE_KEY"

# --- These usually don't change -----------------------------------------------
BAYOU_HOST     = "https://bayou.pvos.org"
POST_INTERVAL  = 10        # seconds between fake readings
NODE_ID        = 1         # pretend to be sensor #1 on this feed


def make_reading():
    """
    Build one JSON payload with the fields Bayou recognizes.

    The receiver firmware builds an equivalent object in C++ — see
    examples/v3-ultrasonic/companion_sensor_receiver/main.cpp, function
    `postToBayou`. It sends `distance_meters` + `battery_volts` + a few
    mesh-metadata fields; here we send a single `temperature_c` instead.
    """
    return {
        "private_key":   PRIVATE_KEY,        # required — proves we're allowed to write
        "node_id":       NODE_ID,            # optional — lets one feed hold several sensors
        "temperature_c": round(random.uniform(15.0, 25.0), 2),
        # You can add any other recognized fields alongside. Unknown fields are
        # silently ignored by Bayou, so a typo just drops the value on the floor.
        # "humidity_rh":   45.2,
        # "battery_volts": 3.87,
        # "log":           "test run",
    }


def main():
    if PUBLIC_KEY == "YOUR_PUBLIC_KEY" or PRIVATE_KEY == "YOUR_PRIVATE_KEY":
        raise SystemExit(
            "Edit the top of this script and paste in the public + private keys "
            "you got when you created your Bayou feed."
        )

    url = f"{BAYOU_HOST}/data/{PUBLIC_KEY}"
    print(f"Posting to {url} every {POST_INTERVAL}s. Ctrl-C to stop.")

    while True:
        payload = make_reading()
        try:
            r = requests.post(url, json=payload, timeout=10)
            body = r.text.strip()
            print(f"  temperature_c={payload['temperature_c']}  ->  "
                  f"HTTP {r.status_code}  {body}")
        except requests.RequestException as e:
            print(f"  POST failed: {e}")

        time.sleep(POST_INTERVAL)


if __name__ == "__main__":
    main()
