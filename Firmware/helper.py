import shutil
import gzip
import os
import esptool
import time
import webbrowser
import requests
import configparser
import subprocess
import tempfile
from serial.tools import list_ports
Import("env")


CONFIG = {
    "slaveApp": 0, # heat/cool
    "otMode": 1, # master
    "enableSlave": False,
    "boiler": {
        "dhwOn": True,
        "dhwTemperature": 50,
        "overrideDhw": False,
        "coolOn": False
    },
    "heating": [
        {
            "flow": 45,
            "roomtempcomp": 0,
            "chOn": True,
            "flowMax": 60,
            "exponent": 1.3,
            "gradient": 1.4,
            "offset": 0,
            "marker": [],
            "roomsetpoint": {
                "source": 0,
                "temp": 21
            },
            "roomtemp": {
                "source": 1
            },
            "overrideFlow": False
        },
        {
            "flow": 30,
            "roomtempcomp": 0,
            "chOn": True,
            "flowMax": 60,
            "exponent": 1.3,
            "gradient": 1.4,
            "offset": 0,
            "marker": [],
            "roomsetpoint": {
                "source": 0,
                "temp": 21
            },
            "roomtemp": {
                "source": 1
            },
            "overrideFlow": False
        }
    ],
    "vent": {
        "ventEnable": False,
        "openBypass": False,
        "autoBypass": False,
        "freeVentEnable": False,
        "setpoint": 3
    },
    "outsideTemp": {
        "source": 0,
        "apikey": "undefined",
        "lat": None,
        "lon": None,
        "interval": None
    },
    "mqtt": {
        "host": "",
        "port": 1883,
        "user": "",
        "pass": "",
        "tls": False
    },
    "masterMemberId": 8
}


platform = env.PioPlatform()
board = env.BoardConfig()
mcu = board.get("build.mcu", "esp32") # works for ESP8266 and ESP32

def copy_html(source, target, env):
    with open(os.path.join(env["PROJECT_DATA_DIR"], "index.html"), "r") as fin:
        with open(os.path.join(env["PROJECT_DIR"], "include/html.h"), "w") as fout:
            fout.write('const char html[] PROGMEM = R"html(')
            for line in fin:
                fout.write(line)
            fout.write('\n)html";')
    
def post_build(source, target, env):
    print("Version: " + env.GetProjectOption("custom_version"))
    print("project dir: " + env["PROJECT_DIR"])
    print("build: " + env["BUILD_DIR"])

def before_upload(source, target, env):
    devices = list_ports.comports()
    for d in devices:
        if (d.vid == 12346) and (d.pid == 4097):
            env.Replace(UPLOAD_PORT=d[0])
            break

def load_ap_credentials(env):
    """Read AP SSID/password from default.ini (secrets)."""
    ini = os.path.join(env["PROJECT_DIR"], "default.ini")
    cp = configparser.ConfigParser()
    cp.read(ini)
    ssid = cp.get("secrets", "ap_ssid", fallback="OT Thing")
    password = cp.get("secrets", "ap_password", fallback="12345678")
    return ssid, password

def ensure_windows_wifi_profile(ssid, password):
    """Create a Wi-Fi profile for the config AP on Windows if missing."""
    if os.name != "nt":
        return

    try:
        res = subprocess.run(
            ["netsh", "wlan", "show", "profiles"],
            capture_output=True,
            text=True,
            check=True,
        )
        if ssid in res.stdout:
            return
    except Exception as e:
        print("Could not list Wi-Fi profiles; skipping profile creation:", e)
        return

    profile_xml = f"""<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
    <name>{ssid}</name>
    <SSIDConfig>
        <SSID>
            <name>{ssid}</name>
        </SSID>
    </SSIDConfig>
    <connectionType>ESS</connectionType>
    <connectionMode>manual</connectionMode>
    <MSM>
        <security>
            <authEncryption>
                <authentication>WPA2PSK</authentication>
                <encryption>AES</encryption>
                <useOneX>false</useOneX>
            </authEncryption>
            <sharedKey>
                <keyType>passPhrase</keyType>
                <protected>false</protected>
                <keyMaterial>{password}</keyMaterial>
            </sharedKey>
        </security>
    </MSM>
</WLANProfile>
"""
    tmp = None
    try:
        with tempfile.NamedTemporaryFile("w", delete=False, suffix=".xml") as f:
            f.write(profile_xml)
            tmp = f.name
        subprocess.run(
            ["netsh", "wlan", "add", "profile", f"filename={tmp}", "user=all"],
            check=True,
        )
        print(f"Added Wi-Fi profile '{ssid}' for AP provisioning.")
    except Exception as e:
        print("Failed to add Wi-Fi profile for provisioning:", e)
    finally:
        if tmp and os.path.exists(tmp):
            os.remove(tmp)

def after_upload(source, target, env):
    upload_port = env.get("UPLOAD_PORT", None)
    if upload_port == None:
        env.AutodetectUploadPort()
        upload_port = env.get("UPLOAD_PORT", "none")
    print("Upload port", upload_port)
    with esptool.cmds.detect_chip(port=upload_port) as esp:
        mac = esp.read_mac()
        macstr = ":".join(map(lambda x: "%02X" % x, mac))
        print("MAC is ", macstr)
        esp.hard_reset()
        
        print("Bring target to config mode & press enter")
        input("")
        ap_ssid, ap_password = load_ap_credentials(env)
        ensure_windows_wifi_profile(ap_ssid, ap_password)
        try:
            subprocess.run(
                ["netsh", "wlan", "connect", f"name={ap_ssid}"],
                check=True,
            )
        except Exception as e:
            print(f"Failed to connect to '{ap_ssid}' via netsh:", e)
        time.sleep(2)
        webbrowser.open('http://4.3.2.1')
        print("Press enter to send default config to target")
        input("")
        resp = requests.post("http://4.3.2.1/config", json=CONFIG)
        print(resp)
        backoff = [1, 2, 3]
        for delay_s in backoff:
            time.sleep(delay_s)
            try:
                r = requests.get('http://4.3.2.1/config', timeout=3)
            except Exception as e:
                print(f"/config GET attempt failed ({e}); retrying...")
                continue

            if r.status_code != 200:
                print(f"/config GET returned {r.status_code}, body:\n{r.text}")
                continue
            try:
                conf = r.json()
                print("OT mode:", conf.get("otMode"))
                break
            except Exception as e:
                print("Failed to parse /config JSON:", e, "body:\n", r.text)
        else:
            print("Unable to read /config after retries.")


env.AddPreAction("$BUILD_DIR/src/portal.cpp.o", copy_html)
env.AddPreAction("upload", before_upload)
env.AddPostAction("buildprog", post_build)
env.AddPostAction("upload", after_upload)
