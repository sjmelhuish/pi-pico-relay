# This file is executed on every boot
#import gc
import secrets, rp2

rp2.country('GB')  # change to your country code

def do_connect():
    import network
    sta_if = network.WLAN(network.STA_IF)
    ap_if = network.WLAN(network.AP_IF)
    if ap_if.active():
        ap_if.active(False)
        print("AP turned off ...")
    if not sta_if.isconnected():
        print('connecting to network...')
        sta_if.active(True)
        sta_if.connect(secrets.SSID,
                       secrets.PASSWORD)
        while not sta_if.isconnected():
            pass
    print('network config:', sta_if.ifconfig())

print("Starting network ...")
do_connect()