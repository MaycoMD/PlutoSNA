#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar  5 18:29:45 2023

@author: mayco
"""

import adi

# Create radio
sdr = adi.ad9361(uri="ip:192.168.2.1")
sdr.tx_enabled_channels = [0]
sdr.rx_enabled_channels = [0]

sdr.sample_rate = 1000000  # ADC sampling rate in Hz

# Transmitter properties
sdr.tx_lo = 2000000000 # Tx Local oscillator frequency in Hz
sdr.tx_cyclic_buffer = True
sdr.tx_hardwaregain_chan0 = -30 # Transmitter gain in dB (-89.75 to 0 dB)

# Receiver properties
sdr.rx_rf_bandwidth = 4000000
sdr.rx_lo = sdr.tx_lo
sdr.gain_control_mode_chan0 = "manual" # Disables the AGC
sdr.rx_hardwaregain_chan0 = 0 # Tuner gain in dB (-4 to 71 dB)
sdr.rx_buffer_size = 2**16 # Output data frame size
sdr.rx_output_type = 'raw'

# Read properties
print("TX LO %s MHz" % (sdr.tx_lo/1e6))
print("TX GAIN %s" % (sdr.tx_hardwaregain_chan0))
print("RX LO %s MHz" % (sdr.rx_lo/1e6))
print("RX BW %s MHz" % (sdr.rx_rf_bandwidth/1e6))
print("RX GAIN %s" % (sdr.rx_hardwaregain_chan0))
print("RX BUFFER %s" % (sdr.rx_buffer_size))