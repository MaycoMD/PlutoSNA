#%% Pluto SDR Reflectometer Scan (March 2018)
# Use Pluto SDR with a Mini-Circuits ZHDC-10-63-5+ directional coupler
# Create a 5kHz baseband tone with txrepeat()
# 1. Drive OUT port of directional coupler with Tx
# 2. Attach DUT to IN port
# 3. Sample the reflected signal at COUPL port with Rx
#

#%% LIBRARIES:
from scipy import signal as sg
import numpy as np
import adi

## SETUP:
# Parameters for frequency scan:
cfstart = 70e6  # Start frequency
cfstop = 3000e6  # Stop frequency
step = 1e6
pmax = 1  # Promedio
nstep = int((cfstop-cfstart)/step+1) #594  # Number of frequency steps
cfreq = np.linspace(cfstart,cfstop,nstep)  
cal = np.zeros((nstep,1))  # Array for calibration amplitudes
amp = np.zeros((nstep,1))  # Array for DUT amplitudes
rl = np.zeros((nstep,1))  # Array for DUT return loss

#%% PLUTO PARAMETERS:
# Create radio
sdr = adi.ad9361(uri="ip:192.168.2.1")
sdr.tx_enabled_channels = [0]
sdr.rx_enabled_channels = [0]

sdr.sample_rate = int(1e6)  # ADC sampling rate in Hz

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

#%% Generate the 5kHz test tone: 
swAmplitude = 1 
swFrequency = 5e3 
swSampleRate = sdr.sample_rate
swSamplesPerFrame = 5000
#txdata = conj(sw())  # Use conj() to generate the lower sideband
ndec = 4  # Decimation factor

N = 10000 # number of samples to transmit at once
t = np.arange(N)/sdr.sample_rate
txdata = 0.5*np.exp(2.0j*np.pi*100e3*t) # Simulate a sinusoid of 100 kHz, so it should show up at 915.1 MHz at the receiver
txdata *= 2**14 # The PlutoSDR expects samples to be between -2^14 and +2^14, not -1 and +1 like some SDRs


# Set transmitter:
sdr.tx_lo = int(cfreq[1])

#%% SCANS
# Check that the PlutoSDR is active:
#if not isempty(findPlutoRadio)
fmin = 0 
df = sdr.sample_rate/sdr.rx_buffer_size
fmax = sdr.sample_rate/ndec-df 
freq = np.linspace(fmin,fmax,int(sdr.rx_buffer_size/ndec))/1000  # freq in kHz 
index = swFrequency/df+1 

#%% CALIBRATION LOOP:
#
#input('Remove DUT - then press any key')
for n in range(nstep):
    txfreq = cfreq[n]
    sdr.tx_lo = int(txfreq)  # Tuner frequency in Hz
    sdr.rx_lo = int(txfreq)  # Tuner frequency in Hz
    for m in range(10): # Read multiple times to clear the Rx buffer
    ddata = sg.decimate(float(data),ndec) 
    spec = ifft(ddata)  # Signal amplitude at 5kHz is in spec(51)
    cal0[n] = abs(spec(index)) 
    for p in range(pmax):
        ddata = sg.decimate(np.double(data),ndec) 
        spec = ifft(ddata)  # Signal amplitude at 5kHz is in spec(51)
        cal[n] = ( abs(spec(index)) + cal0(n) ) /2 
    figure(1) 
    plot(1e-6*cfreq,cal) 
    xlabel('Frequency / MHz') 
    ylabel('Amplitude') 
    title('Reflectometer Calibration Scan') 
    drawnow  

#%% Measure the reflection from the DUT:
#
input('Attach DUT - then press any key')
for n in range(nstep):
    txfreq = cfreq(n) 
    tx.CenterFrequency = cfreq(n)  # Tuner frequency in Hz
    tx.transmitRepeat(txdata)  # Repeated transmission
    rx.CenterFrequency = tx.CenterFrequency  # Tuner frequency in Hz
    for m in range(10): # Read multiple times to clear the Rx buffer
        data = rx()  # Fetch a frame from the Pluto SDR
    end
    ddata = decimate(double(data),ndec) 
    spec = ifft(ddata)  # Signal amplitude at 5kHz is in spec(51)
    amp0[n] = abs(spec(index)) 
    for p in range(pmax):
        ddata = decimate(double(data),ndec) 
        spec = ifft(ddata)  # Signal amplitude at 5kHz is in spec(51)
        amp[n] = ( abs(spec(index)) + amp0(n) ) /2 
    end
    rl[n] = 20*log10(amp(n)/cal(n))  # DUT return loss
    #if rl[n]==-Inf
    if rl[n]<=-10:
       rl[n] = -100 
    end
    figure(2) 
    plot(1e-6*cfreq,rl) 
    xlabel('Frequency / MHz') 
    ylabel('Return Loss / dB') 
    title('PlutoSDR Reflectometer: DUT Scan') 
    drawnow 
end

#%% RELEASE SYSTEM OBJECTS:
release(rx) 
release(tx) 
#else
#    warning(message('plutoradio:sysobjdemos:PlutoRadioNotFound')) 
#end
