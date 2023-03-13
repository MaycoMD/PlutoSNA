## Pluto SDR Reflectometer Scan (March 2018)
# Use Pluto SDR with a Mini-Circuits ZHDC-10-63-5+ directional coupler
# Create a 5kHz baseband tone with txrepeat()
# 1. Drive OUT port of directional coupler with Tx
# 2. Attach DUT to IN port
# 3. Sample the reflected signal at COUPL port with Rx
#
## SETUP:
# Parameters for frequency scan:
cfstart = 900e6  # Start frequency
cfstop = 950e6  # Stop frequency
step = 0.1e6 
pmax = 100  # Promedio
nstep = (cfstop-cfstart)/step+1 #594  # Number of frequency steps
#cfreq = linspace(cfstart,cfstop,nstep) 
cfreq = (cfstart:step:cfstop) 
cal = zeros(nstep,1)  # Array for calibration amplitudes
amp = zeros(nstep,1)  # Array for DUT amplitudes
rl = zeros(nstep,1)  # Array for DUT return loss

tx = comm.SDRTxPluto  # SDRTxPluto system object for transmitter
tx.CenterFrequency = cfreq(1)  # Tuner frequency in Hz
tx.Gain = 0  # Transmitter gain in dB (-89.75 to 0 dB)
tx.BasebandSampleRate = 1e5  # DAC sampling rate in Hz
# Generate the 5kHz test tone:
sw = dsp.SineWave 
sw.Amplitude = 1 
sw.Frequency = 5e3 
sw.ComplexOutput = true 
sw.SampleRate = tx.BasebandSampleRate 
sw.SamplesPerFrame = 5000 
txdata = conj(sw())  # Use conj() to generate the lower sideband

rx = comm.SDRRxPluto  # SDRRxPluto system object for receiver
rx.CenterFrequency = tx.CenterFrequency  # Tuner frequency in Hz
rx.GainSource = 'Manual'  # Disables the AGC
rx.Gain = 50  # Tuner gain in dB (-4 to 71 dB)
rx.BasebandSampleRate = tx.BasebandSampleRate  # ADC sampling rate in Hz
rx.SamplesPerFrame = 5000  # Output data frame size
rx.OutputDataType='int16'  # Output data type
ndec = 4  # Decimation factor

## SCANS
# Check that the PlutoSDR is active:
if ~isempty(findPlutoRadio)
fmin = 0 
df = rx.BasebandSampleRate/rx.SamplesPerFrame 
fmax = rx.BasebandSampleRate/ndec-df 
freq = linspace(fmin,fmax,rx.SamplesPerFrame/ndec)/1000  # freq in kHz 
index = sw.Frequency/df+1 

## CALIBRATION LOOP:
#
input('Remove DUT - then press any key')
for n = 1: nstep
    txfreq = cfreq(n) 
    tx.CenterFrequency = txfreq  # Tuner frequency in Hz
    tx.transmitRepeat(txdata)  # Repeated transmission
    rx.CenterFrequency = tx.CenterFrequency  # Tuner frequency in Hz
    for m = 1:10 # Read multiple times to clear the Rx buffer
        data = rx()  # Fetch a frame from the Pluto SDR
    end
    ddata = decimate(double(data),ndec) 
    spec = ifft(ddata)  # Signal amplitude at 5kHz is in spec(51)
    cal0(n) = abs(spec(index)) 
    for p = 1:pmax
        ddata = decimate(double(data),ndec) 
        spec = ifft(ddata)  # Signal amplitude at 5kHz is in spec(51)
        cal(n) = ( abs(spec(index)) + cal0(n) )./2 
    end
    figure(1) 
    plot(1e-6*cfreq,cal) 
    xlabel('Frequency / MHz') 
    ylabel('Amplitude') 
    title('Reflectometer Calibration Scan') 
    drawnow  
end

## Measure the reflection from the DUT:
#
input('Attach DUT - then press any key')
for n = 1: nstep
    txfreq = cfreq(n) 
    tx.CenterFrequency = cfreq(n)  # Tuner frequency in Hz
    tx.transmitRepeat(txdata)  # Repeated transmission
    rx.CenterFrequency = tx.CenterFrequency  # Tuner frequency in Hz
    for m = 1:10 # Read multiple times to clear the Rx buffer
        data = rx()  # Fetch a frame from the Pluto SDR
    end
    ddata = decimate(double(data),ndec) 
    spec = ifft(ddata)  # Signal amplitude at 5kHz is in spec(51)
    amp0(n) = abs(spec(index)) 
    for p=1:pmax
        ddata = decimate(double(data),ndec) 
        spec = ifft(ddata)  # Signal amplitude at 5kHz is in spec(51)
        amp(n) = ( abs(spec(index)) + amp0(n) )./2 
    end
    rl(n) = 20*log10(amp(n)/cal(n))  # DUT return loss
    if rl(n)==-Inf
       rl(n) = -100 
    end
    figure(2) 
    plot(1e-6*cfreq,rl) 
    xlabel('Frequency / MHz') 
    ylabel('Return Loss / dB') 
    title('PlutoSDR Reflectometer: DUT Scan') 
    drawnow 
end
## RELEASE SYSTEM OBJECTS:
release(rx) 
release(tx) 
else
    warning(message('plutoradio:sysobjdemos:PlutoRadioNotFound')) 
end
