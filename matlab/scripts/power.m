%% Measuring Power
%
% The |periodogram| function computes the signal's FFT and normalizes the
% output to obtain a power spectral density, PSD, or a power spectrum from
% which you can measure power. The PSD describes how the power of a time
% signal is distributed with frequency, it has units of watts/Hz. You
% compute the power spectrum by integrating each point of the PSD over the
% frequency interval at which that point is defined (i.e. over the
% resolution bandwidth of the PSD). The units of the power spectrum are
% watts. You can read power values directly from the power spectrum without
% having to integrate over an interval. Note that the PSD and power
% spectrum are real, so they do not contain any phase information.
%
% *Measuring Harmonics at the Output of a Non-Linear Power Amplifier*
%
% Load the data measured at the output of a power amplifier that has third
% order distortion of the form $v_o = v_i + 0.75 v_i^2 + 0.5 v_i^3$, where
% $v_o$ is the output voltage and $v_i$ is the input voltage. The data was
% captured with a sample rate of 3.6 kHz. The input $v_i$ consists of a 60
% Hz sinusoid with unity amplitude. Due to the nature of the non-linear
% distortion, you should expect the amplifier output signal to contain a DC
% component, a 60 Hz component, and second and third harmonics at 120 and
% 180 Hz.
%
% Load 3600 samples of the amplifier output, compute the power spectrum,
% and plot the result in a logarithmic scale (decibels-watts or dBW).

Fs = 100000; %100 kHz
NFFT = length(y);

% Power spectrum is computed when you pass a 'power' flag input
[P,F] = periodogram(y,[],NFFT,Fs,'power');   

plot (F,10*log10(P))
xlabel('Frequency in Hz')
ylabel ('Magnitude in dB')
title('Vibration data sampled at 100kSPS')
legend('Vibration of Vibration Motor at 3.3V')
grid on
axis([0 800 -100 -20])
%helperFrequencyAnalysisPlot2(F,10*log10(P),'Frequency in Hz','Power spectrum (dBW)',[],[],[-0.5 200])