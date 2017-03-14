%% Choice between a multisine and a crested multisine

% Creating a multisine signal as used in SIPE
% Specifing parameters
A = 0.1;        % Amplification of the input signal
T_end = 50;     % end time of simulation
fs = 100;       % sampling frequency
fcut = 5;      % cutoff frequency for multisine signal
dt = 1/fs;
N = T_end/dt;
Nseg = N/10;
t = (0:N-1).'*dt; % time vector

% Random phase multisine signal from 0 to fcut 
fv = (0:Nseg-1)'/Nseg*fs;                % Make a new frequency for the shorter Multisine FFT 
P  = 10;                                  % Amount of periods (10 of 20 seconds)
Tm = T_end./P;                           % Duration of 1 period
freqv=1/Tm:1/Tm:fcut;                    % array of frequencies
cx=exp(1j*rand(length(freqv),1)*2*pi);   % amplitude and phase in freq domain
[xp,~]=msinprep(freqv,cx,Nseg,fs,'screen'); % Generate time series (multisine)

xp_max = max(xp);
xp = (xp/xp_max)*A;                              % Amplitude of 1.5

% Implement repmat
xp_long = repmat(xp,P,1);                        % Create a long signal consisting of multiple periods

Xp=fft(xp);                              % Fourier transform multisine
SXp = 1/N*Xp.*conj(Xp);                  % Auto spectral density

% Crested multisine signal, improved SNR!
[cx2,crinfo]=msinclip(freqv,cx,[],[],50,[],N);  % Crested with 50 iterations
[xpc2,~]=msinprep(freqv,cx2,Nseg,fs,'screen');

xpc2_max = max(xpc2);
xpc2 = (xpc2/xpc2_max)*A; 

% Implement repmat
xpc2_long = repmat(xpc2,P,1);                        % Create a long signal consisting of multiple periods

Xpc2 = fft(xpc2);                        % Fourier transfrom crested multisine
SXpc2 = 1/N*Xpc2.*conj(Xpc2);            % Auto spectral density

figure(1);
plot(t,xp_long,t,xpc2_long)
figure(2);
semilogy(fv,abs(SXp)); hold on 
semilogy(fv,abs(SXpc2));
