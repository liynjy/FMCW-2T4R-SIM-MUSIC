%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                FMCW 2T4R Radar Simulator               %
%                                                        %
% Author: Lin Junyang                                    %
% Email : liynjy@163.com                                 %
% Date  : 2019-10-20                                     %
%                                                        %
% This simulation is based on Matlab Phased Array System %
% Toolbox. For FMCW simulation without using Matlab Too- %
% lbox, refer to another project of "FMCW-SIM" in my Gi- %
% thub repository.                                       %
% This project contains:                                 %
%    1) 2T4R (or1T4R) Plannar Array Simulation.          %
%    2) MUSIC algorithm of DOA (Direction of Arrival)    %
%       estimation.                                      %
%                                                        %
% All Rights Reserved. @ 2019                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;clear;close all

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         Part I: Simulation                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
fc = 24e9;    % 24GHz, 60GHz, 77GHz, etc.
c = 3e8;
lambda = c/fc;

%%
tm = 5e-4;   % Chirp Cycle
bw = 3e9;    % FMCW Bandwidth
range_max = 3;     % Max detection Range 1~100 meters
v_max = 3;         % Max Velocity
%
range_res = c/2/bw;
sweep_slope = bw/tm;
fr_max = range2beat(range_max,sweep_slope,c);
fd_max = speed2dop(2*v_max,lambda);
fb_max = fr_max+fd_max;
fs = max(2*fb_max,bw);

% Use Phased Array System Toolbox to generate an FMCW waveform
waveform = phased.FMCWWaveform('SweepTime',tm,'SweepBandwidth',bw,...
    'SampleRate',fs);

%%
Nty = 2;                    % Tx antenna number y axis
Nry = 2;                    % Rx antenna number y axis
Nrz = 2;                    % Rx antenna number z axis

dty = lambda;
dry = lambda/2;
drz = lambda/2;

txarray = phased.ULA(Nty,dty);               % Tx Array 1x2
rxarray = phased.URA([Nrz Nry],[drz dry]);   % Rx Array 2x2
varray = phased.URA([Nrz Nty*Nry],[drz dry]);

figure;viewArray(txarray,'ShowIndex','All')
figure;viewArray(rxarray,'ShowIndex','All')
figure;viewArray(varray,'ShowIndex','All')
pause(0.1)

%%
transmitter = phased.Transmitter('PeakPower',0.001,'Gain',26);
receiver = phased.ReceiverPreamp('Gain',20,'NoiseFigure',4.5,'SampleRate',fs);

txradiator = phased.Radiator('Sensor',txarray,'OperatingFrequency',fc,...
    'PropagationSpeed',c,'WeightsInputPort',true);
rxcollector = phased.Collector('Sensor',rxarray,'OperatingFrequency',fc,...
    'PropagationSpeed',c);
% 
% txradiator = phased.WidebandRadiator('Sensor',txarray,'CarrierFrequency',fc,...
%     'SampleRate',fs_IF,'PropagationSpeed',c,'WeightsInputPort',true);
% rxcollector = phased.WidebandCollector('Sensor',rxarray,'CarrierFrequency',fc,...
%     'SampleRate',fs_IF,'PropagationSpeed',c);

%%
radar_s = phased.Platform;                % Stationary Radar

%% simulation of 1 target at different positions, speed, directions.
target_dist = [1.5];              % Distance between sensor and 3 targets (meters)
target_speed = [-0.7];            % m/s, only simulate x axis direction motions.
target_az = [-10];                  % azimuth angles, degrees
target_zpos = [-0.7];          % z axis positons
target_rcs = [0.04];            % RCS of targets
target_pos = [target_dist.*cosd(target_az);target_dist.*sind(target_az);target_zpos];   % initial positons

targets = phased.RadarTarget('MeanRCS',target_rcs,'PropagationSpeed',c,'OperatingFrequency',fc);
targetmotion = phased.Platform('InitialPosition',target_pos,'Velocity',[target_speed;0;0]);

%% simulation of 3 targets at different positions, speed, directions.
% target_dist = [1.5 0.8 0.8];              % Distance between sensor and 3 targets (meters)
% target_speed = [-0.7 0.5 0.5];            % m/s, only simulate x axis direction motions.
% target_az = [-10 10 30];                  % azimuth angles, degrees
% target_zpos = [-0.01 -0.85 0.55];          % z axis positons
% target_rcs = [0.02 0.03 0.04];            % RCS of targets
% target_pos = [target_dist.*cosd(target_az);target_dist.*sind(target_az);target_zpos];   % initial positons
% 
% targets = phased.RadarTarget('MeanRCS',target_rcs,'PropagationSpeed',c,'OperatingFrequency',fc);
% targetmotion = phased.Platform('InitialPosition',target_pos,'Velocity',[target_speed;0 0 0;0 0 0]);

%%
% simulation of free space propagtion
channel = phased.FreeSpace('PropagationSpeed',c,...
    'OperatingFrequency',fc,'SampleRate',fs,'TwoWayPropagation',true);


%%
% Generate Time Domain Waveforms of Chirps
% xr is the data received at rx array

rng(2019);
Nsweep = 256;               % Number of Chirps (IF signal) of this simulation
% Dn = fix(fs/fr_max/2);      % Decimation factor
% fs_d = fix(fs/Dn*tm)/tm;    % IF signal sample rate

fs_d = 256000
Dn = 11719

chirp_len = fix(fs_d*waveform.SweepTime);
xr = complex(zeros(chirp_len,Nry*Nrz,Nsweep));
w0 = [0;1];  % weights to enable/disable radiating elements

disp('The simulation will take some time. Please wait...')
for m = 1:Nsweep
    if mod(m,1)==0
        disp([num2str(m),'/',num2str(Nsweep)])
    end
    
    % Update radar and target positions
    [radar_pos,radar_vel] = radar_s(waveform.SweepTime);
    [tgt_pos,tgt_vel] = targetmotion(waveform.SweepTime);
    [~,tgt_ang] = rangeangle(tgt_pos,radar_pos);

    % Transmit FMCW waveform
    sig = waveform();
    txsig = transmitter(sig);

    % Toggle transmit element
    w0 = 1-w0;
    txsig = txradiator(txsig,tgt_ang,w0);

    % Propagate the signal and reflect off the target
    txsig = channel(txsig,radar_pos,tgt_pos,radar_vel,tgt_vel);
    txsig = targets(txsig);

    % Dechirp the received radar return
    rxsig = rxcollector(txsig,tgt_ang);
    rxsig = receiver(rxsig);
    dechirpsig = dechirp(rxsig,sig);

    % Decimate the return to reduce computation requirements
    for n = size(xr,2):-1:1
        xr(:,n,m) = decimate(dechirpsig(:,n),Dn,'FIR');
    end
end

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         Part II: Signal Processing                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
% Virtual Array Chirp Data
Nvsweep = Nsweep/2;
xr1 = xr(:,:,1:2:end);
xr2 = xr(:,:,2:2:end);

xrv = cat(2,xr1,xr2);   % Use 2T4R

% xrv = xr1;              % Use 1T4R (need to change varray correspondingly)
% varray = phased.URA([Nrz Nry],[drz dry]);
%%
% FFT points
nfft_r = 2^nextpow2(size(xrv,1));
nfft_d = 2^nextpow2(size(xrv,3));

% RDM Algorithm
rngdop = phased.RangeDopplerResponse('PropagationSpeed',c,...
    'DopplerOutput','Speed','OperatingFrequency',fc,'SampleRate',fs_d,...
    'RangeMethod','FFT','PRFSource','Property',...
    'RangeWindow','Hann','PRF',1/(Nty*waveform.SweepTime),...
    'SweepSlope',waveform.SweepBandwidth/waveform.SweepTime,...
    'RangeFFTLengthSource','Property','RangeFFTLength',nfft_r,...
    'DopplerFFTLengthSource','Property','DopplerFFTLength',nfft_d,...
    'DopplerWindow','Hann');

% RD Map
[resp,r,sp] = rngdop(xrv);

figure;plotResponse(rngdop,squeeze(xrv(:,1,:)));axis([-v_max v_max 0 range_max-0.05])

%%
respmap = squeeze(mag2db(abs(resp(:,1,:))));
threshold = -18; %dB
respmap = respmap-max(respmap(:));                     % Normalize map
peakmat = phased.internal.findpeaks2D(respmap,0,threshold);   
[rangeidx,dopidx] = ind2sub(size(respmap),find(peakmat));

ridx = rangeidx;
rng_est = r(ridx);

vidx = dopidx;
vel_est = sp(vidx);

%%

xv = squeeze(sum(resp(ridx,:,:),1))';

musicazelspectrum = phased.MUSICEstimator2D('SensorArray',varray,'PropagationSpeed',c,...
    'OperatingFrequency',fc,'DOAOutputPort',true,...
    'AzimuthScanAngles',-45:45,'ElevationScanAngles',-45:45);
[~,ang] = musicazelspectrum(xv);
figure;plotSpectrum(musicazelspectrum);

disp('Target DOA:')
angle = ang(:,1)   % Single Target DOA Estimation

%%
% for k=1:size(ridx,1)
%     xv = squeeze(resp(ridx(k),:,:))';
% 
%     musicazelspectrum = phased.MUSICEstimator2D('SensorArray',varray,'PropagationSpeed',c,...
%         'OperatingFrequency',fc,'DOAOutputPort',true,...
%         'AzimuthScanAngles',-65:65,'ElevationScanAngles',-65:65);
%     [~,ang] = musicazelspectrum(xv)
%     figure;plotSpectrum(musicazelspectrum);
% end

