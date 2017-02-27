% initialize FPGA
fugihandle = fugiboard('Open', 'Pendulum1');
fugihandle.WatchdogTimeout = 0.5;
fugiboard('SetParams', fugihandle);
fugiboard('Write', fugihandle, 0, 0, 0, 0);  % dummy write to sync interface board
fugiboard('Write', fugihandle, 5, 1, 0, 0);  % reset position, activate relay
data = fugiboard('Read', fugihandle);        % get version info from FPGA
model = bitshift(data(1), -5); 
version = bitand(data(1), 31);
disp(sprintf('FPGA setup %d, version %d', model, version)); % for the B&P it should say 1 1, or else there is a problem
fugiboard('Write', fugihandle, 0, 1, 0, 0);  % end reset
pause(0.1);                         % give relay some time to respond
