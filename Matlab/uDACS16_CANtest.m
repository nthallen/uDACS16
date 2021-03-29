%%********************************************************************
% uDACS16 CAN Test                                8:12 AM 3/15/2021
% 
% CAN-based test of basic uDACS16 functions. 
% May be integrated into main uDACS16_test.m 
% 
% 

% cd C:\huarp\ElecCore\uDACS\code\uDACS16\uDACS16\Matlab
% cd C:\Users\nort\Documents\Documents\Exp\Boards\uDACS16\uDACS16\Matlab

%%
% Estabish CAN connection 
%
% Rev set to A for uDACS16 until Rev B
Rev = 'A';

sbsl = subbusd_slcan;           % create the subbus_serial_CAN object
sbsl.close;                     % close its serial port
sbsl.open;                      % open its serial port

% Enter CAN_ID for board under test, and check it is a uDACS16
%
CAN_ID = 1; % Set as needed. Default is Test: CAN_ID = 1
while(1)
  try
    fprintf(1, 'Attempting CAN ID %d\n', CAN_ID);
    Board_ID = sbsl.SBCAN_read_addrs(CAN_ID,2); % Board_ID
    if Board_ID <= 4
	  Build = sbsl.SBCAN_read_addrs(CAN_ID,3); % Build
	  SerialNo = sbsl.SBCAN_read_addrs(CAN_ID,4); % board Serial Number
	  InstID = sbsl.SBCAN_read_addrs(CAN_ID,5); % board InstID
	  
	  if Board_ID == 1
        BdCfg = 'uDACS A';
      elseif Board_ID == 2
        BdCfg = 'uDACS B';
      elseif ( Board_ID == 3 || Board_ID == 4 )
        BdCfg = 'SCoPEx Engine Assembly';
      else
        BdCfg = 'Test';
      end
      
      fprintf(1, 'Good Connection with CAN ID %d! uDACS16 Board ID = %d\n', CAN_ID, Board_ID);
      fprintf(1, 'Attached to uDACS16 S/N %d Build # %d\n', SerialNo, Build);
      fprintf(1, 'Board is Rev %s configured as "%s"\n', Rev, BdCfg);
%      fprintf(1, 'The description is "%s"\n', desc);  % V read not implemented over CAN yet
  	  break
	else
      fprintf(1, 'CAN ID %d is not uDACS16. Returned Board ID = %d\n', CAN_ID, Board_ID);
    end
  catch MExc
    disp(MExc.message); % Uncomment to show 
	warning('No response from CAN ID %d\n', CAN_ID); 
  end
  prompt = 'Enter new CAN ID: \n';
  CAN_ID = input(prompt);  % Type in new CAN ID 
end

% ???
% rm_obj = read_multi_prep([8,40,9,0]);
% [vals,~] = read_multi(s,rm_obj);
% even = mod(vals(2:end),256);
% odd = floor(vals(2:end)/256);
% il = [even odd]';
% il = il(:)';
% nc = find(il == 0,1);
% il = il(1:(nc-1));
% desc = char(il);
% fprintf(1,'Description from FIFO is: %s\n', desc);
%fprintf(1, 'Now figure out how to interpret the result\n');

%%
% Read the ADS1115 ADC Section

ANA_ADDR = 32;  % 0x20 ; uDACS16 base addr for analog inputs

for iadc=1:10
  vals = sbsl.SBCAN_read_inc(CAN_ID, 10, ANA_ADDR);  % 1 I2C Status +8 registers +1 Poll Count if wanted
  fprintf(1,'---------\n');
  fprintf(1,'%04X %d\n', vals(1),vals(end));
  adc = vals(2:end-1); % keep only ADC values
  sadc = adc - (adc>2^15)*2^16; % signed results
  vref = 4.096;
  vadc = vref * sadc / (2^15); % Convert to Volts
  Rpu = 75e3;
  Rth = Rpu * vadc ./ (vref-vadc); % Calculate Thermistor resistance
  for i=1:length(adc)
    fprintf(1,'%8X %8d %10f V %10g Ohm\n', adc(i), adc(i), vadc(i), Rth(i));
  end
  %%
  pause(1);
end

%%
% Read from Timer section
%
TC_ADDR = 64; % 0x40. RTC_BASE_ADDR 
TC_F = 1e5;
% TC_F = 8e6; % for testing: ~9 Minutes to rollover
T0 = -1;
iT0 = -1;
N = 100;
Tsec = 20;
Psec = Tsec/N;
for ielp = 0:N
  vals = sbsl.SBCAN_read_inc(CAN_ID, 4, TC_ADDR);  % Read 0x40 - 0x43
  T1 = vals(1) + vals(2)*65536; % 32bit Total elapsed time in msec
  if T0 >= 0
    dT = (T1-T0)/TC_F;
    TT = (T1-iT0)/TC_F;
    fprintf(1, 'Elapsed/Loop/Max/State: TT:%.3f s dT:%.5f s LT:%.3f ms LMT:%.3f ms\n', TT, dT, vals(3)/TC_F*1e3, vals(4)/TC_F*1e3);
  else
    iT0 = T1;
    tic;
    fprintf(1, 'Starting value %.6f sec\n', iT0/TC_F);
  end
  T0 = T1;
  if ielp < N; pause(Psec); end
end
telapsed = toc;
iT1 = T1;
ips = ((iT1-iT0)/TC_F)/telapsed;
fprintf(1, 'Observed %.5f seconds/seconds\n', ips);

%%
N = 100; % Set to 1000 for thorough test
curlooptime = zeros(N,1);
for i=1:N
  val = sbsl.SBCAN_read_inc(CAN_ID, 1, TC_ADDR+2);  % Read 0x42
  curlooptime(i) = (val/TC_F)*1e3; % msec
  pause(.1);
end
%
figure; plot(curlooptime,'.');
ylabel('msec');

%%
% MS5607 Barometer :
%
MS_ADDR = hex2dec('10'); %% 0x10 : MS5607 SPI Base Address

% Read Coefficients
vals = sbsl.SBCAN_read_inc(CAN_ID, 6, MS_ADDR+4);  % [0x14 - 0x19]
%
if isempty(vals) || length(vals) ~= 6 
  error('Expected 6 Coefficients: Read %d\n', length(vals));
else
  fprintf(1, '\nMS5607 Coefficients:\n');
  for i=1:6
    fprintf(1, '  C%d: %x\n', i, vals(i));
  end
end

%%
% Read/Calculate Compensated Pressure and Temperature
%
fprintf(1, '\nMS5607 Pressure and Temperature:\n');
for i=0:9
  vals = sbsl.SBCAN_read_inc(CAN_ID, 4, MS_ADDR);  % [0x10 - 0x13]
  
  PTread = struct( ...
    'T', { typecast(uint32(vals(3)+65536*vals(4)),'single') }, ...
    'P', { typecast(uint32(vals(1)+65536*vals(2)),'single') });
  
  fprintf(1,'P%d: %7.3f mBar ( %7.3f Torr )  T%d: %7.3f degC\n', i, ...
    PTread.P, (PTread.P * 0.750062), i, PTread.T);
 
  pause(1);
end

pause;
%-------  STOP TEST HERE  -------%

%%
% D/A Channel 0
write_subbus(s, 32+10, 0xFFFF);
%%
% D/A Channel 1
write_subbus(s, 32+11, 0xFFFF);
%%
% D/A Channel 2
write_subbus(s, 32+12, 0xFFFF);
%%
% D/A Channel 3
write_subbus(s, 32+13, 0xFFFF);

%%
% Command Testing
cmd_pins = { 'J7', 'J8', 'J34', 'J35' };
for npin = 1:length(cmd_pins)
  pin = cmd_pins{npin};
  cmdnum = (npin-1)*2+1;
  fprintf(1, 'Hit ENTER to command %s ON (cmd %d)\n', pin, cmdnum);
  pause;
  write_subbus(s, 48, cmdnum);
  status = read_subbus(s, 48);
  fprintf(1, '  Status is %02X\n', status);
  cmdnum = (npin-1)*2;
  fprintf(1, 'Hit ENTER to command %s OFF (cmd %d)\n', pin, cmdnum);
  pause;
  write_subbus(s, 48, cmdnum);
  status = read_subbus(s, 48);
  fprintf(1, '  Status is %02X\n', status);
end

%%
% Subbus fail test
tick_subbus(s);
write_subbus(s, 6, 0); % Clear fail
failval = read_subbus(s,6);
if failval
  fprintf(1,'Fail is non-zero after tick and explicit clear\n');
end
%%
fprintf(1, 'Testing default timeout of 120 seconds:\n');
write_subbus(s, 6, 0); % Clear fail
tick_subbus(s, 1);
tic;
failval = read_subbus(s,6);
faildur = toc;
while failval == 0 && faildur < 125
  pause(1);
  failval = read_subbus(s,6);
  faildur = toc;
end
if failval
  if faildur < 119
    fprintf(1,'ERROR: Fail occurred early: %f secs\n', faildur);
  else
    fprintf(1, 'Fail occurred after %f secs, within specs\n', faildur);
  end
else
  fprintf(1, 'ERROR: Timed out after %f secs without fail\n', faildur);
end
%%
fprintf(1, 'Testing reduced timeout of 20 seconds:\n');
tick_subbus(s,1);
write_subbus(s, 6, 100*256+0); % Clear fail
tic;
failval = bitand(read_subbus(s,6),255);
faildur = toc;
while failval == 0 && faildur < 25
  pause(1);
  failval = bitand(read_subbus(s,6),255);
  faildur = toc;
end
if failval
  if faildur < 19
    fprintf(1,'ERROR: Fail occurred early: %f secs\n', faildur);
  else
    fprintf(1, 'Fail occurred after %f secs, within specs\n', faildur);
  end
else
  fprintf(1, 'ERROR: Timed out after %f secs without fail\n', faildur);
end
