%%
% cd C:\huarp\ElecCore\uDACS\code\uDACS16\uDACS16\Matlab
cd C:\Users\nort\Documents\Documents\Exp\Boards\uDACS16\uDACS16\Matlab
%%
serial_port_clear();
%%
[s] = serial_port_init();
set(s,'BaudRate',57600);
% set(s,'BaudRate',115200);
%%
% First check that the board is a uDACS
[subfunc,desc] = get_subfunction(s);
if subfunc ~= 15
  error('Expected subfunction 15 for uDACS16. Reported %d', subfunc);
end
BoardID = read_subbus(s,2);
Build = read_subbus(s,3);
[SerialNo,SNack] = read_subbus(s,4);
[InstID,InstIDack] = read_subbus(s,5);

% Rev set to A for uDACS16 until Rev B
Rev = 'A';

if BoardID == 1
  BdCfg = 'uDACS A';
else
  BdCfg = 'uDACS B';
end

fprintf(1, 'Attached to uDACS16 S/N %d Build # %d\n', SerialNo, Build);
fprintf(1, 'Board is Rev %s configured as "%s"\n', Rev, BdCfg);
fprintf(1, 'The description is "%s"\n', desc);


rm_obj = read_multi_prep([8,40,9,0]);
[vals,~] = read_multi(s,rm_obj);
even = mod(vals(2:end),256);
odd = floor(vals(2:end)/256);
il = [even odd]';
il = il(:)';
nc = find(il == 0,1);
il = il(1:(nc-1));
desc = char(il);
fprintf(1,'Description from FIFO is: %s\n', desc);
%fprintf(1, 'Now figure out how to interpret the result\n');

%%
% Read the ADS1115 ADC Section
rm_obj = read_multi_prep([32,1,41]); % [0x20,1,0x29]
%
% while true
for iadc=1:10
  %%
  [vals,~] = read_multi(s,rm_obj);
  fprintf(1,'---------\n');
  fprintf(1,'%04X %d\n', vals(1),vals(end));
  adc = vals(2:end-1);
  sadc = adc - (adc>2^15)*2^16;
  vref = 4.096;
  vadc = vref * sadc / (2^15);
  Rpu = 75e3;
  Rth = Rpu * vadc ./ (vref-vadc);
  for i=1:length(adc)
    fprintf(1,'%8X %8d %10f V %10g Ohm\n', adc(i), adc(i), vadc(i), Rth(i));
  end
  %%
  pause(1);
end

%%
% Read from Timer section
rm_obj = read_multi_prep([64,1,67]); % 0x40
%
TC_F = 1e5;
% TC_F = 8e6; % for testing: ~9 Minutes to rollover
T0 = -1;
iT0 = -1;
N = 100;
Tsec = 20;
Psec = Tsec/N;
for ielp = 0:N
  [vals,~] = read_multi(s,rm_obj);
  T1 = vals(1) + vals(2)*65536;
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
N = 1000;
curlooptime = zeros(N,1);
for i=1:N
  val = read_subbus(s,66);
  curlooptime(i) = (val/TC_F)*1e3; % msec
  pause(.1);
end
%
figure; plot(curlooptime,'.');
ylabel('msec');

%%
% MS5607 Barometer :
ms_base = hex2dec('10'); %% 0x10

% Read Coefficients
rm_obj = read_multi_prep([ms_base+4,1,ms_base+9]);  % [0x14 - 0x19]
[vals,~] = read_multi(s,rm_obj);
%
if isempty(vals) || length(vals) ~= 6 
  error('vals length was %d, expected 6', length(vals));
end
%
fprintf(1, '\nMS5607 Coefficients:\n');
for i=1:6
  fprintf(1, '  C%d: %x\n', i, vals(i));
end
%%
rm_obj = read_multi_prep([ms_base,1,ms_base+3]); % 0x10 - 0x13

fprintf(1, '\nMS5607 Pressure and Temperature:\n');
for i=0:9
  [vals,ack] = read_multi(s, rm_obj);
  
PTread = struct( ...
  'T', { typecast(uint32(vals(3)+65536*vals(4)),'single') }, ...
  'P', { typecast(uint32(vals(1)+65536*vals(2)),'single') });
  
  fprintf(1,'P%d: %7.3f mBar ( %7.3f Torr )  T%d: %7.3f degC\n', i, ...
    PTread.P, (PTread.P * 0.750062), i, PTread.T);
 
  pause(1);
end

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
