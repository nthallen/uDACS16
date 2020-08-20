%%
% cd C:\huarp\ElecCore\uDACS\code\uDACS16\uDACS16\Matlab
cd C:\Users\nort\Documents\Documents\Exp\Boards\uDACS16\uDACS16\Matlab
%%
serial_port_clear();
%%
[s,port] = serial_port_init();
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
  vref = 4.096;
  vadc = vref * adc / (2^16);
  for i=1:length(adc)
    fprintf(1,'%8X %8d %10f V\n', adc(i), adc(i), vadc(i));
  end
  %%
  pause(1);
end

%%
% Read from Timer section
rm_obj = read_multi_prep([64,1,68]); % 0x40
%
T0 = -1;
for ielp = 1:100
  [vals,~] = read_multi(s,rm_obj);
  T1 = vals(1) + vals(2)*65536;
  if T0 >= 0
    dT = T1-T0;
    fprintf(1, 'Elapsed/Loop/Max/State: %.5f %.3f %.3f %.0f\n', dT*1e-5, vals(3)*1e-2, vals(4)*1e-2, vals(5));
  end
  T0 = T1;
  pause(.1);
end
%%
N = 1000;
curlooptime = zeros(N,1);
for i=1:N
  val = read_subbus(s,66);
  curlooptime(i) = val*1e-2;
  pause(.1);
end
%
figure; plot(curlooptime,'.');
ylabel('msec');

%%
% Honeywell Pressure Sensor Readings:
Honeybase = hex2dec('50');
%%
% NWpsdesc = read_subbus(s,Honeybase+9);
rm_obj = read_multi_prep([Honeybase+9,50,Honeybase+10,0]);
[vals,~] = read_multi(s,rm_obj);
if isempty(vals) || vals(1) ~= 46 || vals(1)+1 ~= length(vals)
  error('vals length was %d, expected 46', length(vals));
end
%
PSdef = struct( ...
  'PartNumber', { words2txt(vals(2:10)) words2txt(vals(23+(2:10))) }, ...
  'SerialNumber', { words2txt(vals(11:16)) words2txt(vals(23+(11:16))) }, ...
  'PressureUnit', { words2txt(vals(17:19)) words2txt(vals(23+(17:19))) }, ...
  'PressureRef', { words2txt(vals(20)) words2txt(vals(23+20)) }, ...
  'PressureRange', { typecast(uint32(vals(21)+65536*vals(22)),'single'), ...
                     typecast(uint32(vals(23+21)+65536*vals(23+22)),'single') }, ...
  'PressureMin', { typecast(uint32(vals(23)+65536*vals(24)),'single'), ...
                     typecast(uint32(vals(23+23)+65536*vals(23+24)),'single') });
%
for i=1:2
  fprintf(1, 'Sensor %d:\n', i);
  fprintf(1, '  Part: %s\n', PSdef(i).PartNumber);
  fprintf(1, '  S/N:  %s\n', PSdef(i).SerialNumber);
  fprintf(1, '  Unit: %s %s\n', PSdef(i).PressureUnit, PSdef(i).PressureRef);
  fprintf(1, '  Min/Range: %f/%f\n', PSdef(i).PressureMin, PSdef(i).PressureRange);
end
%%
conv2Torr = struct('PSI', 51.7149, 'inH2O', 1.86645);
rm_obj = read_multi_prep([Honeybase,1,Honeybase+8]); % 0x40
%%
[vals,ack] = read_multi(s, rm_obj);
status = vals(1);
fprintf(1,'Status: 0x%X\n', status);
PSread = struct( ...
  'T', { typecast(uint32(vals(2)+65536*vals(3)),'single'), ...
         typecast(uint32(vals(6)+65536*vals(7)),'single') }, ...
  'P', { typecast(uint32(vals(4)+65536*vals(5)),'single'), ...
         typecast(uint32(vals(8)+65536*vals(9)),'single') });
for i=1:2
  fprintf(1,'P%d: %7.3f %5s %s   T%d: %7.3f\n', i, ...
    PSread(i).P, PSdef(i).PressureUnit, PSdef(i).PressureRef, ...
    i, PSread(i).T);
  fprintf(1,'P%d: %7.3f %5s %s   T%d: %7.3f\n', i, ...
    PSread(i).P*conv2Torr.(PSdef(i).PressureUnit), 'Torr', PSdef(i).PressureRef, ...
    i, PSread(i).T);
end
% fprintf(1,'P2: %7.3f %5s %s   T2: %7.3f\n', PS_P2, PSdef(2).PressureUnit, PSdef(2).PressureRef, PS_T2);

