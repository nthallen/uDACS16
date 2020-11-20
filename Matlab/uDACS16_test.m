%%
cd C:\huarp\ElecCore\uDACS\code\uDACS16\uDACS16\Matlab
%%
serial_port_clear();
%%
[s,port] = serial_port_init('COM4');
set(s,'BaudRate',57600);
% set(s,'BaudRate',115200);
%%
% First check that the board is a uDACS
[subfunc,desc] = get_subfunction(s);
if subfunc ~= 15
  error('Expected BdID 15. Reported %d', subfunc);
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

fprintf(1, 'Attached to uDACS S/N %d Build # %d\n', SerialNo, Build);
fprintf(1, 'Board is Rev %s configured as "%s"\n', Rev, BdCfg);
fprintf(1, 'The description is "%s"\n', desc);


rm_obj = read_multi_prep([8,40,9,0]);
[vals,ack] = read_multi(s,rm_obj);
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
rm_obj = read_multi_prep([32,1,41]); % [0x20,1,0x29]
%
% while true
for iadc=1:10
  %%
  [vals,ack] = read_multi(s,rm_obj);
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
% MS5607 Barometer :
ms_base = hex2dec('10'); %% 0x10

% Read Coefficients
rm_obj = read_multi_prep([ms_base+4,1,ms_base+9]);  % [0x14 - 0x19]
[vals,~] = read_multi(s,rm_obj);
%%
% if isempty(vals) || length(vals) ~= 6 
%   error('vals length was %d, expected 6', length(vals));
% end
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


