%%
[s,port] = serial_port_init;
%%
rm_obj = read_multi_prep([hex2dec('61') 1 hex2dec('63')]);
%%
N = 100;
tbl = zeros(N,3);
for i=1:N
  [values,ack] = read_multi(s,rm_obj);
  if ack == 1
    tbl(i,:) = values;
  end
end
%
%V1 = read_subbus(s,hex2dec('61'));
%%
serial_port_clear;
