function [ value, ack_out ] = read_subbus(s, addr)
  % [value, ack] = read_subbus(s, addr);
  % s: serialport object
  % addr: subbus address
  % value: data
  % ack: 1 on successful acknowledge, 0 on nack, -1 on timeout,
  %      -2 on other errors
  writeline(s, sprintf('R%X', addr));
  tline = char(readline(s));
  if isempty(tline)
    ack = -1;
  elseif tline(1) == 'R'
    ack = 1;
  elseif tline(1) == 'r'
    ack = 0;
  else
    ack = -2;
  end
  if ack >= 0
    value = hex2dec(tline(2:end));
  else
    value = 0;
  end
  if nargout > 1
    ack_out = ack;
  elseif ack ~= 1
    error(sprintf('ack=%d on read_subbus(0x%X)', ack, addr));
  end