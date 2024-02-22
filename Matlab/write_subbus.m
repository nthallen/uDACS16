function [ack_out,line_out] = write_subbus(s, addr, value)
  % ack = write_subbus(s, addr, value);
  % s: serialport object
  % addr: subbus address
  % value: data
  % ack: 1 on successful acknowledge, 0 on nack, -1 on timeout,
  %      -2 on other errors
  writeline(s, sprintf('W%X:%X', addr, value));
  tline = char(readline(s));
  if isempty(tline)
    ack = -1;
  elseif tline(1) == 'W'
    ack = 1;
  elseif tline(1) == 'w'
    ack = 0;
  else
    ack = -2;
  end
  if nargout > 0
    ack_out = ack;
    if nargout > 1
      line_out = tline;
    end
  elseif ack ~= 1
    error(sprintf('ack=%d on write_subbus(0x%X)', ack, addr));
  end
