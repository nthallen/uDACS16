function [values,ack] = read_multi(s, rm_obj)
  % [values, ack] = read_multi(s, rm_obj);
  % s is a serialport object
  % rm_obj is the output of read_multi_prep();
  % values is an array of all the returned values
  % ack is 1 if all responses are acknowledged, 0 if any are NACK,
  % -1 on timeout and -2 on any other errors.
  writeline(s, sprintf('%s', rm_obj.cmd));
  tline = readline(s);
  tline = char(tline);
  if isempty(tline)
    ack = -1;
    values = [];
  else
    [strings,matches] = strsplit(tline,'[MmU]','DelimiterType','RegularExpression');
    values = hex2dec(strings(2:end));
    if all(contains(matches,'M'))
      ack = 1;
    elseif any(contains(matches,'U'))
      ack = -2;
    else
      ack = 0;
    end
%     if length(values) > 2 && length(values) < rm_obj.count
%       warning('Short read');
%     end
  end
