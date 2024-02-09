function [ subfunction, desc] = get_subfunction(s)
% [subfunction, desc] = get_subfunction(s)
while true
  %fprintf(s,'V');
  writeline(s,'V');
  while true
    tline = readline(s);
    if isempty(tline)
      fprintf(1,'ERROR: V response timeout\n');
      break;
    end
    [A,n,~,nextindex] = sscanf(tline,'V%x:%x:');
    if n == 2
      subfunction = A(1);
      if nargout > 1
        tline = char(tline);
        desc = tline(nextindex:end);
      end
      return;
    else
      fprintf(1, 'ERROR: V response was: "%s"\n', tline);
    end
  end
end
