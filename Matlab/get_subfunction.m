function [ subfunction, desc] = get_subfunction(s)
% [subfunction, desc] = get_subfunction(s)
fprintf(s,"V\n");
tline = fgetl(s);
[A,n,~,nextindex] = sscanf(tline,'V%x:%x:');
if n ~= 2
  error('Error getting subfunction code');
end
subfunction = A(1);
if nargout > 1
  desc = tline(nextindex:end);
end
fgetl(s); % needed to clear input buffer