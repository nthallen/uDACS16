function identify_uDACS16(s)
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
elseif BoardID == 2
  BdCfg = 'uDACS B';
elseif ( BoardID == 3 || BoardID == 4 )
  BdCfg = 'SCoPEx Engine Assembly';
else
  BdCfg = 'Test';
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
