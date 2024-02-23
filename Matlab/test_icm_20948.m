%%
[s,port] = serial_port_init('',115200,1);
addr.cmd = 48;
%%
identify_uDACS16(s);
[icm_mode,icm_fs] = report_icm_mode(s);
%%
test_mode_switch(s,0,0);
%%
test_mode_switches(s);
%%
for fs = 0:3
  test_slow_mode(s,fs);
end

%%
% serial_port_clear;
clear s
%%
% Fast mode diagnostics:
test_fast_mode(s,0);
%%
clear res;
mr_stat =  read_multi_prep(100, 103);
for i=1:-1:1
  [diag,ack] = read_subbus(s,103); % try to clear diag before test
  thisres = test_fast_mode2(s, 0, 10000);
  res(i) = thisres;
  res(i).Nread = sum(res(i).stats(1:end-1,2));
  while true
    [values,ack] = read_multi(s,mr_stat);
    if values(1) == 0 && values(2) == 0; break; end
    fprintf(1,'+');
  end
end
fprintf(1,'\nMean Nread is %.1f +/- %.1f\n', mean([res.Nread]), std([res.Nread]));
%%
function res = test_fast_mode2(s, fs, N)
  % res = test_fast_mode2(s, fs. N);
  % res.stats cols are:
  %   mode (0x64)
  %   internal diagnostic (0x67)
  %   FIFO count (0x65)
  fprintf(1, '.');
  res.a = zeros(N,3);
  res.stats = zeros(N,4);
  Nread = 0;
  res.Nstats = 0;
  % ICM FIFO N Bytes, uDACS FIFO N Words, FIFO Contents
  % rm_obj = read_multi_prep(100, 103, [101 495 102 0]);
  rm_obj = read_multi_prep(100, 103, 101);
  write_subbus_v(s, 48, 50+fs);
  write_subbus_v(s, 48, 40+2);
  tic;

  leftover = [];
  n_leftover = 0;
  while Nread < N && res.Nstats < 10000 && toc < 10
    [values,ack] = read_multi(s, rm_obj);
    if ack ~= 1; break; end
    nwords = length(values) - 3 + n_leftover;
    remainder = mod(nwords,3);
    nrows = (nwords-remainder)/3;
    stats = [values(1:3)' nrows];
    if any(stats ~= [2 0 0 0])
      res.Nstats = res.Nstats+1;
      res.stats(res.Nstats,:) = stats;
    end
    if values(1) ~= 2; break; end
    % try
    %   res.a(Nread+(1:nrows),:) = ...
    %     reshape([leftover; values(4:end-remainder)],3,[])';
    % catch
    %   fprintf(1,'Error on reshape: nwords:%d remainder:%d nrows:%d\n', nwords, remainder, nrows);
    %   return;
    % end
    Nread = Nread + nrows;
    res.Nread = Nread;
    if remainder
      try
        leftover = values(end-remainder+(1:remainder));
      catch
        fprintf(1,'leftover\n');
      end
    else
      leftover = [];
    end
    n_leftover = remainder;
  end
  res.stats = res.stats(1:res.Nstats,:);
  res.dur = toc;
  res.fin_ack = ack;
  res.fin_fin_ack = write_subbus_v(s, 48, 40);
end

function ack = write_subbus_v(s, addr, value)
  [ack,line] = write_subbus(s, addr, value);
  if ack == -2
    fprintf(1,'ack -2 on write_subbus: "%s"', line);
  end
end

function test_fast_mode(s, fs)
  fprintf(1, '\ntest_fast_mode(%d)\n', fs);
  n_FIFO0 = read_subbus(s, 103); %
  n_FIFO = read_subbus(s, 101); % 0x65
  fprintf(1, '  n_FIFO: %d/%d\n', n_FIFO0, n_FIFO);
  write_subbus(s, 48, 50+fs);
  dur = 0;
  tic;
  write_subbus(s, 48, 40+2);
  for i=1:20
    n_FIFO0 = read_subbus(s, 103); %
    n_FIFO = read_subbus(s, 101); % 0x65
    mode = bitand(read_subbus(s, 100),7);
    if mode == 0
      dur = toc;
      break;
    end
    fprintf(1, '  %2d: n_FIFO: %d/%d mode: %d\n', i, n_FIFO0, n_FIFO, mode);
  end
  if dur == 0
    dur = toc;
  end
  write_subbus(s, 48, 40);
  n_FIFO0 = read_subbus(s, 103); %
  n_FIFO = read_subbus(s, 101); % 0x65
  fprintf(1, '  n_FIFO: %d/%d  dT = %f\n', n_FIFO0, n_FIFO, dur);
end

function [icm_mode,icm_fs] = report_icm_mode(s)
[icm_mode_fs,ack] = read_subbus(s, 100); % 0x64
if ~ack
  fprintf(1,'No ACK for ICM Mode: Vibration Sensor apparently not supported\n');
  icm_mode = 0;
  icm_fs = 0;
else
  icm_mode = bitand(icm_mode_fs,7);
  icm_fs = bitand(icm_mode_fs,24)/8; % (icm_mode_fs%0x18)>>3
  switch icm_mode
    case 0, mode_text = 'Idle';
    case 1, mode_text = 'Slow';
    case 2, mode_text = 'Fast';
      OTHERWISE, mode_text = 'Unknown';
  end
  fprintf(1,'  ICM Mode %d: %s\n', icm_mode, mode_text);
  full_scale = 2*2^icm_fs;
  fprintf(1,'  ICM Full Scale is %d g\n', full_scale);
  % pause(0.5);
  % [accel_cfg_rb,ack] = read_subbus(s, 103); % 0x67
  % fprintf(1,'  ACCEL_CONFIG: %02X\n', accel_cfg_rb);
end
end

function test_slow_mode(s, fs)
  fprintf(1,'\nTesting Slow Mode operation with fs %d:\n', fs);
  full_scale = 2*2^fs;
  fprintf(1,'  Setting full scale to %d g\n', full_scale);
  ack = write_subbus(s, 48, 50+fs);
  [icm_mode,icm_fs] = report_icm_mode(s);
  fprintf(1,'  Selecting Slow Mode\n');
  ack = write_subbus(s, 48, 41);
  [icm_mode,icm_fs] = report_icm_mode(s);
  if icm_mode == 1
    fprintf(1,'  Acquiring data slowly:\n');
    rm_obj = read_multi_prep([hex2dec('61') 1 hex2dec('63')]);
    N = 100;
    tbl = zeros(N,3);
    for i=1:N
      [values,ack] = read_multi(s,rm_obj);
      V = values>32767;
      if any(V)
        values(V) = values(V) - 65536;
      end
      if ack == 1
        tbl(i,:) = values;
      end
    end
    tbl = full_scale*tbl/32768;
    ax = nsubplots(tbl);
    title(ax(1), sprintf('Full Scale = %d g', full_scale));
  end
  fprintf(1,'\nReturning to No Mode:\n');
  ack = write_subbus(s, 48, 50);
  ack = write_subbus(s, 48, 40);
  [icm_mode,icm_fs] = report_icm_mode(s);
end

function test_mode_switch(s, mode, fs)
  % test_mode_switch(s);
  % s is a serialport object
  ack = write_subbus(s, 48, 50 + fs);
  ack = write_subbus(s, 48, 40 + mode);
  % pause(0.5);
  icm_mode_fs = read_subbus(s, 100); % 0x64
  icm_accel_cfg = read_subbus(s, 103); % 0x67
  fprintf(1, '  %d,%d: mode: 0x%02X  ACCEL_CONFIG: 0x%02X\n', ...
      mode, fs, icm_mode_fs, icm_accel_cfg);
  pause(1);
end

function test_mode_switches(s)
  fprintf(1, '\ntest_mode_switches():\n');
  for mode = 0:1 % skipping fast mode for the moment
    for fs = 0:3
      test_mode_switch(s,mode,fs);
    end
  end
  test_mode_switch(s,0,0);
end
