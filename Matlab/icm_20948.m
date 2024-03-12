function icm_20948
  [s,~] = serial_port_init('',115200,1);
  identify_uDACS16(s);

  Samples_per_report = 512;
  fs = 0;
  new_fs = fs;
  fig = figure;
  opts.start = 1;
  opts.fs = fs;
  fig.UserData = opts;
  top = uimenu(fig,'Text','IMU');
  uimenu(top,'Text','Stop', ...
          'Callback', @(s,e)set_opt(fig,0,0), ...
          'Interruptible', 'off');
  uimenu(top,'Text','2g', ...
          'Callback', @(s,e)set_opt(fig,1,0), ...
          'Interruptible', 'off');
  uimenu(top,'Text','4g', ...
          'Callback', @(s,e)set_opt(fig,1,1), ...
          'Interruptible', 'off');
  uimenu(top,'Text','8g', ...
          'Callback', @(s,e)set_opt(fig,1,2), ...
          'Interruptible', 'off');
  uimenu(top,'Text','16g', ...
          'Callback', @(s,e)set_opt(fig,1,3), ...
          'Interruptible', 'off');
  ax = axes(fig);

  L = Samples_per_report; % length for FFT
  res.a = zeros(L,3);
  Nread = 0;
  
  % ICM FIFO N Bytes, uDACS FIFO N Words, FIFO Contents
  rm_obj = read_multi_prep(100, 103, [101 495 102 0]);
  % rm_obj = read_multi_prep(100, 103, 101);
  write_subbus_v(s, 48, 50+fs);
  write_subbus_v(s, 48, 40+2);
  Fs = 515; % Should calibrate this...
  N2 = floor(L/2);
  x = 1:N2;
  f = x*Fs/L;
  leftover = [];
  n_leftover = 0;
  max_amp = 0;
  Nmem = 20;
  mem = zeros(N2,Nmem);
  imem = 1;
  report_icm_mode(s);
  
  tic;
  cur_time = toc;
  while fig.UserData.start
    if fig.UserData.fs ~= fs
      new_fs = round(fig.UserData.fs);
      if new_fs < 0 || new_fs > 3
        new_fs = fs;
      end
    end
    [values,ack] = read_multi(s, rm_obj);
    if ack ~= 1; break; end
    nwords = length(values) - 3 + n_leftover;
    remainder = mod(nwords,3);
    nrows = (nwords-remainder)/3;
    if bitand(values(1),3) ~= 2
      fprintf(1,'Mode value was %d\n', values(1));
      break;
    end
    vals = values(4:end);
    Vneg = vals >= 32768;
    if any(Vneg)
      vals(Vneg) = vals(Vneg)-65536;
    end
    if Nread + nrows > L
      remainder = remainder + (Nread+nrows-L)*3;
      nrows = L-Nread;
    end
    res.a(Nread+(1:nrows),:) = ...
      reshape([leftover; vals(1:end-remainder)],3,[])';
    Nread = Nread + nrows;
    if Nread == L
      res.a = res.a * 2^(fs+1) / 32768;
      A = res.a - ones(length(res.a),1)*mean(res.a);
      YA = fft(A)/L;
      VA = vecnorm(YA,2,2);
      maxVA = max(VA);
      if maxVA > max_amp
        max_amp = maxVA;
      end
      mem(:,imem) = VA(x);
      max_mem = max(mem,[],2);
      imem = mod(imem,Nmem)+1;
      plot(ax,f,VA(x,:),f,max_mem,'.');
      set(ax,'ylim',[0 max_amp],'xgrid','on','ygrid','on');
      title(ax, sprintf('T = %.1f',cur_time));
      drawnow;
      Nread = 0;
    end
    if remainder
      leftover = vals(end-remainder+(1:remainder));
    else
      leftover = [];
    end
    n_leftover = remainder;
    cur_time = toc;
    if new_fs ~= fs
      write_subbus_v(s, 48, 50+new_fs);
      fprintf(1,'Set fs to %d\n', new_fs);
      fs = new_fs;
    end
  end
  report_icm_mode(s);
  write_subbus_v(s, 48, 40);
  report_icm_mode(s);
  delete(fig);
end

function set_opt(fig, start, fs)
  fig.UserData.start = start;
  fig.UserData.fs = fs;
end

function [icm_mode,icm_fs] = report_icm_mode(s, quiet)
  if nargin < 2
    quiet = 0;
  end
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
      otherwise, mode_text = 'Unknown';
    end
    if ~quiet
      fprintf(1,'  ICM Mode %d: %s\n', icm_mode, mode_text);
      full_scale = 2*2^icm_fs;
      fprintf(1,'  ICM Full Scale is %d g\n', full_scale);
    end
  end
end

function ack = write_subbus_v(s, addr, value)
  [ack,line] = write_subbus(s, addr, value);
  if ack == -2
    fprintf(1,'ack -2 on write_subbus: "%s"', line);
  end
end
