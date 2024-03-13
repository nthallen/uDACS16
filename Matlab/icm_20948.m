function [ sps, T] = icm_20948(NreportLim)
  NreportMax = 3600;
  if nargin < 1
    NreportLim = NreportMax;
  end
  [s,~] = serial_port_init('',115200,1);
  identify_uDACS16(s);

  Fs = 540; % Based on MATLAB cal 3/12/24
  Samples_per_report = 512;
  L = Samples_per_report; % length for FFT
  Skip_Samples0 = Fs-Samples_per_report;
  Skip_Samples = Skip_Samples0;
  LS = L+Skip_Samples;
  fs = 0;
  new_fs = fs;
  fig = figure;
  ax = [subplot(2,1,1) subplot(2,1,2)];
% ax = axes(fig);
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
  
  Data = zeros(2*L,3);
  Nread = 0;
  % For calculating sample rate
  Nwords_per_sec = 0;
  samples_per_sec = zeros(NreportMax,1);
  Nsecs = 0;
  Tsample0 = -1;
  ReportT = zeros(NreportMax,1);
  Nreport = 0;
  ITerr = 0;
  gITerr = -4; % guess
  gP = -60;
  corrs = zeros(NreportMax,1);
    
  % ICM FIFO N Bytes, uDACS FIFO N Words, FIFO Contents
  rm_obj = read_multi_prep(100, 103, [101 495 102 0]);
  rm_hdr_len = 3; % 100, 103, 101
  % rm_obj = read_multi_prep(100, 103, 101);
  write_subbus_v(s, 48, 50+fs);
  write_subbus_v(s, 48, 40+2);
  N2 = floor(L/2);
  x = 1:N2;
  f = x*Fs/L;
  leftover = [];
  % n_leftover = 0;
  max_amp = 0;
  Nmem = 20;
  mem = zeros(N2,Nmem);
  imem = 1;
  report_icm_mode(s);
  
  tic;
  % cur_time = toc;
  while fig.UserData.start && Nreport <= NreportLim
    if fig.UserData.fs ~= fs
      new_fs = round(fig.UserData.fs);
      if new_fs < 0 || new_fs > 3
        new_fs = fs;
      end
    end
    [values,ack] = read_multi(s, rm_obj);
    Tsample = toc;
    if ack ~= 1; break; end
    vals = [leftover; values(rm_hdr_len+1:end)];
    Vneg = vals >= 32768;
    if any(Vneg)
      vals(Vneg) = vals(Vneg)-65536;
    end
    % fprintf(1,'nwps:%d vals:%d Nsecs:%d\n', Nwords_per_sec, length(vals), Nsecs);
    if Tsample0 < 0
      if ~isempty(vals)
        Tsample0 = Tsample; % we won't count this read
      end
    else
      Nwords_per_sec = Nwords_per_sec + length(vals);
      dT = Tsample-Tsample0;
      if dT >= 1
        Nsecs = Nsecs + 1;
        samples_per_sec(Nsecs) = Nwords_per_sec/dT/3;
        Nwords_per_sec = 0;
        Tsample0 = Tsample0 + floor(dT);
      end
    end
    % vals = [leftover; vals];
    nwords = length(vals);
    remainder = mod(nwords,3);
    nrows = (nwords-remainder)/3;
    % write out nrows to MLF 
    if bitand(values(1),3) ~= 2
      fprintf(1,'Mode value was %d\n', values(1));
      break;
    end
    if Nread + nrows > LS
      remainder = remainder + (Nread+nrows-LS)*3;
      nrows = LS-Nread;
    end
    try
      Data(Nread+(1:nrows),:) = ...
        reshape(vals(1:end-remainder),3,[])';
    catch
      fprintf(1,'Nread:%d nrows:%d remainder:%d\n', ...
        Nread, nrows, remainder);
      return;
    end
    Nread = Nread + nrows;
    if Nread == LS
      Data = Data * 2^(fs+1) / 32768;
      A = Data((1:L)+Skip_Samples,:);
      A = A - ones(L,1)*mean(A);
      % A = Data - ones(length(Data),1)*mean(Data);
      YA = fft(A)/L;
      VA = vecnorm(YA,2,2);
      maxVA = max(VA);
      if maxVA > max_amp
        max_amp = maxVA;
      end
      mem(:,imem) = VA(x);
      max_mem = max(mem,[],2);
      imem = mod(imem,Nmem)+1;
      Nread = 0;
      Nreport = Nreport+1;
      ReportT(Nreport) = Tsample;
      Terr = Tsample - Nreport;
      ITerr = ITerr + Terr;
      corr = round(gP*Terr+gITerr*ITerr);
      corrs(Nreport) = corr;
      Skip_Samples = Skip_Samples0 + corr;
      LS = L+Skip_Samples;
      % plot(ax(1),f,VA(x,:),f,max_mem,'.');
      % set(ax(1),'ylim',[0 max_amp],'xgrid','on','ygrid','on');
      % title(ax(1), sprintf('T = %.1f err=%.2f corr=%d',Tsample,Terr,corr));
      plot(ax(2),1:Nreport,corrs(1:Nreport));
      drawnow;
    end
    if remainder
      leftover = vals(end-remainder+(1:remainder));
    else
      leftover = [];
    end
    % n_leftover = remainder;
    % cur_time = toc;
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
  if nargout > 0
    sps = samples_per_sec(1:Nsecs);
    if nargout > 1
      T = ReportT(1:Nreport);
    end
  end
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
