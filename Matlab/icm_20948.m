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
  cmap = zeros(256,3);
  cmap(:,1) = linspace(1,0.3,256);
  cmap(:,3) = linspace(1,0.3,256);
  colormap(fig,cmap);
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
  discard = false;
    
  % 100: ICM FIFO Mode
  % 103: ICM Diagnostic (ICM FIFO count)
  % 101: uDACS FIFO N Words
  % 102: FIFO Contents
  % 0: placeholder to indicate a FIFO read
  rm_obj = read_multi_prep(100, 103, [101 495 102 0]);
  rm_hdr_len = 3; % 100, 103, 101
  % For fullscale update progress monitoring
  % Mode, N uDACS FIFO, N ICM FIFO
  rm_fsupdate = read_multi_prep(100, 101, 103);
  write_subbus_v(s, 48, 50+fs);
  write_subbus_v(s, 48, 40+2);
  N2 = floor(L/2);
  x = 1:N2;
  f = x*Fs/L;
  leftover = [];
  % n_leftover = 0;
  max_amp = 0;
  Npeaks = 3;
  Nmem = 20;
  memF = zeros(Npeaks,Nmem);
  memA = zeros(Npeaks,Nmem);
  memage = zeros(Npeaks,Nmem);
  imem = 1;

  scatter(ax(1),memF(:),memA(:),18,memage(:));
  set(ax(1),'xlim',[f(1) f(end)],'xgrid','on','ygrid','on');
  axes(ax(1));
  c = colorbar('Direction','reverse');
  c.Label.String = 'Age secs';
  ax1_sc = ax(1).Children;
  
  % Nspectra = 10;
  % spectra = zeros(N2,Nspectra);
  % ispectra = 0;

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

      if discard
        discard = false;
      else
        % can calculate the max acceleration over the last second:
        % Amax = max(vecnorm(Data(1:LS,:),2,2));
        % fprintf(1,'Amax = %.2f\n', Amax);
  
        % Now calculate the FFT
        A = Data((1:L)+Skip_Samples,:);
        A = A - ones(L,1)*mean(A);
        % A = Data - ones(length(Data),1)*mean(Data);
        YA = fft(A)/L;
        VA = vecnorm(YA,2,2);
        % driver will need to identify peaks here
        % this code is ultimately a ground function:
        maxVA = max(VA);
        if maxVA > max_amp
          max_amp = maxVA;
        end
        peaks = find_peaks(VA(x),Npeaks);
        memF(:,imem) = f(peaks(:,1));
        memA(:,imem) = peaks(:,2);
        memage = memage+1;
        memage(:,imem) = 0;
        % mem(:,imem) = VA(x);
        max_mem = max(max(memA));
        imem = mod(imem,Nmem)+1;

        % if ispectra < Nspectra
        %   ispectra = ispectra+1;
        %   spectra(:,ispectra) = VA(x);
        % end
      end
      %
      Nread = 0;
      Nreport = Nreport+1;
      ReportT(Nreport) = Tsample;
      Terr = Tsample - Nreport;
      ITerr = ITerr + Terr;
      corr = round(gP*Terr+gITerr*ITerr);
      corrs(Nreport) = corr;
      Skip_Samples = Skip_Samples0 + corr;
      if Skip_Samples < 0
        fprintf(1,'Skip_Samples underflow: %d\n', Skip_Samples);
        Skip_Samples = 0;
      end
      LS = L+Skip_Samples;
      % plot(ax(1),f,VA(x,:),f,max_mem,'.');
      % plot(ax(1),f,VA(x,:),f(peaks(:,1)),peaks(:,2),'*');
      % plot(ax(1),f,VA(x,:),memF(:),memA(:),'*');
      %scatter(ax(1),memF(:),memA(:),12,memage(:));
      ax1_sc.XData = memF(:);
      ax1_sc.YData = memA(:);
      ax1_sc.CData = memage(:);
      set(ax(1),'ylim',[0 max_mem]);
      % axes(ax(1));
      % c = colorbar('Direction','reverse');
      % c.Label.String = 'Age secs';
      title(ax(1), sprintf('T = %.1f err=%.2f corr=%d LS=%d',Tsample,Terr,corr,LS));
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
      T1 = toc;
      write_subbus_v(s, 48, 40); % Mode 0
      no_mode = 0+8*fs;
      while fig.UserData.start
        % For fullscale update progress monitoring
        % Mode, N uDACS FIFO, N ICM FIFO
        [values,~] = read_multi(s, rm_fsupdate);
        if all(values == [no_mode, 0, 0]')
          break;
        end
        T2 = toc;
        if T2-T1 > 1
          fprintf(1,'T2 no_mode expected [%d,0,0], read [%f,%f,%f]\n', no_mode, values);
          break;
        end
      end
      T2 = toc;
      write_subbus_v(s, 48, 50+new_fs);
      write_subbus_v(s, 48, 42);
      leftover = [];
      fs = new_fs;
      new_mode = 2+8*fs;
      while fig.UserData.start
        mode = read_subbus(s, 100);
        if mode == new_mode
          break;
        end
        T3 = toc;
        if T3-T2 > 1
          fprintf(1,'T3 new_mode expected %d, read %d\n', new_mode, mode);
          break;
        end
      end
      T3 = toc;
      fprintf(1,'%.2f %.2f %.2f %.2f: Set fs to %d\n', Tsample, T1, T2, T3, fs);
      discard = true;
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

function pks = find_peaks(F, N)
  % pks = find_peaks(F,N)
  % F is the FFT of the vibration data
  % N is the number of peaks to locate
  % pks(:,1) is the index of the peak locations in 
  pki = find(diff(sign(diff(F))) < 0)+1;
  [A,I] = sort(F(pki),'descend');
  pks = [pki(I(1:N)) A(1:N)];
end

