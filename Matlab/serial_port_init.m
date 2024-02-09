function [s, port_out] = serial_port_init(port,baudrate)
% s = serial_port_init(port);
% [s, port] = serial_port_init;
% Returns an open serial object with timeout set to 0.1
% The serial object should be closed with the following sequence:
% fclose(s);
% delete(s);
% clear s;
%
% If opening the serial port fails, an error dialog is displayed and
% an empty array is returned.
s = [];
if nargout > 1
  port_out = '';
end
cfg_loaded = 0;
if (nargin < 1 || isempty(port))
  port = '';
  if exist('./uDACS16SerialPort.mat','file')
    rport = load('./uDACS16SerialPort.mat');
    if isfield(rport,'port')
      port = rport.port;
    end
    if isfield(rport,'baudrate')
      baudrate = rport.baudrate;
    end
    clear rport
    cfg_loaded = 1;
  elseif nargin < 2
    baudrate = 57600;
  end
elseif nargin < 2
  baudrate = 57600;
end

ports = serialportlist("all");
if isempty(ports)
  port = '';
elseif length(ports) == 1
  port = ports{1};
else
  if ~isempty(port) && ...
      ~any(strcmpi(port, ports))
    % This is not a good choice
    port = '';
  end
end
if isempty(port)
  if isempty(ports)
    % closereq;
    h = errordlg('No serial port found','uDACS16 Serial Port Error','modal');
    uiwait(h);
    return;
  else
    sel = listdlg('ListString',ports,...
      'SelectionMode','single','Name','uDACS16_Port', ...
      'PromptString','Select Serial Port:', ...
      'ListSize',[160 50]);
    if isempty(sel)
      % closereq;
      return;
    else
      port = ports{sel};
    end
  end
end

if ~cfg_loaded
  save uDACS16SerialPort.mat port baudrate
end

if nargout > 1
  port_out = port;
end
isobj = 0;
isopen = 0;
try
  s = serialport(port,baudrate,'Timeout',0.1); % 'InputBufferSize',3000
  configureTerminator(s,"LF");
  warning('off','serialport:serialport:ReadlineWarning');
  tline = 'a';
  while ~isempty(tline)
    tline = readline(s);
  end
  if nargout < 2
    fprintf(1, 'Successfully opened port %s\n', port);
  end
catch ME
  h = errordlg(sprintf('Error: %s\nMessage: %s\nport = %s\n', ...
    ME.identifier, ME.message, port), ...
    'uDACS16 serialport Error', 'modal');
  uiwait(h);
  delete(s);
  s = [];
end
