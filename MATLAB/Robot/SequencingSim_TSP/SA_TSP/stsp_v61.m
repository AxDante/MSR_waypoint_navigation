function varargout = stsp_v61(varargin)
% STSP_V61 M-file for stsp_v61.fig
%      STSP_V61, by itself, creates a new STSP_V61 or raises the existing
%      singleton*.
%
%      H = STSP_V61 returns the handle to a new STSP_V61 or the handle to
%      the existing singleton*.
%
%      STSP_V61('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in STSP_V61.M with the given input arguments.
%
%      STSP_V61('Property','Value',...) creates a new STSP_V61 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before stsp_v61_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to stsp_v61_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help stsp_v61

% Last Modified by GUIDE v2.5 12-Jan-2006 22:46:00

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @stsp_v61_OpeningFcn, ...
                   'gui_OutputFcn',  @stsp_v61_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before stsp_v61 is made visible.
function stsp_v61_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to stsp_v61 (see VARARGIN)

% Choose default command line output for stsp_v61
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes stsp_v61 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = stsp_v61_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function initial_temperature_Callback(hObject, eventdata, handles)
% hObject    handle to initial_temperature (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of initial_temperature as text
%        str2double(get(hObject,'String')) returns contents of initial_temperature as a double
s = str2double(get(hObject,'String'));
if isnan(s)
    set(hObject,'String','1000')
end
if s < 0 | s > 10000
    set(hObject,'String','1000')
end

% --- Executes during object creation, after setting all properties.
function initial_temperature_CreateFcn(hObject, eventdata, handles)
% hObject    handle to initial_temperature (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function maximum_iterations_Callback(hObject, eventdata, handles)
% hObject    handle to maximum_iterations (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of maximum_iterations as text
%        str2double(get(hObject,'String')) returns contents of maximum_iterations as a double
s = str2double(get(hObject,'String'));
if isnan(s)
    set(hObject,'String','10000')
end
if s < 1000 | s > 1000000
    set(hObject,'String','10000')
end

% --- Executes during object creation, after setting all properties.
function maximum_iterations_CreateFcn(hObject, eventdata, handles)
% hObject    handle to maximum_iterations (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on slider movement.
function cooling_rate_Callback(hObject, eventdata, handles)
% hObject    handle to cooling_rate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
s = sprintf('Cooling Rate : %0.3f',get(hObject,'Value'));
set(handles.cooling_rate_display, 'String', s);
% --- Executes during object creation, after setting all properties.
function cooling_rate_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cooling_rate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function cities_to_swap_Callback(hObject, eventdata, handles)
% hObject    handle to cities_to_swap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
s = sprintf('Cities to swap : %d',get(hObject,'Value'));
set(handles.cities_to_swap_display,'String',s)

% --- Executes during object creation, after setting all properties.
function cities_to_swap_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cities_to_swap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function cooling_rate_display_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cooling_rate_display (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes during object creation, after setting all properties.
function cities_to_swap_display_CreateFcn(hObject, eventdata, handles)
% hObject    handle to cities_to_swap_display (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in start_button.
function start_button_Callback(hObject, eventdata, handles)
% hObject    handle to start_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = get(handles.select_data_set, 'Value');
switch selection
    case 2
        data_file;
    case 3
        loadbays29;
    case 4
        loadatt48;
    case 5
        loadst70;
    case 6
        loadpr76;
    case 7
        loadgr96;
    case 8
        loadeil101;
    case 9
        loadpcb442;
    case 10
        loadeil535;
end
if selection ~= 1
    set(handles.initial_temperature, 'Enable', 'Off')
    set(handles.cooling_rate, 'Enable', 'Off')
    set(handles.maximum_iterations, 'Enable', 'Off')
    set(handles.cities_to_swap, 'Enable', 'Off')
    set(handles.start_button, 'Enable', 'Off')
    set(handles.select_data_set, 'Enable', 'Off')
    cities = load ('cities.mat');
    cities = cities.cities;
    simulatedannealing(cities,...
        str2double(get(handles.initial_temperature,'String')),...
        get(handles.cooling_rate,'Value'),...
        str2double(get(handles.maximum_iterations,'String')),...
        get(handles.cities_to_swap,'Value'));
    set(handles.initial_temperature, 'Enable', 'On')
    set(handles.cooling_rate, 'Enable', 'On')
    set(handles.maximum_iterations, 'Enable', 'On')
    set(handles.cities_to_swap, 'Enable', 'On')
    set(handles.start_button, 'Enable', 'On')
    set(handles.select_data_set, 'Enable', 'On')
end

% --- Executes on selection change in select_data_set.
function select_data_set_Callback(hObject, eventdata, handles)
% hObject    handle to select_data_set (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns select_data_set contents as cell array
%        contents{get(hObject,'Value')} returns selected item from select_data_set

% --- Executes during object creation, after setting all properties.
function select_data_set_CreateFcn(hObject, eventdata, handles)
% hObject    handle to select_data_set (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


