function varargout = RobotControl_GUI(varargin)
%ROBOTCONTROL_GUI M-file for RobotControl_GUI.fig
%      ROBOTCONTROL_GUI, by itself, creates a new ROBOTCONTROL_GUI or raises the existing
%      singleton*.
%
%      H = ROBOTCONTROL_GUI returns the handle to a new ROBOTCONTROL_GUI or the handle to
%      the existing singleton*.
%
%      ROBOTCONTROL_GUI('Property','Value',...) creates a new ROBOTCONTROL_GUI using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to RobotControl_GUI_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      ROBOTCONTROL_GUI('CALLBACK') and ROBOTCONTROL_GUI('CALLBACK',hObject,...) call the
%      local function named CALLBACK in ROBOTCONTROL_GUI.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help RobotControl_GUI

% Last Modified by GUIDE v2.5 09-Aug-2014 00:47:38

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @RobotControl_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @RobotControl_GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
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


% --- Executes just before RobotControl_GUI is made visible.
function RobotControl_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)
model_open(handles)
% Choose default command line output for RobotControl_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

num=get(handles.front_obs,'Value');
set(handles.front_obstacle,'String',num);
num=get(handles.left_obs,'Value');
set(handles.left_obstacle,'String',num);
num=get(handles.right_obs,'Value');
set(handles.right_obstacle,'String',num);
num=get(handles.back_obs,'Value');
set(handles.back_obstacle,'String',num);
% UIWAIT makes RobotControl_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = RobotControl_GUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function model_open(handles)
% Make sure the diagram is still open
  if  isempty(find_system('Name','FuzzyMobileRobotGuidance')),
    open_system('FuzzyMobileRobotGuidance'); 
    open_system('FuzzyMobileRobotGuidance/Robot/Obstacle Sensor')
       %set_param('FuzzyMobileRobotGuidance/Robot/Obstacle Sensor/Front Gain','Position',[275 14 340 56])
        % Put  values of Kf and Ki from the GUI into the Block dialogs
    set_param('FuzzyMobileRobotGuidance/Robot/Obstacle Sensor/Left Gain','Gain',...
                            get(handles.left_obs,'String'))
    set_param('FuzzyMobileRobotGuidance/Robot/Obstacle Sensor/Front Gain','Gain',...
                            get(handles.front_obs,'String'))
    set_param('FuzzyMobileRobotGuidance/Robot/Obstacle Sensor/Right Gain','Gain',...
                            get(handles.right_obs,'String'))
    set_param('FuzzyMobileRobotGuidance/Robot/Obstacle Sensor/Back Gain','Gain',...
                            get(handles.back_obs,'String')) 
    set_param('FuzzyMobileRobotGuidance/Target Location','Value',...
                            get(handles.target_lat,'String') ,'Value',get(handles.target_lon,'String'))                    
    
  end
%endfunction model_open

function left_obstacle_Callback(hObject, eventdata, handles)
% hObject    handle to left_obstacle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of left_obstacle as text
%        str2double(get(hObject,'String')) returns contents of left_obstacle as a double


% --- Executes during object creation, after setting all properties.
function left_obstacle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to left_obstacle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function front_obstacle_Callback(hObject, eventdata, handles)
% hObject    handle to front_obstacle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of front_obstacle as text
%        str2double(get(hObject,'String')) returns contents of front_obstacle as a double


% --- Executes during object creation, after setting all properties.
function front_obstacle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to front_obstacle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function target_lat_Callback(hObject, eventdata, handles)
% hObject    handle to target_lat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of target_lat as text
%        str2double(get(hObject,'String')) returns contents of target_lat as a double


% --- Executes during object creation, after setting all properties.
function target_lat_CreateFcn(hObject, eventdata, handles)
% hObject    handle to target_lat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function target_lon_Callback(hObject, eventdata, handles)
% hObject    handle to target_lon (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of target_lon as text
%        str2double(get(hObject,'String')) returns contents of target_lon as a double


% --- Executes during object creation, after setting all properties.
function target_lon_CreateFcn(hObject, eventdata, handles)
% hObject    handle to target_lon (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function front_obs_Callback(hObject, eventdata, handles)
% hObject    handle to front_obs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num=get(hObject,'Value');
set(handles.front_obstacle,'String',num);
model_open(handles)
set_param('FuzzyMobileRobotGuidance/Robot/Obstacle Sensor/Front Gain','Gain',num2str(num))
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function front_obs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to front_obs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function back_obs_Callback(hObject, eventdata, handles)
% hObject    handle to back_obs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num=get(hObject,'Value');
set(handles.back_obstacle,'String',num);
model_open(handles)
set_param('FuzzyMobileRobotGuidance/Robot/Obstacle Sensor/Back Gain','Gain',num2str(num))
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function back_obs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to back_obs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function right_obs_Callback(hObject, eventdata, handles)
% hObject    handle to right_obs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num=get(hObject,'Value');
set(handles.right_obstacle,'String',num);
model_open(handles)
set_param('FuzzyMobileRobotGuidance/Robot/Obstacle Sensor/Right Gain','Gain',num2str(num))
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function right_obs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to right_obs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function left_obs_Callback(hObject, eventdata, handles)
% hObject    handle to left_obs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
num=get(hObject,'Value');
set(handles.left_obstacle,'String',num);
model_open(handles)
set_param('FuzzyMobileRobotGuidance/Robot/Obstacle Sensor/Left Gain','Gain',num2str(num))
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function left_obs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to left_obs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in setbutton.
function setbutton_Callback(hObject, eventdata, handles)
% hObject    handle to setbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
lat=get(handles.target_lat,'String');
lon=get(handles.target_lon,'String');
loc=strcat('[',lat,', ', lon,']');
set_param('FuzzyMobileRobotGuidance/TargetLocation','Value',loc)


function right_obstacle_Callback(hObject, eventdata, handles)
% hObject    handle to right_obstacle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of right_obstacle as text
%        str2double(get(hObject,'String')) returns contents of right_obstacle as a double


% --- Executes during object creation, after setting all properties.
function right_obstacle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to right_obstacle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function back_obstacle_Callback(hObject, eventdata, handles)
% hObject    handle to back_obstacle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of back_obstacle as text
%        str2double(get(hObject,'String')) returns contents of back_obstacle as a double


% --- Executes during object creation, after setting all properties.
function back_obstacle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to back_obstacle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in start_button.
function start_button_Callback(hObject, eventdata, handles)
% hObject    handle to start_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
sim('FuzzyMobileRobotGuidance');

% --- Executes on button press in stop_button.
function stop_button_Callback(hObject, eventdata, handles)
% hObject    handle to stop_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set_param('FuzzyMobileRobotGuidance','SimulationCommand','stop')


% --- Executes on button press in random_btn.
function random_btn_Callback(hObject, eventdata, handles)
% hObject    handle to random_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
vrclose;
world =  vrworld('Robot','reuse');
open(world);
fig =vrfigure(world);
Obstacle=vrnode(world,'Obstacle');
Buil=vrnode(world,'Building3');
Buil2=vrnode(world,'Building2');
Buil1=vrnode(world,'Building1');
Rose=vrnode(world,'Rose');
Tree4=vrnode(world,'Tree4');
Tree2=vrnode(world,'Tree2');
Tree1=vrnode(world,'Tree1');
palm_tree2=vrnode(world,'palm_tree2');
palm_tree=vrnode(world,'palm_tree');
Car=vrnode(world,'Car');
set(fig, 'Viewpoint', 'side_view');
    T1=randi([-1000 1500],1,3);
    T2=randi([-1000 1500],1,3);
    T3=randi([-1000 1500],1,3);
    T4=randi([-1000 1500],1,3);
    T5=randi([-1000 1500],1,3);
    T6=randi([-1000 1500],1,3);
    T7=randi([-1000 1500],1,3);
    T8=randi([-1000 1500],1,3);
    T9=randi([-1000 1500],1,3);
    T10=randi([-1000 1500],1,3);
    Obstacle.translation=[T1(1,1) 0 T1(1,3)];
    Buil.translation=[T2(1,1) 0 T2(1,3)];
    Buil2.translation=[T3(1,1) 0 T3(1,3)];
    Buil1.translation=[T4(1,1) 0 T4(1,3)];
    Rose.translation=[T5(1,1) 0 T5(1,3)];
    Tree4.translation=[T6(1,1) 0 T6(1,3)];
    Tree2.translation=[T7(1,1) 0 T7(1,3)];
    Tree1.translation=[T8(1,1) 0 T8(1,3)];
    palm_tree2.translation=[T9(1,1) 0 T9(1,3)];
    palm_tree.translation=[T10(1,1) 0 T10(1,3)];
    p1=Obstacle.translation;
    p2=Buil.translation;
    p3=Car.translation;
    display(p1);
    display(p3); 
    theta= atan((p1(1,3)-p3(1,3))/(p1(1,1)-p3(1,1)));
    %theta=direction(p3(1,1),p3(1,3),p1(1,1),p1(1,3));
    display(theta);
    vrdrawnow;
