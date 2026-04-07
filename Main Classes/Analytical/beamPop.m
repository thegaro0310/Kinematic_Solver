function varargout = beamPop(varargin)
% BEAMPOP MATLAB code for beamPop.fig
%      BEAMPOP, by itself, creates a new BEAMPOP or raises the existing
%      singleton*.
%
%      H = BEAMPOP returns the handle to a new BEAMPOP or the handle to
%      the existing singleton*.
%
%      BEAMPOP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BEAMPOP.M with the given input arguments.
%
%      BEAMPOP('Property','Value',...) creates a new BEAMPOP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before beamPop_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to beamPop_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help beamPop

% Last Modified by GUIDE v2.5 11-Aug-2016 11:52:57

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @beamPop_OpeningFcn, ...
                   'gui_OutputFcn',  @beamPop_OutputFcn, ...
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


% --- Executes just before beamPop is made visible.
function beamPop_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to beamPop (see VARARGIN)

% Choose default command line output for beamPop
handles.output = hObject;
handles.widthHeightRatio=handles.beamGUI.Position(3)/handles.beamGUI.Position(4);
handles.beamGUI.Position=[0.2 0.2 0.8*handles.widthHeightRatio  0.8];
% Update handles structure
handles=colorEverything(handles);
guidata(hObject, handles);

% UIWAIT makes beamPop wait for user response (see UIRESUME)
% uiwait(handles.beamGUI);

function handles=colorEverything(handles)
%color the GUI
handles.beamGUI.Color= Colors.PanelBackground.getColor();

%text backgrounds
textAreas={'thicknessStatic','widthStatic','eStatic','m1','m2','gpa','fStatic',...
    'fyStatic','mStatic','n','n2','nmm','lengthStatic','m3','external'};
for i=1:length(textAreas)
    handles.(textAreas{i}).BackgroundColor=Colors.PanelBackground.getColor();
end
%text colors
for i=1:length(textAreas)
    handles.(textAreas{i}).ForegroundColor=Colors.Text.getColor();
end

handles.verticalLine.Visible='off';
handles.verticalLine.XLim=[0,1];
handles.verticalLine.YLim=[0,1];
line([0, 0], [0 1], 'Parent', handles.verticalLine,'LineWidth',3);
%initial values
handles.thickness.String='0.001';
handles.width.String='0.001';
handles.length.String='0.1';
handles.E.String='69';
handles.fx.String='1';
handles.fy.String='2';
handles.m.String='-0.1';
handles=analyze(handles);
[A,map,~]  = imread('singleBeam.png','BackGroundColor',Colors.PanelBackground.getColor());
imshow(A,map,'Parent',handles.schematics);
handles.schematics.HandleVisibility='off';
handles.schematics.Visible='off';



function handles=analyze(handles)
%analyze the beam
thickness=str2double(handles.thickness.String);
width=str2double(handles.width.String);
length=str2double(handles.length.String);
E=str2double(handles.E.String)*1e9;
Fx=str2double(handles.fx.String);
Fy=str2double(handles.fy.String);
M=str2double(handles.m.String);
handles.beam=BeamLite(E,length,thickness,width);
handles.beam=handles.beam.updateLoading(Fx,Fy,M);
tic
handles.beam=handles.beam.analytical();
toc
handles=plotAll(handles);


function handles=plotAll(handles)
%plot the requested plots
plotArray=zeros(1,4);
plotArray(handles.drawMode.Value)=1;
if logical(handles.external.Value)
    %draw on an external plot
    ax=axes(figure);
    cla(ax);
    handles.beam=handles.beam.draw(ax,plotArray);
else
    %draw on an internal plot
    ax=handles.plotArea;
    cla(ax);
    handles.beam=handles.beam.draw(ax,plotArray);
end


% --- Outputs from this function are returned to the command line.
function varargout = beamPop_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function thickness_Callback(hObject, ~, handles)
% hObject    handle to thickness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of thickness as text
%        str2double(get(hObject,'String')) returns contents of thickness as a double
if isnan(str2double(hObject.String)) || str2double(hObject.String) <=0
    hObject.String='0.001';
end
handles=analyze(handles);
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function thickness_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thickness (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function width_Callback(hObject, eventdata, handles)
% hObject    handle to width (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of width as text
%        str2double(get(hObject,'String')) returns contents of width as a double
if isnan(str2double(hObject.String)) || str2double(hObject.String) <=0
    hObject.String='0.001';
end
handles=analyze(handles);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function width_CreateFcn(hObject, eventdata, handles)
% hObject    handle to width (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function E_Callback(hObject, eventdata, handles)
% hObject    handle to E (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of E as text
%        str2double(get(hObject,'String')) returns contents of E as a double
if isnan(str2double(hObject.String)) || str2double(hObject.String) <=0
    hObject.String='69';
end
handles=analyze(handles);
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function E_CreateFcn(hObject, eventdata, handles)
% hObject    handle to E (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function fx_Callback(hObject, eventdata, handles)
% hObject    handle to fx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fx as text
%        str2double(get(hObject,'String')) returns contents of fx as a double
if isnan(str2double(hObject.String))
    hObject.String='1';
end
handles=analyze(handles);
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function fx_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta_Callback(hObject, eventdata, handles)
% hObject    handle to fy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fy as text
%        str2double(get(hObject,'String')) returns contents of fy as a double
if isnan(str2double(hObject.String))
    hObject.String='90';
end
handles=analyze(handles);
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function theta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function m_Callback(hObject, eventdata, handles)
% hObject    handle to m (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of m as text
%        str2double(get(hObject,'String')) returns contents of m as a double
if isnan(str2double(hObject.String))
    hObject.String='0.1';
end
handles=analyze(handles);
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function m_CreateFcn(hObject, eventdata, handles)
% hObject    handle to m (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function length_Callback(hObject, eventdata, handles)
% hObject    handle to length (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of length as text
%        str2double(get(hObject,'String')) returns contents of length as a double
if isnan(str2double(hObject.String)) || str2double(hObject.String) <=0
    hObject.String='0.1';
end
handles=analyze(handles);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function length_CreateFcn(hObject, eventdata, handles)
% hObject    handle to length (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in shapeCheck.
function shapeCheck_Callback(hObject, eventdata, handles)
% hObject    handle to shapeCheck (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of shapeCheck
handles=plotAll(handles);
guidata(hObject,handles);

% --- Executes on button press in curvatureCheck.
function curvatureCheck_Callback(hObject, eventdata, handles)
% hObject    handle to curvatureCheck (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of curvatureCheck
handles=plotAll(handles);
guidata(hObject,handles);


% --- Executes on button press in bendingCheck.
function bendingCheck_Callback(hObject, eventdata, handles)
% hObject    handle to bendingCheck (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of bendingCheck
handles=plotAll(handles);
guidata(hObject,handles);


% --- Executes on selection change in drawMode.
function drawMode_Callback(hObject, eventdata, handles)
% hObject    handle to drawMode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns drawMode contents as cell array
%        contents{get(hObject,'Value')} returns selected item from drawMode
handles=plotAll(handles);
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function drawMode_CreateFcn(hObject, eventdata, handles)
% hObject    handle to drawMode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in external.
function external_Callback(hObject, eventdata, handles)
% hObject    handle to external (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of external
handles=plotAll(handles);
guidata(hObject,handles);

% --- Executes on button press in plotAll.
function plotAll_Callback(hObject, eventdata, handles)
% hObject    handle to plotAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ax=axes(figure);
handles.beam=handles.beam.draw(ax,ones(1,4));
guidata(hObject,handles);


% --- Executes on button press in export.
function export_Callback(hObject, eventdata, handles)
% hObject    handle to export (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[file,path] = uiputfile('*.mat','Save Beam Data As');
if file ~= 0
    fileName=strcat(path,file);
    save(fileName,'-struct','handles','beam');
end



function fy_Callback(hObject, eventdata, handles)
% hObject    handle to fy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of fy as text
%        str2double(get(hObject,'String')) returns contents of fy as a double
if isnan(str2double(hObject.String))
    hObject.String='1';
end
handles=analyze(handles);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function fy_CreateFcn(hObject, eventdata, handles)
% hObject    handle to fy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
