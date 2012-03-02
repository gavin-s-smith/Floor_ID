function varargout = demoGUI(varargin)
% DEMOGUI MATLAB code for demoGUI.fig
%      DEMOGUI, by itself, creates a new DEMOGUI or raises the existing
%      singleton*.
%
%      H = DEMOGUI returns the handle to a new DEMOGUI or the handle to
%      the existing singleton*.
%
%      DEMOGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DEMOGUI.M with the given input arguments.
%
%      DEMOGUI('Property','Value',...) creates a new DEMOGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before demoGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to demoGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help demoGUI

% Last Modified by GUIDE v2.5 02-Mar-2012 11:34:52

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @demoGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @demoGUI_OutputFcn, ...
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


% --- Executes just before demoGUI is made visible.
function demoGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to demoGUI (see VARARGIN)

% Choose default command line output for demoGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes demoGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = demoGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
[pathname] = uigetdir;
    if isequal(pathname,0)
        
    else
        set(handles.text1,'String',pathname);
    end
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton1.
function pushbutton1_ButtonDownFcn(hObject, eventdata, handles)
    

% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
pathname = uigetdir;
    if isequal(pathname,0 )
        
    else
        set(handles.text2,'String',pathname);
    end
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
numberPart = demo(get(handles.text1,'String'),get(handles.text2,'String'));
vizResults([get(handles.text2,'String'),'/imgs'],num2str(numberPart));
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton4.
%function pushbutton4_Callback(hObject, eventdata, handles)
%dataViewDir = get(handles.text1,'String');
%numberPart = str2num(dataViewDir(length(dataViewDir)-3:length(dataViewDir)));
%vizResults([get(handles.text2,'String'),'/imgs'],num2str(numberPart));
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
