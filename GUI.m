function varargout = GUI(varargin)
% GUI MATLAB code for GUI.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      patientb to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI

% Last Modified by GUIDE v2.5 08-May-2017 20:01:36

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_OutputFcn, ...
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


% --- Executes just before GUI is made visible.
function GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI (see VARARGIN)

% Choose default command line output for GUI
handles.output = hObject;
handles.CZ = 1; handles.PZ = 1; handles.TZ = 1; handles.T = 1;
handles.volumeCZ = 0; handles.volumePZ = 0; handles.volumeTZ = 0; handles.volumeT = 0;
handles.boundariesCZ = []; handles.boundariesPZ = []; handles.boundariesTZ = []; handles.boundariesT = [];
handles.imagesCZ= []; handles.imagesPZ=[]; handles.imagesTZ=[]; handles.imagesT =[];
% handles.filenames = 0;
% handles.Jpathnames = 0;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in imageLoad.
function imageLoad_Callback(hObject, eventdata, handles)
% hObject    handle to imageLoad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[filenames, pathname, filterindex] = uigetfile (...
{ '*.*', 'All Files (*.*)'}, ...
 'Pick a file',...
 'MultiSelect', 'on');

     if (ischar(filenames )== 1)
    [X] = dicomread([pathname filenames]);
    info = dicominfo([pathname filenames]);
    set(handles.List, 'String', filenames);
    imshow(X,[], 'Parent', handles.imageViewer());
    else
    [X] = dicomread([pathname filenames{1,1}]);
    info = dicominfo([pathname filenames{1,1}]);
    set(handles.List, 'String', filenames);
    imshow(X,[], 'Parent', handles.imageViewer());
    end
    
set(handles.Patient, 'String', info.PatientName.FamilyName);
set(handles.PatientI, 'String', info.PatientID);
set(handles.PatientB, 'String', info.PatientBirthDate);
set(handles.StudyI, 'String', info.StudyID);
set(handles.StudyD, 'String', info.StudyDate);
set(handles.Slice, 'String', info.SliceLocation);
set(handles.Instance, 'String', info.InstanceNumber);

handles.X=X;
handles.info=info;
handles.pathname=pathname;
handles.filenames = filenames;
guidata(hObject,handles)






function Patient_Callback(hObject, eventdata, handles)
% hObject    handle to Patient (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Patient as text
%        str2double(get(hObject,'String')) returns contents of Patient as a double


% --- Executes during object creation, after setting all properties.
function Patient_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Patient (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function PatientI_Callback(hObject, eventdata, handles)
% hObject    handle to Patient (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Patient as text
%        str2double(get(hObject,'String')) returns contents of Patient as a double


% --- Executes during object creation, after setting all properties.
function PatientI_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Patient (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




function Slice_Callback(hObject, eventdata, handles)
% hObject    handle to StudyD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of StudyD as text
%        str2double(get(hObject,'String')) returns contents of StudyD as a double


% --- Executes during object creation, after setting all properties.
function Slice_CreateFcn(hObject, eventdata, handles)
% hObject    handle to StudyD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PatientB_Callback(hObject, eventdata, handles)
% hObject    handle to PatientB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PatientB as text
%        str2double(get(hObject,'String')) returns contents of PatientB as a double


% --- Executes during object creation, after setting all properties.
function PatientB_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PatientB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function StudyI_Callback(hObject, eventdata, handles)
% hObject    handle to StudyI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of StudyI as text
%        str2double(get(hObject,'String')) returns contents of StudyI as a double


% --- Executes during object creation, after setting all properties.
function StudyI_CreateFcn(hObject, eventdata, handles)
% hObject    handle to StudyI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function StudyD_Callback(hObject, eventdata, handles)
% hObject    handle to StudyD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of StudyD as text
%        str2double(get(hObject,'String')) returns contents of StudyD as a double


% --- Executes during object creation, after setting all properties.
function StudyD_CreateFcn(hObject, eventdata, handles)
% hObject    handle to StudyD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PatientN_Callback(hObject, eventdata, handles)
% hObject    handle to Slice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Slice as text
%        str2double(get(hObject,'String')) returns contents of Slice as a double


% --- Executes during object creation, after setting all properties.
function PatientN_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Slice (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Instance_Callback(hObject, eventdata, handles)
% hObject    handle to Instance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Instance as text
%        str2double(get(hObject,'String')) returns contents of Instance as a double


% --- Executes during object creation, after setting all properties.
function Instance_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Instance (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in List.
function List_Callback(hObject, eventdata, handles)
% hObject    handle to List (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns List contents as cell array
%        contents{get(hObject,'Value')} returns selected item from List

index = get(handles.List, 'Value');

if (isempty(strfind(handles.filenames{1,index}, 'jpg')) == 1)

[X] = dicomread([handles.pathname handles.filenames{1,index}]);
info = dicominfo([handles.pathname handles.filenames{1,index}]);
imshow(X,[], 'Parent', handles.imageViewer());

set(handles.Patient, 'String', info.PatientName.FamilyName);
set(handles.PatientI, 'String', info.PatientID);
set(handles.PatientB, 'String', info.PatientBirthDate);
set(handles.StudyI, 'String', info.StudyID);
set(handles.StudyD, 'String', info.StudyDate);
set(handles.Slice, 'String', info.SliceLocation);
set(handles.Instance, 'String', info.InstanceNumber);

 else

[X] = imread([handles.pathname handles.filenames{1,index}]);
set(handles.List, 'String', handles.filenames);
imshow(X,[], 'Parent', handles.imageViewer());

 end




% --- Executes during object creation, after setting all properties.
function List_CreateFcn(hObject, eventdata, handles)
% hObject    handle to List (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Anonymize.
function Anonymize_Callback(hObject, eventdata, handles)
% hObject    handle to Anonymize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

path = uigetdir;
index = get(handles.List, 'Value');
number = num2str(handles.filenames{1,index}(1,end));

[X] = dicomread([handles.pathname handles.filenames{1,index}]);
info =dicominfo([handles.pathname handles.filenames{1,index}]);

patientName = ['Patient ' number];

info.PatientName.FamilyName = patientName;

info.PatientBirthDate = 'xxxxx';

info.PatientID = 'xxxxx';

filename = [path '\' patientName];
dicomwrite(handles.X, filename, info);





% --- Executes on button press in AnonymizeAll.
function AnonymizeAll_Callback(hObject, eventdata, handles)
% hObject    handle to AnonymizeAll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[path] = uigetdir;
for i = 1:size(handles.filenames,2)

% index = num2str(handles.filenames{1,i}(1,end-1:end));

[X] = dicomread([handles.pathname handles.filenames{1,i}]);
info =dicominfo([handles.pathname handles.filenames{1,i}]);

patientName = handles.filenames{1,i};

info.PatientName.FamilyName = patientName;

info.PatientBirthDate = 'xxxxx';

info.PatientID = 'xxxxx';

filename =  [path '\' patientName];
dicomwrite(X, filename, info);
end




% --- Executes on button press in Left.
function Left_Callback(hObject, eventdata, handles)
% hObject    handle to Left (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


currentIndex = get(handles.List, 'Value');
previousIndex = currentIndex - 1;

    if (previousIndex == 0)
        previousIndex = 1;
    end

set(handles.List, 'Value', previousIndex);
h = findobj('Tag', 'List');
List_Callback(h, eventdata, handles);



% --- Executes on button press in Right.
function Right_Callback(hObject, eventdata, handles)
% hObject    handle to Right (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

currentIndex = get(handles.List, 'Value');
nextIndex = currentIndex + 1;
bound = length(get(handles.List, 'String'));

    if (nextIndex == bound+1)
        nextIndex = bound;
    end

set(handles.List, 'Value', nextIndex);
h = findobj('Tag', 'List');
List_Callback(h, eventdata, handles);

if (get(handles.segmentCheck,'Value') == 1)
s = findobj('Tag', 'Colors');
Colors_Callback(s,eventdata, handles);
end
 


% --- Executes on button press in ConvertJPG.
function ConvertJPG_Callback(hObject, eventdata, handles)
% hObject    handle to ConvertJPG (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

bound =length( get(handles.List, 'String'));
path = uigetdir;
for index = 1:bound

if (ischar(handles.filenames))
info = dicominfo([handles.pathname handles.filenames]);
image = dicomread([handles.pathname handles.filenames]);
image = (mat2gray(image));
filename= [path '\' handles.filenames '.jpg'];
matFile = [path '\' handles.filenames '.mat' ];
save (matFile, 'info');
imwrite(image, filename, 'jpg' , 'Quality',100, 'Mode', 'lossy');

else 
info = dicominfo([handles.pathname handles.filenames{1,index}]);
image = dicomread([handles.pathname handles.filenames{1,index}]);
image = mat2gray(image);
filename= [path '\' handles.filenames{1,index} '.jpg'];
matFile = [path '\' handles.filenames{1,index} '.mat' ];
save (matFile, 'info');
imwrite(image, filename, 'jpg' , 'Quality',100, 'Mode', 'lossy');
end 
end

% --- Executes on button press in ConvertDCM.
function ConvertDCM_Callback(hObject, eventdata, handles)
% hObject    handle to ConvertDCM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

bound =length( get(handles.List, 'String'));
path = uigetdir;
for index = 1:bound

if (ischar(handles.filenames)==1)

image = imread([handles.pathname handles.filenames]);
filename= [path '\' handles.filenames];
[~,name,~] = fileparts(filename);
metadata_name = [handles.pathname  name '.mat'];
image_name = [path '\' name '.dcm'];
metadata = load(metadata_name);
dicomwrite(image, image_name, metadata.info,'CreateMode', 'create', 'WriteMode' , 1);


else 
image = imread([handles.pathname handles.filenames{1,index}]);
filename= [path '\' handles.filenames{1,index}];
[~,name,~] = fileparts(filename);
metadata_name = [handles.pathname  name '.mat'];
image_name = [path '\' name '.dcm'];
metadata = load(metadata_name);
dicomwrite(image, image_name, metadata.info,'CreateMode', 'copy' ,'WritePrivate', 1);
% display(handles.pathname);
end    
end

function PatientI_DeleteFcn(hObject, eventdata, handles)
% --- Executes on selection change in Colors.
function Colors_Callback(hObject, eventdata, handles)
% hObject    handle to Colors (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns Colors contents as cell array
%        contents{get(hObject,'Value')} returns selected item from Colors


color = get(hObject, 'String');
stringValue = get(hObject, 'Value');
index = get(handles.List, 'Value');
image= (dicomread([handles.pathname handles.filenames{1,index}]));
info = dicominfo([handles.pathname handles.filenames{1,index}]);

if (stringValue == 2)
       
    handles.segmentCZ = imfreehand();
    setColor(handles.segmentCZ, 'red');
    binaryImage = handles.segmentCZ.createMask();
    measurements = regionprops(binaryImage, 'area');
    surfaceArea = (prod(info.PixelSpacing))*measurements.Area;
    volume = 1.25*surfaceArea;
    handles.volumeCZ = handles.volumeCZ + volume;
    structBoundaries = bwboundaries(binaryImage);
    xy =structBoundaries{1};
    handles.boundariesCZ = [handles.boundariesCZ;
                                           xy];

    handles.imagesCZ(:,:,handles.CZ) = uint8(binaryImage);
%      display(size(handles.imagesCZ));
    handles.CZ = handles.CZ + 1;
     
end

if (stringValue == 3)
      handles.segmentPZ = imfreehand();
      setColor(handles.segmentPZ, 'green');
      binaryImage = handles.segmentPZ.createMask();
    measurements = regionprops(binaryImage, 'area');
    surfaceArea = (prod(info.PixelSpacing))*measurements.Area;
    volume = 1.25*surfaceArea;
    handles.volumePZ = handles.volumePZ + volume;
    structBoundaries = bwboundaries(binaryImage);
    xy =structBoundaries{1};
    handles.boundariesPZ = [handles.boundariesPZ;
                                        xy];

    handles.imagesPZ(:,:,handles.PZ) = uint8(binaryImage);
%     display(size(handles.imagesPZ));
    handles.PZ = handles.PZ + 1;
end

if (stringValue == 4)
      handles.segmentTZ = imfreehand();
        setColor(handles.segmentTZ, 'blue');
        binaryImage = handles.segmentTZ.createMask();
    measurements = regionprops(binaryImage, 'area');
    surfaceArea = (prod(info.PixelSpacing))*measurements.Area;
    volume = 1.25*surfaceArea;
    handles.volumeTZ = handles.volumeTZ + volume;
    structBoundaries = bwboundaries(binaryImage);
    xy =structBoundaries{1};
    handles.boundariesTZ = [handles.boundariesTZ;
                                        xy];

    handles.imagesTZ(:,:,handles.TZ) = uint8(binaryImage);
%      display(size(handles.imagesTZ));
    handles.TZ = handles.TZ + 1;
end

if (stringValue == 5)
      handles.segmentT = imfreehand();
    setColor(handles.segmentT, 'white');
    binaryImage = handles.segmentT.createMask();
    measurements = regionprops(binaryImage, 'area');
    surfaceArea = (prod(info.PixelSpacing))*measurements.Area;
    volume = 1.25*surfaceArea;
    handles.volumeT = handles.volumeT + volume;
    structBoundaries = bwboundaries(binaryImage);
    xy =structBoundaries{1};
    handles.boundariesT = [handles.boundariesT;
                                        xy];

    handles.imagesT(:,:,handles.T) = uint8(binaryImage);
%     display(size(handles.imagesT));
    handles.T = handles.T + 1;
end
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function Colors_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Colors (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Show3D.
function Show3D_Callback(hObject, eventdata, handles)
% hObject    handle to Show3D (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if  (~(isempty(handles.imagesCZ)));
imagesCZ = [];
xCZ = handles.boundariesCZ(:,2);
yCZ = handles.boundariesCZ(:,1);

    leftColumnCZ = min(xCZ);
    rightColumnCZ = max(xCZ);
    topLineCZ = min(yCZ);
    bottomLineCZ = max(yCZ);
    widthCZ = rightColumnCZ - leftColumnCZ +1;
    heightCZ = bottomLineCZ -topLineCZ +1;
    
     for i = 1:size(handles.imagesCZ,3)
     imagesCZ(:,:,i) = imcrop(handles.imagesCZ(:,:,i), [ leftColumnCZ, topLineCZ, widthCZ, heightCZ]);
    end

end

if  (~(isempty(handles.imagesPZ)));
imagesPZ = [];
xPZ = handles.boundariesPZ(:,2);
yPZ = handles.boundariesPZ(:,1);

    leftColumnPZ = min(xPZ);
    rightColumnPZ = max(xPZ);
    topLinePZ = min(yPZ);
    bottomLinePZ = max(yPZ);
    widthPZ = rightColumnPZ - leftColumnPZ +1;
    heightPZ = bottomLinePZ -topLinePZ +1;
    
     for i = 1:size(handles.imagesPZ,3)
     imagesPZ(:,:,i) = imcrop(handles.imagesPZ(:,:,i), [ leftColumnPZ, topLinePZ, widthPZ, heightPZ]);
    end
end

if  (~(isempty(handles.imagesTZ)));
imagesTZ = [];
xTZ = handles.boundariesTZ(:,2);
yTZ = handles.boundariesTZ(:,1);

    leftColumnTZ = min(xTZ);
    rightColumnTZ = max(xTZ);
    topLineTZ = min(yTZ);
    bottomLineTZ = max(yTZ);
    widthTZ = rightColumnTZ - leftColumnTZ +1;
    heightTZ = bottomLineTZ -topLineTZ +1;
    
     for i = 1:size(handles.imagesTZ,3)
     imagesTZ(:,:,i) = imcrop(handles.imagesTZ(:,:,i), [ leftColumnTZ, topLineTZ, widthTZ, heightTZ]);
    end
end

if  (~(isempty(handles.imagesT)));
imagesT = [];
xT = handles.boundariesT(:,2);
yT = handles.boundariesT(:,1);

    leftColumnT = min(xT);
    rightColumnT = max(xT);
    topLineT = min(yT);
    bottomLineT = max(yT);
    widthT = rightColumnT - leftColumnT +1;
    heightT = bottomLineT -topLineT +1;
    
     for i = 1:size(handles.imagesT,3)
     imagesT(:,:,i) = imcrop(handles.imagesT(:,:,i), [ leftColumnT, topLineT, widthT, heightT]);
    end
end


    
%      imshow(images(:,:,1),[]);
% [rows,cols,slices] = size(imagesCZ);
% [X,Y,Z] = meshgrid(1:cols, 1:rows, 1:slices);
% [X2,Y2,Z2] = meshgrid(1:cols, 1:rows, 0.05:0.05:slices);
% out = interp3(X, Y, Z, images, X2, Y2, Z2, 'cubic', 0);   
% display (size(out));
% out = squeeze(out);       

if (exist('imagesCZ'))
figure
%#D = smooth3(D);
p = patch( isosurface(imagesCZ,0) );
isonormals(imagesCZ, p);
set(p, 'FaceColor',[1,.75,.65], 'EdgeColor','none')
daspect([1 1 .5]), view(3), axis tight, axis vis3d
camlight, lighting gouraud
%# add isocaps
patch(isocaps(imagesCZ,0), 'FaceColor','interp', 'EdgeColor','none');
end 


if (exist('imagesPZ'))
figure;
p = patch( isosurface(imagesPZ,0) );
isonormals(imagesPZ, p);
set(p, 'FaceColor',[1,.75,.65], 'EdgeColor','none')
daspect([1 1 .5]), view(3), axis tight, axis vis3d
camlight, lighting gouraud
%# add isocaps
patch(isocaps(imagesPZ,0), 'FaceColor','interp', 'EdgeColor','none');
end



if (exist('imagesTZ'))
figure;
p = patch( isosurface(imagesTZ,0) );
isonormals(imagesTZ, p);
set(p, 'FaceColor',[1,.75,.65], 'EdgeColor','none')
daspect([1 1 .5]), view(3), axis tight, axis vis3d
camlight, lighting gouraud
%# add isocaps
patch(isocaps(imagesTZ,0), 'FaceColor','interp', 'EdgeColor','none');
end


if (exist('imagesT'))
figure;
p = patch( isosurface(imagesT,0) );
isonormals(imagesT, p);
set(p, 'FaceColor',[1,.75,.65], 'EdgeColor','none')
daspect([1 1 .5]), view(3), axis tight, axis vis3d
camlight, lighting gouraud
%# add isocaps
patch(isocaps(imagesT,0), 'FaceColor','interp', 'EdgeColor','none');
end  

% --- Executes on button press in segmentCheck.
function segmentCheck_Callback(hObject, eventdata, handles)
% hObject    handle to segmentCheck (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of segmentCheck
if(get(handles.segmentCheck, 'Value') == 1)
    h = findobj('Tag', 'Colors');
    Colors_Callback(h, eventdata, handles)
else
    
end

% --- Executes on button press in LoadJPG.
function LoadJPG_Callback(hObject, eventdata, handles)
% hObject    handle to LoadJPG (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


 [filenames, pathname, filterindex] = uigetfile (...
{ '*.jpg'}, ...
 'Pick a file',...
 'MultiSelect', 'on');
    
    if (ischar(filenames )== 1)
    [X] = imread([pathname filenames]);
    set(handles.List, 'String', filenames);
    imshow(X,[], 'Parent', handles.imageViewer());
    else
    [X] = imread([pathname filenames{1,1}]);
    set(handles.List, 'String', filenames);
    imshow(X,[], 'Parent', handles.imageViewer());
    end
    
   handles.pathname = pathname;
   handles.filenames = filenames;  
  guidata(hObject, handles);

  


% --- Executes on button press in Volume.
function Volume_Callback(hObject, eventdata, handles)
% hObject    handle to Volume (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if (~(isempty(handles.volumeCZ)))
    set(handles.CentralZone, 'String', handles.volumeCZ);

end
 if (~(isempty(handles.volumePZ)))
      set(handles.PeripheralZone, 'String', handles.volumePZ);
 end
 
if (~(isempty(handles.volumeTZ)))
     set(handles.TransitionZone, 'String', handles.volumeTZ);
end

if (~(isempty(handles.volumeT)))
     set(handles.Tumor, 'String', handles.volumeT);
end
guidata(hObject, handles);



function CentralZone_Callback(hObject, eventdata, handles)
% hObject    handle to CentralZone (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of CentralZone as text
%        str2double(get(hObject,'String')) returns contents of CentralZone as a double


% --- Executes during object creation, after setting all properties.
function CentralZone_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CentralZone (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PeripheralZone_Callback(hObject, eventdata, handles)
% hObject    handle to PeripheralZone (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PeripheralZone as text
%        str2double(get(hObject,'String')) returns contents of PeripheralZone as a double


% --- Executes during object creation, after setting all properties.
function PeripheralZone_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PeripheralZone (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function TransitionZone_Callback(hObject, eventdata, handles)
% hObject    handle to TransitionZone (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of TransitionZone as text
%        str2double(get(hObject,'String')) returns contents of TransitionZone as a double


% --- Executes during object creation, after setting all properties.
function TransitionZone_CreateFcn(hObject, eventdata, handles)
% hObject    handle to TransitionZone (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Tumor_Callback(hObject, eventdata, handles)
% hObject    handle to Tumor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Tumor as text
%        str2double(get(hObject,'String')) returns contents of Tumor as a double


% --- Executes during object creation, after setting all properties.
function Tumor_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Tumor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
