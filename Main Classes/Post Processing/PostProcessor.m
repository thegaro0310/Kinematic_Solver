classdef PostProcessor < handle
    %post processor
    properties
        figures;
        handles;
        sessionKinematics=Kinematics.empty;
        sessionLoadAnalysis=LoadAnalysis.empty;
        sessionDistanceAnalysis=DistanceAnalysis.empty;
        sessionAdvantageAnalysis=Advantage.empty;
        sessionBistable=BiStable.empty;
        mode=0;
        currentSession=0;
        currentRun=0;
        currentFrame=1;
        directory;
        busy=0;
        %data
        nodes=Node.empty;
        initialNodeNumber=0;
        allBeams=cell(0);
        forces=Force.empty;
        moments=Moment.empty;
        torsionSprings=TorsionSpring.empty;
        currentWorkSpace=WorkSpace.empty;
        originalNodes=[];
        stateList=[];
        powerList=[];
        magnitudeList=[];
        inputLoad=[];
        distance=[];
        loadList=[];
        energyList=[];
        %
        xAxis=struct('mode',0,'type',[],'frames',[],'id',0,'session',0);
        yAxis=struct('mode',0,'type',[],'frames',[],'id',0,'session',0);
        yInput=1;
        trackedNodes=[];
        dummyLines=gobjects(0);
        colors=[0.2745 0.1255 0.4000;1.0000 0.7216 0.3725;1.0000 0.4784 0.3529;0 0.6667 0.6275;0.5569 0.8235 0.7882;0.9882 0.9569 0.8510];
    end
    
    methods
        function obj=PostProcessor(workSpace,sessionKinematics,sessionLoadAnalysis,sessionDistanceAnalysis,sessionAdvantageAnalysis,sessionBistable,directory)
            %constructor
            obj.currentWorkSpace=workSpace;
            obj.sessionKinematics=sessionKinematics;
            obj.sessionLoadAnalysis=sessionLoadAnalysis;
            obj.sessionDistanceAnalysis=sessionDistanceAnalysis;
            obj.sessionAdvantageAnalysis=sessionAdvantageAnalysis;
            obj.sessionBistable=sessionBistable;
            obj.directory=directory;
        end
        
        function handle=getHandles(obj)
            %return the handle
            handle=guidata(obj.handles);
        end
        
        function numberSessions=numberOfSessions(obj)
            %number of sessions
            numberSessions=[length(obj.sessionKinematics) length(obj.sessionLoadAnalysis) length(obj.sessionDistanceAnalysis) length(obj.sessionAdvantageAnalysis) length(obj.sessionBistable)];
        end
        
        function deleteAll(obj)
            %delete everything
            for i=1:length(obj.nodes)
                obj.nodes(i)=obj.nodes(i).deleteNodeDrawing();
            end
            for i=1:length(obj.allBeams)
                obj.allBeams{i}=obj.allBeams{i}.deleteLinkDrawing();
            end
            if isgraphics(obj.dummyLines)
                delete(obj.dummyLines);
            end
            %delete forces
            for i=1:length(obj.forces)
                obj.forces(i)=obj.forces(i).deleteDrawing();
            end
            %delete moments
            for i=1:length(obj.moments)
                obj.moments(i)=obj.moments(i).deleteDrawing();
            end
            %delete torsion springs
            for i=1:length(obj.torsionSprings)
                obj.torsionSprings(i)=obj.torsionSprings(i).deleteDrawing();
            end
            if ~isempty(obj.distance)
                obj.distance.line();
            end
            cla();
        end
        
        function deleteTrack(obj,id)
            %delete a tracked node
            if id> obj.initialNodeNumber
                obj.nodes(id)=[];
                for i=id:length(obj.nodes)
                    obj.nodes(i)=obj.nodes(i).setID(obj.nodes(i).getID() -1);
                end
                handles=obj.getHandles();
                for i=1:size(handles.trackedNodesPopup.String)
                    if str2double(handles.trackedNodesPopup.String{i}) > id
                        handles.trackedNodesPopup.String{i}=num2str(str2double(handles.trackedNodesPopup.String{i})-1);
                    end
                end
            end
            obj.trackedNodes(obj.trackedNodes == id) =[];
            cla();
            obj.plotEverything();
        end
        
        function addTrack(obj,id)
            %add tracked node
            handles=obj.getHandles();
            obj.trackedNodes(end+1)=id;
            if isempty(handles.trackedNodesPopup.String) || strcmp(handles.trackedNodesPopup.String{1},'-')
                handles.trackedNodesPopup.String{1}=num2str(id);
            else
                handles.trackedNodesPopup.String{end+1}=num2str(id);
            end
        end
        
        
        function fillRuns(obj)
            %fill number of runs
            handles=obj.getHandles();
            switch(obj.mode)
                case 1
                    %kinematics
                    handles.runStatic.Visible='off';
                    handles.runPopup.Visible='off';
                case 2
                    %load analysis
                    handles.runStatic.Visible='on';
                    handles.runPopup.Visible='on';
                    runs=obj.sessionLoadAnalysis(obj.currentSession).runNumber-1;
                    string=1:runs;
                    string=cellstr(num2str(string'));
                    handles.runPopup.String=string;
                case 3
                    %distance analysis
                    handles.runStatic.Visible='on';
                    handles.runPopup.Visible='on';
                    runs=obj.sessionDistanceAnalysis(obj.currentSession).runNumber-1;
                    string=1:runs;
                    string=cellstr(num2str(string'));
                    handles.runPopup.String=string;
                case 4
                    %ma analysis
                    handles.runStatic.Visible='on';
                    handles.runPopup.Visible='on';
                    runs=obj.sessionAdvantageAnalysis(obj.currentSession).runNumber-1;
                    string=1:runs;
                    string=cellstr(num2str(string'));
                    handles.runPopup.String=string;
                case 5
                    %bistable analysis
                    handles.runStatic.Visible='on';
                    handles.runPopup.Visible='on';
                    runs=obj.sessionBistable(obj.currentSession).runNumber-1;
                    string=1:runs;
                    string=cellstr(num2str(string'));
                    handles.runPopup.String=string;
            end
        end
        
        function fillAnimationTab(obj)
            %fill the animation tab
            handles=obj.getHandles();
            obj.currentFrame=1;
            obj.powerList=[];
            obj.magnitudeList=[];
            obj.inputLoad=[];
            obj.distance=[];
            obj.loadList=[];
            obj.energyList=[];
            obj.originalNodes=[];
            switch(obj.mode)
                case 1
                    %kinematics
                    %static fields
                    animationLength=length(obj.sessionKinematics(obj.currentSession).stateList);
                    obj.currentWorkSpace=obj.sessionKinematics(obj.currentSession).workspace;
                    obj.stateList=obj.sessionKinematics(obj.currentSession).stateList;
                    obj.nodes=obj.sessionKinematics(obj.currentSession).nodes;
                    obj.initialNodeNumber=length(obj.nodes);
                    obj.allBeams=obj.sessionKinematics(obj.currentSession).allBeams;
                    obj.forces=Force.empty;
                    obj.moments=Moment.empty;
                    obj.torsionSprings=TorsionSpring.empty;
                case 2
                    %load analysis
                    animationLength=size(obj.sessionLoadAnalysis(obj.currentSession).run(obj.currentRun).state,1);
                    obj.currentWorkSpace=obj.sessionLoadAnalysis(obj.currentSession).run(obj.currentRun).workspace;
                    obj.stateList=mat2cell(obj.sessionLoadAnalysis(obj.currentSession).run(obj.currentRun).state,ones(1,size(obj.sessionLoadAnalysis(obj.currentSession).run(obj.currentRun).state,1)));
                    obj.nodes=obj.sessionLoadAnalysis(obj.currentSession).static.kinematic.nodes;
                    obj.initialNodeNumber=length(obj.nodes);
                    obj.allBeams=obj.sessionLoadAnalysis(obj.currentSession).static.kinematic.allBeams;
                    obj.forces=obj.sessionLoadAnalysis(obj.currentSession).run(obj.currentRun).forces;
                    obj.moments=obj.sessionLoadAnalysis(obj.currentSession).run(obj.currentRun).moments;
                    obj.torsionSprings=obj.sessionLoadAnalysis(obj.currentSession).run(obj.currentRun).torsionSprings;
                    obj.powerList=obj.sessionLoadAnalysis(obj.currentSession).run(obj.currentRun).power;
                case 3
                    %distance analysis
                    animationLength=size(obj.sessionDistanceAnalysis(obj.currentSession).run(obj.currentRun).state,1);
                    obj.currentWorkSpace=obj.sessionDistanceAnalysis(obj.currentSession).run(obj.currentRun).workspace;
                    obj.stateList=mat2cell(obj.sessionDistanceAnalysis(obj.currentSession).run(obj.currentRun).state,ones(1,size(obj.sessionDistanceAnalysis(obj.currentSession).run(obj.currentRun).state,1)));
                    obj.nodes=obj.sessionDistanceAnalysis(obj.currentSession).static.kinematic.nodes;
                    obj.initialNodeNumber=length(obj.nodes);
                    obj.allBeams=obj.sessionDistanceAnalysis(obj.currentSession).static.kinematic.allBeams;
                    obj.forces=obj.sessionDistanceAnalysis(obj.currentSession).run(obj.currentRun).forces;
                    obj.moments=obj.sessionDistanceAnalysis(obj.currentSession).run(obj.currentRun).moments;
                    obj.torsionSprings=obj.sessionDistanceAnalysis(obj.currentSession).run(obj.currentRun).torsionSprings;
                    obj.magnitudeList=obj.sessionDistanceAnalysis(obj.currentSession).run(obj.currentRun).magnitudeList;
                    obj.distance=obj.sessionDistanceAnalysis(obj.currentSession).run(obj.currentRun).distance;
                    obj.inputLoad=obj.sessionDistanceAnalysis(obj.currentSession).run(obj.currentRun).inputLoad;
                    obj.powerList=ones(animationLength,1)*100;
                    obj.originalNodes=obj.nodes;
                case 4
                    %advantage analysis
                    animationLength=size(obj.sessionAdvantageAnalysis(obj.currentSession).run(obj.currentRun).state,1);
                    obj.currentWorkSpace=obj.sessionAdvantageAnalysis(obj.currentSession).run(obj.currentRun).workspace;
                    obj.stateList=mat2cell(obj.sessionAdvantageAnalysis(obj.currentSession).run(obj.currentRun).state,ones(1,size(obj.sessionAdvantageAnalysis(obj.currentSession).run(obj.currentRun).state,1)));
                    obj.nodes=obj.sessionAdvantageAnalysis(obj.currentSession).static.kinematic.nodes;
                    obj.initialNodeNumber=length(obj.nodes);
                    obj.allBeams=obj.sessionAdvantageAnalysis(obj.currentSession).static.kinematic.allBeams;
                    obj.forces=obj.sessionAdvantageAnalysis(obj.currentSession).run(obj.currentRun).forces;
                    obj.moments=obj.sessionAdvantageAnalysis(obj.currentSession).run(obj.currentRun).moments;
                    obj.torsionSprings=obj.sessionAdvantageAnalysis(obj.currentSession).run(obj.currentRun).torsionSprings;
                    obj.powerList=obj.sessionAdvantageAnalysis(obj.currentSession).run(obj.currentRun).power;
                case 5
                    %energy analysis
                    animationLength=size(obj.sessionBistable(obj.currentSession).run(obj.currentRun).state,1);
                    obj.currentWorkSpace=obj.sessionBistable(obj.currentSession).run(obj.currentRun).workspace;
                    obj.stateList=mat2cell(obj.sessionBistable(obj.currentSession).run(obj.currentRun).state,ones(1,size(obj.sessionBistable(obj.currentSession).run(obj.currentRun).state,1)));
                    obj.nodes=obj.sessionBistable(obj.currentSession).static.kinematic.nodes;
                    obj.initialNodeNumber=length(obj.nodes);
                    obj.allBeams=obj.sessionBistable(obj.currentSession).static.kinematic.allBeams;
                    obj.forces=obj.sessionBistable(obj.currentSession).run(obj.currentRun).forces;
                    obj.moments=obj.sessionBistable(obj.currentSession).run(obj.currentRun).moments;
                    obj.torsionSprings=obj.sessionBistable(obj.currentSession).run(obj.currentRun).torsionSprings;
                    obj.powerList=obj.sessionBistable(obj.currentSession).run(obj.currentRun).power;
                    obj.loadList=obj.sessionBistable(obj.currentSession).run(obj.currentRun).load;
                    obj.energyList=obj.sessionBistable(obj.currentSession).run(obj.currentRun).energy;
            end
            handles.totalFrameStatic2.String=num2str(animationLength);
            handles.endFrameStatic2.String=num2str(animationLength);
            handles.currentFrameStatic2.String='1';
            handles.startFrameStatic2.String='1';
            %sliders
            if animationLength < 2
                handles.animationPanel.Visible='off';
            else
                handles.animationPanel.Visible='on';
            end
            handles.slider1.Min=1;handles.slider2.Min=1;
            handles.slider1.Max=animationLength;handles.slider2.Max=animationLength;
            if animationLength == 0
                handles.slider1.SliderStep=[1 1];handles.slider2.SliderStep=[1 1];
            else
                handles.slider1.SliderStep=[1/animationLength 10/animationLength];handles.slider2.SliderStep=[1/animationLength 10/animationLength];
            end
            handles.slider1.Value=1;handles.slider2.Value=animationLength;
            obj.currentFrame=1;
            %get number of nodes
            number=length(obj.nodes);
            string=1:number;
            string=cellstr(num2str(string'));
            handles.nodePopup.String=string;
            handles.percentText.String='50';
            handles.node1Text.String='1';
            handles.node2Text.String=num2str(number);
            handles=obj.setUnits(handles);
            obj.deleteAll();
            %delete axis
            handles.xAxisStatic2.String='-';
            handles.yAxisList.String=[];
            obj.yInput=1;
            handles.showNodes.Value=1;
            %delete tracked nodes
            obj.trackedNodes=[];
            handles.trackedNodesPopup.Value=1;
            handles.trackedNodesPopup.String={'-'};
            %plotting
            handles.xAxisStatic2.String='Frames';
            obj.xAxis.mode=-1;obj.xAxis.id=-1;
            guidata(handles.mainGUI,handles);
        end
        
        function setForceRightClick(obj,source,callbackdata,id,mode1,mode2)
            %right click for moment
            handles=obj.getHandles();
            lowerLimit=str2double(handles.startFrameStatic2.String);
            upperLimit=str2double(handles.endFrameStatic2.String);
            switch(obj.mode)
                case 1
                    %kinematics
                    string=['Kinematics-' num2str(obj.currentSession) ':Force ' num2str(id)];
                case 2
                    %load analysis
                    string=['Load Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Force ' num2str(id)];
                case 3
                    %distance analysis
                    string=['Distance Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Force ' num2str(id)];
                case 4
                    %mechanical advantage
                    string=['Mechanical Advantage-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Force ' num2str(id)];
                case 5
                    %energy plot
                    string=['Energy Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Force ' num2str(id)];
            end
            switch mode1
                case 1
                    string=[string ,'(Magnitude)'];
                case 2
                    string=[string ,'(X Component)'];
                case 3
                    string=[string ,'(Y Component)'];
            end
            if mode2 ==1
                %x axis
                obj.xAxis=struct('mode',obj.mode,'type',[4 mode1],'frames',[min([lowerLimit upperLimit]) max([lowerLimit upperLimit])],'id',id,'session',obj.currentSession);
                handles.xAxisStatic2.String=string;
            else
                %y axis
                obj.yAxis(obj.yInput)=struct('mode',obj.mode,'type',[4 mode1],'frames',[min([lowerLimit upperLimit]) max([lowerLimit upperLimit])],'id',id,'session',obj.currentSession);
                handles.yAxisList.String{end+1}=string;
                obj.yInput=obj.yInput+1;
            end
            
        end
        
        
        function setMomentRightClick(obj,source,callbackdata,id,mode1,mode2)
            %right click for moment
            handles=obj.getHandles();
            lowerLimit=str2double(handles.startFrameStatic2.String);
            upperLimit=str2double(handles.endFrameStatic2.String);
            switch(obj.mode)
                case 1
                    %kinematics
                    string=['Kinematics-' num2str(obj.currentSession) ':Moment ' num2str(id)];
                case 2
                    %load analysis
                    string=['Load Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Moment ' num2str(id)];
                case 3
                    %distance analysis
                    string=['Distance Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Moment ' num2str(id)];
                case 4
                    %mechanical advantage
                    string=['Mechanical Advantage-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Moment ' num2str(id)];
                case 5
                    %energy plot
                    string=['Energy Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Moment ' num2str(id)];
            end
            string=[string ,'(Magnitude)'];
            if mode2 ==1
                %x axis
                obj.xAxis=struct('mode',obj.mode,'type',[3 1],'frames',[min([lowerLimit upperLimit]) max([lowerLimit upperLimit])],'id',id,'session',obj.currentSession);
                handles.xAxisStatic2.String=string;
            else
                %y axis
                obj.yAxis(obj.yInput)=struct('mode',obj.mode,'type',[3 1],'frames',[min([lowerLimit upperLimit]) max([lowerLimit upperLimit])],'id',id,'session',obj.currentSession);
                handles.yAxisList.String{end+1}=string;
                obj.yInput=obj.yInput+1;
            end
            
        end
        
        function setNodeXAxis(obj,~,~,id,mode)
            %for right click on a node
            handles=obj.getHandles();
            lowerLimit=str2double(handles.startFrameStatic2.String);
            upperLimit=str2double(handles.endFrameStatic2.String);
            obj.xAxis=struct('mode',obj.mode,'type',[1 mode],'frames',[min([lowerLimit upperLimit]) max([lowerLimit upperLimit])],'id',id,'session',obj.currentSession);
            switch(obj.mode)
                case 1
                    %kinematics
                    string=['Kinematics-' num2str(obj.currentSession) ':Node ' num2str(id)];
                case 2
                    %load analysis
                    string=['Load Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Node ' num2str(id)];
                case 3
                    %distance analysis
                    string=['Distance Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Node ' num2str(id)];
                case 4
                    %mechanical advantage
                    string=['Mechanical Advantage-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Node ' num2str(id)];
                case 5
                    %energy plot
                    string=['Energy Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Node ' num2str(id)];
            end
            if mode == 1
                string=[string ,'(X)'];
            else
                string=[string ,'(Y)'];
            end
            handles.xAxisStatic2.String=string;
        end
        
        function setNodeYAxis(obj,~,~,id,mode)
            %for right click on a node
            handles=obj.getHandles();
            lowerLimit=str2double(handles.startFrameStatic2.String);
            upperLimit=str2double(handles.endFrameStatic2.String);
            obj.yAxis(obj.yInput)=struct('mode',obj.mode,'type',[1 mode],'frames',[min([lowerLimit upperLimit]) max([lowerLimit upperLimit])],'id',id,'session',obj.currentSession);
            obj.yInput=obj.yInput+1;
            switch(obj.mode)
                case 1
                    %kinematics
                    string=['Kinematics-' num2str(obj.currentSession) ':Node ' num2str(id)];
                case 2
                    %load analysis
                    string=['Load Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Node ' num2str(id)];
                case 3
                    %distance analysis
                    string=['Distance Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Node ' num2str(id)];
                case 4
                    %mechanical advantage
                    string=['Mechanical Advantage-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Node ' num2str(id)];
                case 5
                    %energy plot
                    string=['Energy Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Node ' num2str(id)];
            end
            if mode == 1
                string=[string ,'(X)'];
            else
                string=[string ,'(Y)'];
            end
            handles.yAxisList.String{end+1}=string;
        end
        
        function pos=getLinkPosFromID(obj,id)
            %get position of the rigid link from id
            pos=0;
            for i=1:length(obj.allBeams)
                if obj.allBeams{i}.id == id
                    pos=i;
                    return;
                end
            end
        end
        
        function setLinkXAxis(obj,~,~,id,mode)
            %for right click on a node
            handles=obj.getHandles();
            id=obj.getLinkPosFromID(id);
            lowerLimit=str2double(handles.startFrameStatic2.String);
            upperLimit=str2double(handles.endFrameStatic2.String);
            obj.xAxis=struct('mode',obj.mode,'type',[2 mode],'frames',[min([lowerLimit upperLimit]) max([lowerLimit upperLimit])],'id',id,'session',obj.currentSession);
            switch(obj.mode)
                case 1
                    %kinematics
                    string=['Kinematics-' num2str(obj.currentSession) ':Link ' num2str(obj.allBeams{id}.nodes(1,1)) '-'  num2str(obj.allBeams{id}.nodes(1,2))];
                case 2
                    %load analysis
                    string=['Load Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Link ' num2str(obj.allBeams{id}.nodes(1,1)) '-'  num2str(obj.allBeams{id}.nodes(1,2))];
                case 3
                    %distance analysis
                    string=['Distance Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Link ' num2str(obj.allBeams{id}.nodes(1,1)) '-'  num2str(obj.allBeams{id}.nodes(1,2))];
                case 4
                    %mechanical advantage
                    string=['Mechanical Advantage-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Link ' num2str(obj.allBeams{id}.nodes(1,1)) '-'  num2str(obj.allBeams{id}.nodes(1,2))];
                case 5
                    %energy plot
                    string=['Energy Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Link ' num2str(obj.allBeams{id}.nodes(1,1)) '-'  num2str(obj.allBeams{id}.nodes(1,2))];
            end
            lengthString=LengthUnit.getString(obj.currentWorkSpace.unitLength);
            loadString=ForceUnit.getString(obj.currentWorkSpace.unitForce);
            switch mode
                case 1
                    string=[string ,'(Angle°)'];
                case 2
                    string=[string ,'(Length)'];
                case 3
                    string=[string ,'(Tip Fx(',loadString,')',')'];
                case 4
                    string=[string ,'(Tip Fy(',loadString,')',')'];
                case 5
                    string=[string ,'(Tip M(',loadString,'.',lengthString,')',')'];
            end
            handles.xAxisStatic2.String=string;
        end
        
        function setLinkYAxis(obj,~,~,id,mode)
            %for right click on a node
            handles=obj.getHandles();
            id=obj.getLinkPosFromID(id);
            lowerLimit=str2double(handles.startFrameStatic2.String);
            upperLimit=str2double(handles.endFrameStatic2.String);
            obj.yAxis(obj.yInput)=struct('mode',obj.mode,'type',[2 mode],'frames',[min([lowerLimit upperLimit]) max([lowerLimit upperLimit])],'id',id,'session',obj.currentSession);
            obj.yInput=obj.yInput+1;
            switch(obj.mode)
                case 1
                    %kinematics
                    string=['Kinematics-' num2str(obj.currentSession) ':Link ' num2str(obj.allBeams{id}.nodes(1,1)) '-'  num2str(obj.allBeams{id}.nodes(1,2))];
                case 2
                    %load analysis
                    string=['Load Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Link ' num2str(obj.allBeams{id}.nodes(1,1)) '-'  num2str(obj.allBeams{id}.nodes(1,2))];
                case 3
                    %distance analysis
                    string=['Distance Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Link ' num2str(obj.allBeams{id}.nodes(1,1)) '-'  num2str(obj.allBeams{id}.nodes(1,2))];
                case 4
                    %mechanical advantage
                    string=['Mechanical Advantage-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Link ' num2str(obj.allBeams{id}.nodes(1,1)) '-'  num2str(obj.allBeams{id}.nodes(1,2))];
                case 5
                    %energy plot
                    string=['Energy Analysis-' num2str(obj.currentSession) '/'  num2str(obj.currentRun) ':Link ' num2str(obj.allBeams{id}.nodes(1,1)) '-'  num2str(obj.allBeams{id}.nodes(1,2))];
            end
            lengthString=LengthUnit.getString(obj.currentWorkSpace.unitLength);
            loadString=ForceUnit.getString(obj.currentWorkSpace.unitForce);
            switch mode
                case 1
                    string=[string ,'(Angle°)'];
                case 2
                    string=[string ,'(Length)'];
                case 3
                    string=[string ,'(Tip Fx(',loadString,')',')'];
                case 4
                    string=[string ,'(Tip Fy(',loadString,')',')'];
                case 5
                    string=[string ,'(Tip M(',loadString,'.',lengthString,')',')'];
            end
            handles.yAxisList.String{end+1}=string;
        end
        
        
        function setBistableAxis(obj,id)
            %addd energy or load to axises
            handles=obj.getHandles();
            lowerLimit=str2double(handles.startFrameStatic2.String);
            upperLimit=str2double(handles.endFrameStatic2.String);
            switch id
                case 3
                    %energy x
                    obj.xAxis=struct('mode',obj.mode,'type',5,'frames',[min([lowerLimit upperLimit]) max([lowerLimit upperLimit])],'id',id,'session',obj.currentSession);
                    string='Total Potential Energy';
                    handles.xAxisStatic2.String=string;
                case 4
                    %energy y
                    obj.yAxis(obj.yInput)=struct('mode',obj.mode,'type',5,'frames',[min([lowerLimit upperLimit]) max([lowerLimit upperLimit])],'id',id,'session',obj.currentSession);
                    obj.yInput=obj.yInput+1;
                    string='Total Potential Energy';
                    handles.yAxisList.String{end+1}=string;
                case 5
                    %load x
                    obj.xAxis=struct('mode',obj.mode,'type',6,'frames',[min([lowerLimit upperLimit]) max([lowerLimit upperLimit])],'id',id,'session',obj.currentSession);
                    string='Driving Load';
                    handles.xAxisStatic2.String=string;
                case 6
                    %load y
                    obj.yAxis(obj.yInput)=struct('mode',obj.mode,'type',6,'frames',[min([lowerLimit upperLimit]) max([lowerLimit upperLimit])],'id',id,'session',obj.currentSession);
                    obj.yInput=obj.yInput+1;
                    string='Driving Load';
                    handles.yAxisList.String{end+1}=string;
            end
        end
        
        function createNewPlot(obj)
            %create a new plot
            if logical(obj.xAxis.id) && ~isempty(obj.yAxis) && logical(obj.yAxis(1).id)
                handles=obj.getHandles();
                obj.figures(end+1)=figure('Name',['DAS 2D: Figure ' num2str(length(obj.figures))],'NumberTitle','off');
                hold on;
                if obj.xAxis.mode ~= -1
                    xData=zeros(1,obj.xAxis.frames(2)-obj.xAxis.frames(1)+1);
                    startFrame=obj.xAxis.frames(1);
                end
                id=obj.xAxis.id;
                if  obj.xAxis.mode == -1
                    %frames
                    xData=[];
                end
                %x input
                if ~isempty(xData)
                    switch (obj.xAxis.type(1))
                        case 1
                            %node
                            nodeList=obj.getNodeList(startFrame,startFrame+length(xData)-1);
                            if obj.xAxis.type(2) == 1
                                %x coordinate
                                for i=1:length(xData)
                                    xData(i)=nodeList(i,2*id-1);
                                end
                            else
                                %y coordinate
                                for i=1:length(xData)
                                    xData(i)=nodeList(i,2*id);
                                end
                            end
                        case 2
                            %link
                            switch obj.xAxis.type(2)
                                case 1
                                    %angle
                                    xData=obj.getAngleList(startFrame,startFrame+length(xData)-1,id);         
                                case 2
                                    %length
                                    xData=obj.getLengthList(startFrame,startFrame+length(xData)-1,id);  
                                case {3,4,5}
                                    %fx,fy,m
                                    xData=obj.getForceList(startFrame,startFrame+length(xData)-1,id,obj.xAxis.type(2)-2,obj.currentWorkSpace);  
                            end
                        case 3
                            %moment
                            if isempty(obj.powerList)
                                powerList=ones(length(xData),1);
                            else
                                powerList=obj.powerList(startFrame:startFrame+length(xData)-1)'/100;
                            end
                            if isempty(obj.magnitudeList)
                                magnitudeList=ones(length(xData),1)*obj.moments(id).magnitude; 
                            else
                                magnitudeList=obj.magnitudeList(startFrame:startFrame+length(xData)-1); 
                            end
                            xData=powerList.*magnitudeList;
                        case 4
                            %force
                            if isempty(obj.powerList)
                                powerList=ones(length(xData),1);
                            else
                                powerList=obj.powerList(startFrame:startFrame+length(xData)-1)'/100;
                            end
                            switch(obj.xAxis.type(2))
                                case 1
                                    %magnitude-follower
                                    if isempty(obj.magnitudeList)
                                        magnitudeList=ones(length(xData),1)*obj.forces(id).magnitude;
                                    else
                                        magnitudeList=obj.magnitudeList(startFrame:startFrame+length(xData)-1);
                                    end
                                case 2
                                    %magnitude-follower
                                    if isempty(obj.magnitudeList)
                                        magnitudeList=ones(length(xData),1)*obj.forces(id).xValue;
                                    else
                                        magnitudeList=obj.magnitudeList(startFrame:startFrame+length(xData)-1,1);
                                    end
                                case 3
                                    %magnitude-follower
                                    if isempty(obj.magnitudeList)
                                        magnitudeList=ones(length(xData),1)*obj.forces(id).yValue;
                                    else
                                        magnitudeList=obj.magnitudeList(startFrame:startFrame+length(xData)-1,2);
                                    end
                            end
                            xData=powerList.*magnitudeList;
                        case 5
                            %energy
                            xData=obj.energyList;
                        case 6
                            %load
                            xData=obj.loadList;
                    end
                end
                %y input
                for j=1:obj.yInput-1
                    yData=zeros(1,obj.yAxis(j).frames(2)-obj.yAxis(j).frames(1)+1);
                    startFrame=obj.yAxis(j).frames(1);
                    id=obj.yAxis(j).id;
                    switch (obj.yAxis(j).type(1))
                        case 1
                            %node
                            nodeList=obj.getNodeList(startFrame,startFrame+length(yData)-1); 
                            if obj.yAxis(j).type(2) == 1
                                %x coordinate
                                for i=1:length(yData)
                                    yData(i)=nodeList(i,2*id-1);
                                end
                            else
                                %y coordinate
                                for i=1:length(yData)
                                    yData(i)=nodeList(i,2*id);
                                end
                            end
                        case  2
                            %link
                            switch obj.yAxis(j).type(2)
                                case 1
                                    %angle
                                    yData=obj.getAngleList(startFrame,startFrame+length(yData)-1,id);         
                                case 2
                                    %length
                                    yData=obj.getLengthList(startFrame,startFrame+length(yData)-1,id);  
                                case {3,4,5}
                                    %fx,fy,m
                                    yData=obj.getForceList(startFrame,startFrame+length(yData)-1,id,obj.yAxis(j).type(2)-2,obj.currentWorkSpace);  
                            end
                        case 3
                            %moment
                            if isempty(obj.powerList)
                                powerList=ones(length(yData),1);
                            else
                                powerList=obj.powerList(startFrame:startFrame+length(yData)-1)'/100;
                            end
                            if isempty(obj.magnitudeList)
                                magnitudeList=ones(length(yData),1)*obj.moments(id).magnitude;
                            else
                                magnitudeList=obj.magnitudeList(startFrame:startFrame+length(yData)-1);
                            end
                            yData=powerList.*magnitudeList;
                        case 4
                            %force
                            if isempty(obj.powerList)
                                powerList=ones(1,length(yData));
                            else
                                powerList=obj.powerList(startFrame:startFrame+length(yData)-1)'/100;
                            end
                            switch(obj.yAxis(j).type(2))
                                case 1
                                    %magnitude-follower
                                    if isempty(obj.magnitudeList)
                                        magnitudeList=ones(length(yData),1)*obj.forces(id).magnitude;
                                    else
                                        magnitudeList=obj.magnitudeList(startFrame:startFrame+length(yData)-1);
                                    end
                                case 2
                                    %magnitude-follower
                                    if isempty(obj.magnitudeList)
                                        magnitudeList=ones(length(yData),1)*obj.forces(id).xValue;
                                    else
                                        magnitudeList=obj.magnitudeList(startFrame:startFrame+length(yData)-1,1);
                                    end
                                case 3
                                    %magnitude-follower
                                    if isempty(obj.magnitudeList)
                                        magnitudeList=ones(length(yData),1)*obj.forces(id).yValue;
                                    else
                                        magnitudeList=obj.magnitudeList(startFrame:startFrame+length(yData)-1,2);
                                    end
                            end
                            yData=powerList.*magnitudeList;
                        case 5
                            %energy
                            yData=obj.energyList;
                        case 6
                            %load
                            yData=obj.loadList;
                    end
                    
                    if isempty(xData)
                        plot(1:length(yData),yData);
                        if logical(handles.showCheckBox.Value)
                            %save the data
                            data{1,1}='Frames';
                            name=cellstr(handles.yAxisList.String);
                            data{1,end+1}=char(name(j));
                            for i=1:length(yData)
                                data{1+i,1}=i;
                                data{1+i,end}=yData(i);
                            end
                        end
                    else
                        plot(xData(1:min([length(xData) length(yData)])),yData(1:min([length(xData) length(yData)])));
                        if logical(handles.showCheckBox.Value)
                            %save the data
                            data{1,1}=char(handles.xAxisStatic2.String);
                            name=cellstr(handles.yAxisList.String);
                            data{1,end+1}=char(name(j));
                            for i=1:min([length(xData) length(yData)])
                                data{1+i,1}=xData(i);
                                data{1+i,end}=yData(i);
                            end
                        end
                    end
                    if isempty(xData)
                        xlabel('Frames');
                    else
                        xlabel(handles.xAxisStatic2.String);
                    end
                    legend(cellstr(handles.yAxisList.String));
                end
                if logical(handles.showCheckBox.Value)
                    dataOutput('data',data,'directory',obj.directory);
                end
            end
        end
        
        
        function changeFrame(obj)
            %nodes
            switch obj.mode
                case  1
                    %kinematics
                    for i=1:length(obj.allBeams)
                        obj.allBeams{i}=obj.allBeams{i}.updateBeam(obj.stateList{obj.currentFrame},obj.stateList{obj.currentFrame});
                    end
                    for i=1:length(obj.nodes)
                        %plot nodes
                        obj.nodes(i)=obj.nodes(i).deleteNodeDrawing();
                    end
                    for i=1:length(obj.sessionKinematics(obj.currentSession).allBeams)
                        obj.sessionKinematics(obj.currentSession).allBeams{i}=obj.sessionKinematics(obj.currentSession).allBeams{i}.updateBeam(obj.stateList{obj.currentFrame},obj.stateList{obj.currentFrame});
                    end
                    obj.nodes = obj.sessionKinematics(obj.currentSession).updateNodes(obj.stateList{obj.currentFrame});
                case  2
                    %load analysis
                    for i=1:length(obj.allBeams)
                        obj.allBeams{i}=obj.allBeams{i}.updateBeam(obj.stateList{obj.currentFrame},obj.stateList{obj.currentFrame});
                    end
                    for i=1:length(obj.nodes)
                        %plot nodes
                        obj.nodes(i)=obj.nodes(i).deleteNodeDrawing();
                    end
                    for i=1:length(obj.sessionLoadAnalysis(obj.currentSession).static.kinematic.allBeams)
                        obj.sessionLoadAnalysis(obj.currentSession).static.kinematic.allBeams{i}=obj.sessionLoadAnalysis(obj.currentSession).static.kinematic.allBeams{i}.updateBeam(obj.stateList{obj.currentFrame},obj.stateList{obj.currentFrame});
                    end
                    obj.nodes = obj.sessionLoadAnalysis(obj.currentSession).static.kinematic.updateNodes(obj.stateList{obj.currentFrame});    
                case 3
                    %distance analysis
                    if  obj.inputLoad(1,1) == 1
                        %force type
                        if logical(obj.forces(obj.inputLoad(1,2)).follower)
                            %follower
                            obj.forces(obj.inputLoad(1,2)).magnitude=obj.magnitudeList(obj.currentFrame,1);
                        else
                            obj.forces(obj.inputLoad(1,2)).xValue=obj.magnitudeList(obj.currentFrame,1);
                            obj.forces(obj.inputLoad(1,2)).yValue=obj.magnitudeList(obj.currentFrame,2);
                        end
                    else
                        %moment type
                        obj.moments(obj.inputLoad(1,2)).magnitude=obj.magnitudeList(obj.currentFrame,1);
                    end
                    
                    for i=1:length(obj.allBeams)
                        obj.allBeams{i}=obj.allBeams{i}.updateBeam(obj.stateList{obj.currentFrame},obj.stateList{obj.currentFrame});
                    end
                    for i=1:length(obj.nodes)
                        %plot nodes
                        obj.nodes(i)=obj.nodes(i).deleteNodeDrawing();
                    end
                    for i=1:length(obj.sessionDistanceAnalysis(obj.currentSession).static.kinematic.allBeams)
                        obj.sessionDistanceAnalysis(obj.currentSession).static.kinematic.allBeams{i}=obj.sessionDistanceAnalysis(obj.currentSession).static.kinematic.allBeams{i}.updateBeam(obj.stateList{obj.currentFrame},obj.stateList{obj.currentFrame});
                    end
                    obj.nodes = obj.sessionDistanceAnalysis(obj.currentSession).static.kinematic.updateNodes(obj.stateList{obj.currentFrame});
                case 4
                    %ma
                    for i=1:length(obj.allBeams)
                        obj.allBeams{i}=obj.allBeams{i}.updateBeam(obj.stateList{obj.currentFrame},obj.stateList{obj.currentFrame});
                    end
                    for i=1:length(obj.nodes)
                        %plot nodes
                        obj.nodes(i)=obj.nodes(i).deleteNodeDrawing();
                    end
                    obj.nodes = obj.sessionAdvantageAnalysis(obj.currentSession).static.kinematic.updateNodes(obj.stateList{obj.currentFrame});
                case 5
                    %energy
                    for i=1:length(obj.allBeams)
                        obj.allBeams{i}=obj.allBeams{i}.updateBeam(obj.stateList{obj.currentFrame},obj.stateList{obj.currentFrame});
                    end
                    for i=1:length(obj.nodes)
                        %plot nodes
                        obj.nodes(i)=obj.nodes(i).deleteNodeDrawing();
                    end
                    obj.nodes = obj.sessionBistable(obj.currentSession).static.kinematic.updateNodes(obj.stateList{obj.currentFrame});                    
            end
        end
        
        function drawDistance(obj)
            %draw the distance
            handles=obj.getHandles();
            limit=obj.currentWorkSpace.getLimit()*0.5;
            mainFig=handles.designPlot;
            target=obj.distance.target(1,2);
            if obj.distance.type == 1
                %node type
                node=obj.originalNodes(obj.distance.member).getNode();
                if obj.distance.target(1) == 1
                    %x motion
                    obj.distance.line=plot(mainFig,target+[node.x node.x],node.y+[limit*0.2 -limit*0.2],'LineStyle','--','Color',Colors.NormalLink.getColor());
                else
                    %y motion
                    obj.distance.line=plot(mainFig,node.x+[limit*0.2 -limit*0.2],target+[node.y node.y],'LineStyle','--','Color',Colors.NormalLink.getColor());
                end
            else
                %link type
                link=obj.allBeams{obj.distance.member};
                linkLength=link.length0;
                linkAngle=link.theta0;
                node=obj.originalNodes(link.nodes(1,1)).getNode();
                if obj.distance.target(1,1) == 1
                    %CW motion
                    newAngle=linkAngle-obj.distance.target(1,2)*pi/180;
                else
                    %CCW motion
                    newAngle=linkAngle+obj.distance.target(1,2)*pi/180;
                end
                obj.distance.line=plot(mainFig,[node.x node.x+linkLength*cos(newAngle) ],[node.y node.y+linkLength*sin(newAngle) ],'LineStyle','--','Color',Colors.NormalLink.getColor());
            end
        end
        
        function plotEverything(obj)
            %plot everything
            handles=obj.getHandles();
            %resize the axis area
            hold(handles.designPlot,'on');
            axis(handles.designPlot,[-obj.currentWorkSpace.getX(),obj.currentWorkSpace.getX(),-obj.currentWorkSpace.getY(),obj.currentWorkSpace.getY()]);
            limit=obj.currentWorkSpace.getLimit()*0.5;
            obj.changeFrame();
            %draw all links
            for i=1:length(obj.allBeams)
                %plot links
                obj.allBeams{i}=obj.allBeams{i}.drawLink(handles.designPlot,obj.nodes,limit,Module.PostProcessing,obj);
            end
            %draw all nodes
            if logical(handles.showNodes.Value)
                for i=1:length(obj.nodes)
                    %plot nodes
                    obj.nodes(i)=obj.nodes(i).drawNode(handles.designPlot,limit,Module.PostProcessing,obj);
                end
            else
                for i=1:length(obj.nodes)
                    %plot nodes
                    obj.nodes(i)=obj.nodes(i).deleteNodeDrawing();
                end
            end
            %draw forces
            for i=1:length(obj.forces)
                obj.forces(i)=obj.forces(i).drawStatic(obj,obj.nodes,obj.allBeams,obj.powerList(obj.currentFrame));
            end
            %draw moments
            for i=1:length(obj.moments)
                obj.moments(i)=obj.moments(i).drawStatic(obj,obj.nodes,obj.allBeams,obj.powerList(obj.currentFrame));
            end
            %draw torsionSprings
            for i=1:length(obj.torsionSprings)
                obj.torsionSprings(i)=obj.torsionSprings(i).drawStatic(obj,obj.nodes,obj.allBeams);
            end
            %draw distance
            if ~isempty(obj.distance)
                obj.drawDistance();
            end
        end
        
        function nodeList=getNodeList(obj,lowerRange,upperRange)
            %find the tracked nodes
            switch obj.mode
                case  1
                    %kinematics
                    nodeList=zeros(upperRange-lowerRange+1,length(obj.nodes)*2);
                    for j=lowerRange:upperRange
                        for i=1:length(obj.allBeams)
                            obj.allBeams{i}=obj.allBeams{i}.updateBeam(obj.stateList{j},obj.stateList{j});
                        end
                        for i=1:length(obj.sessionKinematics(obj.currentSession).allBeams)
                            obj.sessionKinematics(obj.currentSession).allBeams{i}=obj.sessionKinematics(obj.currentSession).allBeams{i}.updateBeam(obj.stateList{j},obj.stateList{j});
                        end
                        nodeList(j-lowerRange+1,:) = obj.sessionKinematics(obj.currentSession).addAllNodes(obj.sessionKinematics(obj.currentSession).updateNodes(obj.stateList{j}));
                    end
                 case  2
                    %load analysis
                    nodeList=zeros(upperRange-lowerRange+1,length(obj.nodes)*2);
                    for j=lowerRange:upperRange
                        for i=1:length(obj.allBeams)
                            obj.allBeams{i}=obj.allBeams{i}.updateBeam(obj.stateList{j},obj.stateList{j});
                        end
                        for i=1:length(obj.sessionLoadAnalysis(obj.currentSession).static.kinematic.allBeams)
                            obj.sessionLoadAnalysis(obj.currentSession).static.kinematic.allBeams{i}=obj.sessionLoadAnalysis(obj.currentSession).static.kinematic.allBeams{i}.updateBeam(obj.stateList{j},obj.stateList{j});
                        end
                        nodeList(j-lowerRange+1,:) = obj.sessionLoadAnalysis(obj.currentSession).static.kinematic.addAllNodes(obj.sessionLoadAnalysis(obj.currentSession).static.kinematic.updateNodes(obj.stateList{j}));
                    end  
                case 3
                    %distance analysis
                    nodeList=zeros(upperRange-lowerRange+1,length(obj.nodes)*2);
                    for j=lowerRange:upperRange
                        for i=1:length(obj.allBeams)
                            obj.allBeams{i}=obj.allBeams{i}.updateBeam(obj.stateList{j},obj.stateList{j});
                        end
                        for i=1:length(obj.sessionLoadAnalysis(obj.currentSession).static.kinematic.allBeams)
                            obj.sessionDistanceAnalysis(obj.currentSession).static.kinematic.allBeams{i}=obj.sessionDistanceAnalysis(obj.currentSession).static.kinematic.allBeams{i}.updateBeam(obj.stateList{j},obj.stateList{j});
                        end
                        nodeList(j-lowerRange+1,:) = obj.sessionDistanceAnalysis(obj.currentSession).static.kinematic.addAllNodes(obj.sessionDistanceAnalysis(obj.currentSession).static.kinematic.updateNodes(obj.stateList{j}));
                    end 
                case 4
                    %ma
                    nodeList=zeros(upperRange-lowerRange+1,length(obj.nodes)*2);
                    for j=lowerRange:upperRange
                        for i=1:length(obj.allBeams)
                            obj.allBeams{i}=obj.allBeams{i}.updateBeam(obj.stateList{j},obj.stateList{j});
                        end
                        nodeList(j-lowerRange+1,:) = obj.sessionAdvantageAnalysis(obj.currentSession).static.kinematic.addAllNodes(obj.sessionAdvantageAnalysis(obj.currentSession).static.kinematic.updateNodes(obj.stateList{j}));
                    end  
                case 5
                    %energy
                    nodeList=zeros(upperRange-lowerRange+1,length(obj.nodes)*2);
                    for j=lowerRange:upperRange
                        for i=1:length(obj.allBeams)
                            obj.allBeams{i}=obj.allBeams{i}.updateBeam(obj.stateList{j},obj.stateList{j});
                        end
                        nodeList(j-lowerRange+1,:) = obj.sessionBistable(obj.currentSession).static.kinematic.addAllNodes(obj.sessionBistable(obj.currentSession).static.kinematic.updateNodes(obj.stateList{j}));
                    end                       
            end
        end
        
        function angleList=getAngleList(obj,lowerRange,upperRange,id)
            %fill the angle list
            angleList=zeros(upperRange-lowerRange+1,1);
            for j=lowerRange:upperRange
                [angleList(j-lowerRange+1,1),~] = obj.allBeams{id}.getAngle(obj.stateList{j},obj.stateList{j},100);
            end
            angleList=angleList*180/pi;
        end
        
        function lengthList=getLengthList(obj,lowerRange,upperRange,id)
            %fill the length list
            lengthList=zeros(upperRange-lowerRange+1,1);
            for j=lowerRange:upperRange
                obj.allBeams{id}=obj.allBeams{id}.updateBeam(obj.stateList{j},obj.stateList{j});
                lengthList(j-lowerRange+1,1)=obj.allBeams{id}.length;
            end
        end
        
        function forceList=getForceList(obj,lowerRange,upperRange,id,type,workspace)
            %fill the force list
            forceList=zeros(upperRange-lowerRange+1,1);
            for j=lowerRange:upperRange
                obj.allBeams{id}=obj.allBeams{id}.updateBeam(obj.stateList{j},obj.stateList{j});
                tipForces=obj.allBeams{id}.getForce(workspace);
                forceList(j-lowerRange+1,1)=tipForces(1,type);
            end
        end
        
        function animate(obj)
            %animate the mechanism
            delete(obj.dummyLines);
            cla();
            handles=obj.getHandles();
            obj.busy=1;
            handles.stopButton.Visible='on';
            lowerLimit=str2double(handles.startFrameStatic2.String);
            upperLimit=str2double(handles.endFrameStatic2.String);
            if ~isnan(lowerLimit) && ~isnan(upperLimit)
                range=[min([lowerLimit upperLimit]) max([lowerLimit upperLimit])];
                nodeList=obj.getNodeList(range(1,1),range(1,2));
                for i=range(1,1):range(1,2)
                    if logical(obj.busy)
                        obj.currentFrame=i;
                        obj.plotEverything();
                        handles.currentFrameStatic2.String=num2str(i);
                        %draw tracked nodes
                        for j=1:length(obj.trackedNodes)
                            x=nodeList(1:i-range(1,1)+1,2*obj.trackedNodes(j)-1);
                            y=nodeList(1:i-range(1,1)+1,2*obj.trackedNodes(j));
                            obj.dummyLines(j)=plot(x,y,'LineStyle','none','Marker','.','Color',obj.colors(j,:));
                        end
                        drawnow;
                       % pause(1/25);
                    end
                end
            end
            handles.stopButton.Visible='off';
            obj.busy=0;
        end
        
        function workspace=getWorkspace(obj)
            %get the workspace
            workspace=obj.currentWorkSpace;
        end
        
        function mode=getMode(obj)
            %get the mode
            mode=Module.PostProcessing;
        end
        
        function handles=setUnits(obj,handles)
            %set units for the GUI
            lengthString=LengthUnit.getString(obj.currentWorkSpace.unitLength);
            loadString=ForceUnit.getString(obj.currentWorkSpace.unitForce);
            handles.forceUnit1.String=loadString;
            handles.forceUnit2.String=loadString;
            handles.momentUnit.String=[loadString '.' lengthString];
            handles.unitTorsionSpring.String=[loadString '.' lengthString '/rad'];
            handles.unitLinear.String=[loadString '/' lengthString];
            
            handles.unit1.String=lengthString;
            handles.unit2.String=lengthString;
            handles.unit4.String=lengthString;
            handles.unit5.String=lengthString;
            handles.unit6.String=lengthString;
            handles.unit7.String=lengthString;
            
            handles.eUnit.String=obj.currentWorkSpace.getEString();
            
        end
        
        function export(obj)
            %export the data
            handles=obj.getHandles();
            if handles.exportPopup.Value == 2
                %matlab file
                [file,path] = uiputfile('*.mat','Export Data');
            else
                [file,path] = uiputfile('*.xlsx','Export Data');
            end
            if file ~= 0 
                %first connectivity
                links{1,1}='Links';
                links{2,1}='Node 1';links{2,2}='Node 2';
                for i=1:length(obj.allBeams)
                    links{2+i,1}=obj.allBeams{i}.nodes(1);
                    links{2+i,2}=obj.allBeams{i}.nodes(2);
                end
                %nodes
                for i=1:length(obj.nodes)
                    nodes{1,2*i}=['Node ', num2str(i)];
                    nodes{2,2*i}='x';
                    nodes{2,2*i+1}='y';
                end
                lowerLimit=str2double(handles.startFrameStatic2.String);
                upperLimit=str2double(handles.endFrameStatic2.String);
                nodeList=obj.getNodeList(lowerLimit,upperLimit) ;
                for i=1:size(nodeList,1)
                    nodes{2+i,1}=['Frame ' num2str(i)];
                    for j=1:size(nodeList,2)
                        nodes{2+i,j+1}=nodeList(i,j);
                    end
                end
                %torsion spring
                springs={};
                
                for i=1:length(obj.torsionSprings)
                    springs{1,5*(i-1)+1}='Torsion Spring';
                    springs{1,5*(i-1)+2}='Link 1';
                    springs{1,5*(i-1)+3}='Link2';
                    springs{1,5*(i-1)+4}=['Stiffness(' ForceUnit.getString(obj.currentWorkSpace.unitForce) '.' LengthUnit.getString(obj.currentWorkSpace.unitLength) ')'];
                    springs{2,5*(i-1)+1}=num2str(obj.torsionSprings(i).id);
                    springs{2,5*(i-1)+2}=num2str(obj.torsionSprings(i).link1);
                    springs{2,5*(i-1)+3}=num2str(obj.torsionSprings(i).link2);
                    springs{2,5*(i-1)+4}=num2str(obj.torsionSprings(i).stiffness);
                end
                
                rowIndex=size(springs,1)+1;
                index=1;
                for i=1:length(obj.allBeams)
                    if isa(class(obj.allBeams{i}),'KinematicsBeam') && logical(obj.allBeams{i}.linearSpring)
                        springs{rowIndex,2*index-1}=['Link '];
                        springs{rowIndex,2*index}=['Stiffness(' ForceUnit.getString(obj.currentWorkSpace.unitForce) '/' LengthUnit.getString(obj.currentWorkSpace.unitLength) ')'];
                        springs{rowIndex+1,2*index-1}= num2str(i);
                        springs{rowIndex+1,2*index}=num2str(obj.allBeams{i}.linearSpring);
                        index=index+1;
                    end
                end
                %moment
                moments={};
                arraySize=max([length(obj.powerList) length(obj.magnitudeList)]);
                for i=1:length(obj.moments)
                    moments{1,i+1}=['Moment-' num2str(i) '(' ForceUnit.getString(obj.currentWorkSpace.unitForce) '.' LengthUnit.getString(obj.currentWorkSpace.unitLength) ')'];
                    if isempty(obj.powerList)
                        powerList=ones(arraySize,1);
                    else
                        powerList=obj.powerList(1:arraySize)'/100;
                    end
                    if isempty(obj.magnitudeList)
                        magnitudeList=ones(arraySize,1)*obj.moments(i).magnitude;
                    else
                        magnitudeList=obj.magnitudeList(1:arraySize);
                    end
                    yData=powerList.*magnitudeList;
                    for j=1:length(yData)
                        moments{j+1,i+1}=yData(j);
                    end
                end
                if ~isempty(moments)
                    moments{1,1}='Frame';
                    for j=2:size(moments,1)
                        moments{j,1}=num2str(j-1);
                    end
                end
                %forces
                forces={};
                arraySize=max([length(obj.powerList) length(obj.magnitudeList)]);
                for i=1:length(obj.forces)
                    if ~logical(obj.forces(i).follower)
                        forces{1,2*i}=['Force-' num2str(i) 'X Magnitude(' ForceUnit.getString(obj.currentWorkSpace.unitForce) ')'];
                        forces{1,2*i+1}=['Force-' num2str(i) 'Y Magnitude(' ForceUnit.getString(obj.currentWorkSpace.unitForce) ')'];
                    else
                        forces{1,2*i}=['Force-' num2str(i) 'Magnitude(' ForceUnit.getString(obj.currentWorkSpace.unitForce) ')'];
                        forces{1,2*i+1}=['Force-' num2str(i) 'Angle(' ForceUnit.getString(obj.currentWorkSpace.unitForce) ')'];
                    end
                    if isempty(obj.powerList)
                        powerList=ones(arraySize,1);
                    else
                        powerList=obj.powerList(1:arraySize)'/100;
                    end
                    %magnitude
                    if isempty(obj.magnitudeList)
                        if logical(obj.forces(i).follower)
                            magnitudeList=ones(arraySize,1)*obj.forces(i).magnitude;
                        else
                            magnitudeList(:,1)=ones(arraySize,1)*obj.forces(i).xValue;
                            magnitudeList(:,2)=ones(arraySize,1)*obj.forces(i).yValue;
                        end
                    else
                        magnitudeList=obj.magnitudeList;
                    end
                    if logical(obj.forces(i).follower)
                        yData=powerList.*magnitudeList(1:arraySize);
                    else
                        yData(:,1)=powerList.*magnitudeList(1:arraySize,1);
                        yData(:,2)=powerList.*magnitudeList(1:arraySize,2);
                    end
                    
                    for j=1:length(yData)
                        forces{j+1,2*i}=yData(j,1);
                        if logical(obj.forces(i).follower)
                            forces{j+1,2*i+1}=num2str(obj.forces(i).angle);
                        else
                            forces{j+1,2*i+1}=yData(j,2);
                        end
                    end
                end
                if ~isempty(forces)
                    forces{1,1}='Frame:';
                    for j=2:size(forces,1)
                        forces{j,1}=num2str(j-1);
                    end
                end
                %energy
                energy={};
                if obj.mode ==5 
                    %bistable
                    energy{1,1}='Frame:';
                    for j=2:length(obj.energyList)
                        energy{j,1}=num2str(j-1);
                    end
                    energy{1,2}='Energy:';
                    for j=2:length(obj.energyList)
                        energy{j,2}=obj.energyList(j);
                    end
                    energy{1,3}='Load:';
                    for j=2:length(obj.loadList)
                        energy{j,3}=obj.loadList(j);
                    end
                end
            end
            %beam shape and stress
            shape={};
            for i=lowerLimit:upperLimit
                obj.currentFrame=i;
                obj.changeFrame();
                index=size(shape,1);
                beamIndex=1;
                for j=1:length(obj.allBeams)
                    if ~isa(obj.allBeams{j},'KinematicsBeam') 
                        obj.allBeams{j}=obj.allBeams{j}.getStressShape(obj.currentWorkSpace);
                        shape{index+1,beamIndex*3-2}=['Beam ' num2str(obj.allBeams{j}.nodes(1,1)) '-' num2str(obj.allBeams{j}.nodes(1,2))];
                        shape{index+2,beamIndex*3-2}='Shape-x';
                        shape{index+2,beamIndex*3-1}='Shape-y';
                        shape{index+2,beamIndex*3}='Stress';
                        for k=1:length(obj.allBeams{j}.xValues)
                            shape{index+2+k,beamIndex*3-2}=obj.allBeams{j}.xValues(k)+nodeList(i,2*obj.allBeams{j}.nodes(1,1)-1);
                            shape{index+2+k,beamIndex*3-1}=obj.allBeams{j}.yValues(k)+nodeList(i,2*obj.allBeams{j}.nodes(1,1));
                            shape{index+2+k,beamIndex*3}=obj.allBeams{j}.stressValues(k);
                        end
                        beamIndex=beamIndex+1;
                    end
                end
            end
            
            %export
            if handles.exportPopup.Value == 2
                %matlab file
                try
                    data.nodes=nodes;
                    data.links=links;
                    if ~isempty( springs)
                        data.springs=springs;
                    end
                    if ~isempty( moments)
                        data.moments=moments;
                    end
                    if ~isempty( forces)
                        data.forces=forces;
                    end
                    if ~isempty( energy)
                        data.energy=energy;
                    end
                    if ~isempty(shape )
                        data.shape=shape;
                    end
                    save(strcat(path,file),'data');
                catch err
                    disp(err);
                end
            else
                
                    xlswrite(strcat(path,file),links,'Links');
                    xlswrite(strcat(path,file),nodes,'Nodes');
                    if ~isempty( springs)
                        xlswrite(strcat(path,file),springs,'Springs');
                    end
                    if ~isempty( moments)
                        xlswrite(strcat(path,file),moments,'Moments');
                    end
                    if ~isempty( forces)
                        xlswrite(strcat(path,file),forces,'Forces');
                    end
                    if ~isempty( energy)
                        xlswrite(strcat(path,file),energy,'Energy');
                    end
                    if ~isempty( shape)
                        xlswrite(strcat(path,file),shape,'Shape');
                    end
                try    
                    sheetName = 'Sheet'; % EN: Sheet, DE: Tabelle, etc. (Lang. dependent)
                    % Open Excel file.
                    objExcel = actxserver('Excel.Application');
                    objExcel.Workbooks.Open(fullfile(path, file)); % Full path is necessary!
                    % Delete sheets.
                    try
                        % Throws an error if the sheets do not exist.
                        objExcel.ActiveWorkbook.Worksheets.Item([sheetName '1']).Delete;
                        objExcel.ActiveWorkbook.Worksheets.Item([sheetName '2']).Delete;
                        objExcel.ActiveWorkbook.Worksheets.Item([sheetName '3']).Delete;
                        objExcel.ActiveWorkbook.Worksheets.Item([sheetName '4']).Delete;
                    catch err
                        disp(err); % Do nothing.
                    end
                    % Save, close and clean up.
                    objExcel.ActiveWorkbook.Save;
                    objExcel.ActiveWorkbook.Close;
                    objExcel.Quit;
                    objExcel.delete;
                catch err
                    disp(err);
                end
            end
        end
        
    end
    
end

