classdef Beam
    properties
        id;
        frontMasterID;
        endMasterID;
        degreesOfFreedom;
        nodes;
        joints=Joint.empty;
        crossSection=CrossSection.empty;
        theta0;
        length0;
        inverted=0;
        %for drawing
        line=gobjects(0);
        selected=0;
        drawJoints=[1 1];
        colorBar;
        %storing
        lastValues;
        stressValues;
        xValues;
        yValues;
    end 
    methods
        function beam = Beam(id,frontMaster,endMaster,nodes,joints,angle,length,crossSection)
            %constructor function
            beam.id=id;
            beam.frontMasterID=frontMaster;
            beam.endMasterID=endMaster;
            beam.crossSection=crossSection;
            beam.length0=length;
            beam.theta0=angle;
            switch joints(1)
                case  Joint.GroundSlider
                    switch joints(2)
                        case Joint.GroundPin
                            beam.joints(1)=joints(2);beam.joints(2)=Joint.GroundWelded;
                            beam.nodes(1)=nodes(2);beam.nodes(2)=nodes(1);
                            beam.inverted=1;
                        case Joint.GroundWelded
                            beam.joints(1)=joints(2);beam.joints(2)=joints(1);
                            beam.nodes(1)=nodes(2);beam.nodes(2)=nodes(1);
                            beam.inverted=1;
                        otherwise
                            disp('Invalid Type');
                    end
                case Joint.GroundPin
                    switch joints(2)
                        case Joint.GroundSlider
                            beam.joints(1)=Joint.GroundWelded;beam.joints(2)=joints(2);
                            beam.nodes(1)=nodes(1);beam.nodes(2)=nodes(2);
                        case Joint.GroundPin
                            beam.joints(1)=joints(1);beam.joints(2)=joints(2);
                            beam.nodes(1)=nodes(1);beam.nodes(2)=nodes(2);
                        case Joint.Welded
                            if ~isempty(crossSection) && crossSection.inverse
                                beam.joints(1)=joints(2);beam.joints(2)=joints(1);
                                beam.nodes(1)=nodes(2);beam.nodes(2)=nodes(1);
                                beam.inverted=1;
                            else
                                beam.joints(1)=joints(1);beam.joints(2)=joints(2);
                                beam.nodes(1)=nodes(1);beam.nodes(2)=nodes(2);
                            end
                        case Joint.Pin
                            beam.joints(1)=joints(1);beam.joints(2)=joints(2);
                            beam.nodes(1)=nodes(1);beam.nodes(2)=nodes(2);
                        case Joint.Slider
                            beam.joints(1)=joints(1);beam.joints(2)=joints(2);
                            beam.nodes(1)=nodes(1);beam.nodes(2)=nodes(2);
                        otherwise
                            disp('Invalid Type');
                    end
                case Joint.GroundWelded
                    switch joints(2)
                        case Joint.GroundSlider
                            beam.joints(1)=joints(1);beam.joints(2)=joints(2);
                            beam.nodes(1)=nodes(1);beam.nodes(2)=nodes(2);
                        case Joint.Slider
                            beam.joints(1)=joints(1);beam.joints(2)=joints(2);
                            beam.nodes(1)=nodes(1);beam.nodes(2)=nodes(2);
                        case Joint.Welded
                            beam.joints(1)=joints(1);beam.joints(2)=joints(2);
                            beam.nodes(1)=nodes(1);beam.nodes(2)=nodes(2);
                        case Joint.Pin
                            beam.joints(1)=joints(1);beam.joints(2)=joints(2);
                            beam.nodes(1)=nodes(1);beam.nodes(2)=nodes(2);
                        otherwise
                            disp('Invalid Type');
                    end
                case Joint.Welded
                    switch joints(2)
                        case Joint.GroundPin
                            if ~isempty(crossSection) && crossSection.inverse
                                beam.joints(1)=joints(1);beam.joints(2)=joints(2);
                                beam.nodes(1)=nodes(1);beam.nodes(2)=nodes(2);
                            else
                                beam.joints(1)=joints(2);beam.joints(2)=joints(1);
                                beam.nodes(1)=nodes(2);beam.nodes(2)=nodes(1);
                                beam.inverted=true;
                            end
                        case Joint.GroundWelded
                            beam.joints(1)=joints(2);beam.joints(2)=joints(1);
                            beam.nodes(1)=nodes(2);beam.nodes(2)=nodes(1);
                            beam.inverted=1;
                        case Joint.Welded
                            beam.joints(1)=joints(1);beam.joints(2)=joints(2);
                            beam.nodes(1)=nodes(1);beam.nodes(2)=nodes(2);
                        case Joint.Pin
                            beam.joints(1)=joints(2);beam.joints(2)=joints(1);
                            beam.nodes(1)=nodes(2);beam.nodes(2)=nodes(1);
                            beam.inverted=1;
                        case Joint.Slider
                            beam.joints(1)=joints(1);beam.joints(2)=joints(2);
                            beam.nodes(1)=nodes(1);beam.nodes(2)=nodes(2);
                        otherwise
                            disp('Invalid Type');
                    end
                case Joint.Pin
                    switch joints(2)
                        case Joint.GroundWelded
                            beam.joints(1)=joints(2);beam.joints(2)=joints(1);
                            beam.nodes(1)=nodes(2);beam.nodes(2)=nodes(1);
                            beam.inverted=1;
                        case Joint.GroundPin
                            beam.joints(1)=joints(2);beam.joints(2)=joints(1);
                            beam.nodes(1)=nodes(2);beam.nodes(2)=nodes(1);
                            beam.inverted=1;
                        case Joint.Welded
                            beam.joints(1)=joints(1);beam.joints(2)=joints(2);
                            beam.nodes(1)=nodes(1);beam.nodes(2)=nodes(2);
                        case Joint.Pin
                            beam.joints(1)=joints(1);beam.joints(2)=joints(2);
                            beam.nodes(1)=nodes(1);beam.nodes(2)=nodes(2);
                        case Joint.Slider
                            beam.joints(1)=joints(1);beam.joints(2)=joints(2);
                            beam.nodes(1)=nodes(1);beam.nodes(2)=nodes(2);
                        otherwise
                            disp('Invalid Type');
                    end
                case Joint.Slider
                    switch joints(2)
                        case  Joint.GroundPin
                            beam.joints(1)=joints(2);beam.joints(2)=joints(1);
                            beam.nodes(1)=nodes(2);beam.nodes(2)=nodes(1);
                            beam.inverted=1;
                        case Joint.GroundWelded
                            beam.joints(1)=joints(2);beam.joints(2)=joints(1);
                            beam.nodes(1)=nodes(2);beam.nodes(2)=nodes(1);
                            beam.inverted=1;
                        case Joint.Welded
                            beam.joints(1)=joints(2);beam.joints(2)=joints(1);
                            beam.nodes(1)=nodes(2);beam.nodes(2)=nodes(1);
                            beam.inverted=1;
                        case Joint.Pin
                            beam.joints(1)=joints(2);beam.joints(2)=joints(1);
                            beam.nodes(1)=nodes(2);beam.nodes(2)=nodes(1);
                            beam.inverted=1;
                        otherwise
                            disp('Invalid Type');
                    end
                otherwise
                    disp('Invalid Type');
            end
            if logical(beam.inverted)
                if logical(angle)
                    beam.theta0=sign(-angle)*(pi-abs(angle));
                else
                    beam.theta0=pi;
                end
            end
            %             %reverse PRB Matrix if the beam is inverted
            %             if ~isempty(beam.crossSection) && ~isempty(beam.crossSection.prbModel) && logical(beam.inverted)
%                 beam.crossSection.prbModel(:,2)=flipud(beam.crossSection.prbModel(:,2));
%                 beam.crossSection.prbModel=flipud(beam.crossSection.prbModel);
%             end
        end
        
        function beam=updateInternalID(beam,modifier)
            %do nothing
        end
        %% 
        % GUI functions
        function beam=colorLine(beam,colorBeam)
            %color the line
            for i=1:length(beam.line(1,:))
                if  isgraphics(beam.line(1,i)) && strcmp(class(beam),'KinematicsBeam')
                    beam.line(1,i).Color=colorBeam;
                end
            end
            %color the joints
            for i=2:3
                if isgraphics(beam.line(i,1))
                    beam.line(i,1).EdgeColor=colorBeam;
                end
            end
            %color the ground
            if isgraphics(beam.line(4,1))&& beam.joints(1,1)== Joint.GroundPin
                beam.line(4,1).FaceColor=colorBeam;
                for i=2:7
                    beam.line(4,i).Color=colorBeam;
                end
            elseif isgraphics(beam.line(4,1))
                for i=1:6
                    beam.line(4,i).Color=colorBeam;
                end
            end
            
            if isgraphics(beam.line(5,1)) && beam.joints(1,2)==Joint.GroundPin
                beam.line(5,1).FaceColor=colorBeam;
                for i=2:7
                    beam.line(5,i).Color=colorBeam;
                end
            elseif isgraphics(beam.line(5,1))
                for i=1:6
                    beam.line(5,i).Color=colorBeam;
                end
            end
        end
        
        function beam=deleteLinkDrawing(beam)
            %delete link drawing
            delete(beam.line);
            delete(beam.colorBar);
            beam.line=gobjects(0);
        end
        
        function beam=clickEvent(beam,mode,parent)
            %node click event
            switch(mode)
                case Module.Free
                    fnc=@(src,evnt)beam.clickLinkFree(src,evnt,parent,parent.getPosFromID(beam.id));
                case Module.Range
                    fnc=@(src,evnt)beam.clickLinkRange(src,evnt,parent,parent.getPosFromID(beam.id));
                case Module.Load
                    fnc=[];
                case Module.Distance
                    fnc=@(src,evnt)beam.clickLinkDistance(src,evnt,parent,parent.getPosFromID(beam.id));
                case Module.MechanicalAdvantage
                    fnc=[];
                case Module.EnergyPlot
                    fnc=@(src,evnt)beam.clickLinkBistable(src,evnt,parent,parent.getPosFromID(beam.id));
                case Module.PostProcessing
                    fnc=[];
                case Module.KinematicSynthesis
                    fnc=@(src,evnt)beam.clickLinkKinematicSynthesis(src,evnt,parent,parent.getPosFromID(beam.id));
                case Module.FlexuralStiffnessSynthesis
                    fnc=@(src,evnt)beam.clickLinkFlexuralSynthesis(src,evnt,parent,parent.getPosFromID(beam.id));
                case Module.BiStableSynthesis
                    fnc=[];
                case Module.ConstantForceSynthesis
                    fnc=[];
            end
            
            for i=1:length(beam.line(1,:))
                if  isgraphics(beam.line(1,i))
                    beam.line(1,i).ButtonDownFcn=fnc;
                end
            end
            %same for joints
            if  isgraphics(beam.line(2,1))
                beam.line(2,1).ButtonDownFcn=fnc;
            end
            if  isgraphics(beam.line(3,1))
                beam.line(3,1).ButtonDownFcn=fnc;
            end
            
        end
        
        function beam=rightClickMenu(beam,mode,parent)
            %right click menu
            handles=parent.getHandles();
            switch(mode)
                case Module.PostProcessing
                    menu= uicontextmenu(handles.mainGUI);
                    if isempty(beam.crossSection)
                        uimenu(menu,'Label',['Set Link ' num2str(beam.nodes(1)) '-'  num2str(beam.nodes(2))  ' Angle°  as X Axis'],'Callback',@(source,callbackdata)parent.setLinkXAxis(source,callbackdata,beam.id,1));
                        uimenu(menu,'Label',['Set Link ' num2str(beam.nodes(1)) '-'  num2str(beam.nodes(2)) ' Length  as X Axis'],'Callback',@(source,callbackdata)parent.setLinkXAxis(source,callbackdata,beam.id,2));
                        uimenu(menu,'Label',['Add Link ' num2str(beam.nodes(1)) '-'  num2str(beam.nodes(2)) ' Angle°  to Y Axis'],'Callback',@(source,callbackdata)parent.setLinkYAxis(source,callbackdata,beam.id,1));
                        uimenu(menu,'Label',['Add Link ' num2str(beam.nodes(1)) '-'  num2str(beam.nodes(2)) ' Length  to Y Axis'],'Callback',@(source,callbackdata)parent.setLinkYAxis(source,callbackdata,beam.id,2));
                    end
                case Module.FlexuralStiffnessSynthesis
                    if ~isempty(beam.crossSection)
                        menu= uicontextmenu(handles.mainGUI);
                        m1 = uimenu(menu,'Label','Optimization Variable','Callback',@(source,callbackdata)parent.flexuralSynthesisRight(source,callbackdata,beam.id));
                        if isempty(parent.findFlexuralUnknownPos(beam.id))
                            m1.Checked='off';
                        else
                            m1.Checked='on';
                        end
                    else
                        menu=[];
                    end
                case Module.BiStableSynthesis
                    if ~isempty(beam.crossSection)
                        menu= uicontextmenu(handles.mainGUI);
                        popHandles=parent.getPopHandles();
                        if popHandles.type.Value == 1
                            m1 = uimenu(menu,'Label','Optimization Variable','Callback',@(source,callbackdata)parent.bistableSynthesisRight(source,callbackdata,beam.id));
                        end
                        if isempty(parent.findBistableUnknownPos(beam.id))
                            m1.Checked='off';
                        else
                            m1.Checked='on';
                        end
                    else
                        menu=[];
                    end
                otherwise
                    menu= uicontextmenu(handles.mainGUI);
            end
            for i=1:5
                for j=1:length(beam.line(i,:))
                    if isgraphics(beam.line(i,j))
                        beam.line(i,j).UIContextMenu=menu;
                    end
                end
            end
        end
        
        function beam=setSelected(beam,selected)
            %set the selected
            beam.selected=selected;
        end
        
        function clickLinkFree(beam,src,evnt,parent,id)
            %click during a range
            handles=parent.getHandles();
            currentID=id;
            %idList for the selected nodes
            for i=1:length(parent.getKinematics().allBeams)
                %if multiple selection is permitted previously selected points will be
                %retained else only current point be made selected
                if  i ~= currentID
                    parent.setSelectLink2(i,0);
                elseif i == currentID
                    parent.setSelectLink2(i,1);
                end
            end
            handles.selectedLinkRange.String='link is selected';
            handles.selectedLinkRange.ForegroundColor=Colors.StatusComplete.getColor();
            %get the degrees of freedom for the selected link
            if beam.fixed ==1  || beam.degreesOfFreedom ==0 || (beam.degreesOfFreedom==1 && logical(beam.linearSpring))
                if logical(beam.linearSpring)
                    mode=2;
                elseif  beam.frontMasterID ~= beam.id
                    mode=1;
                else
                    mode=0;
                end
            elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider')
                mode=3;
            elseif beam.joints(2) == Joint.Slider
                mode=2;
            elseif beam.joints(2) == Joint.GroundSlider
                mode=4;
                %initialGuess=(nodes(RB.point2).x-nodes(RB.point1).x)*cos(theta)+(nodes(RB.point2).y-nodes(RB.point1).y)*sin(theta);
            else
                if logical(beam.linearSpring)
                    mode=3;
                else
                    mode=1;
                end
            end
            link=id;
            if beam.joints(1) == Joint.GroundPin || beam.joints(2) == Joint.GroundSlider
                node=1;
            else
                %get the mouse position
                mousePos=get(handles.designPlot,'CurrentPoint');
                mouseX=mousePos(1,1);
                mouseY=mousePos(1,2);
                node1=parent.getKinematics().nodes(beam.nodes(1)).getNode();
                node2=parent.getKinematics().nodes(beam.nodes(2)).getNode();
                %calculate the distances between mouse pos and nodes
                distance2Point1=sqrt((mouseX-node1.x)^2+(mouseY-node1.y)^2);
                distance2Point2=sqrt((mouseX-node2.x)^2+(mouseY-node2.y)^2);
                if distance2Point1 < distance2Point2
                    node=2;
                else
                    node=1;
                end
            end
            freeKinematics=struct('mode',mode,'link',link,'node',node);
            parent.setFreeKinematics(freeKinematics);
            %plot
            parent.plotEverything();
        end
        
        function clickLinkDistance(beam,src,evnt,parent,id)
            %click during a distance analysis
            if logical(parent.getDistanceAnalysis().busy)
                return;
            end
            uiwait(msgbox('Please select a rigid link to rotate.','Error','error','modal'));
            popHandles=parent.getPopHandles();
            figure(popHandles.distanceGUI);
        end
        
        function clickLinkBistable(beam,src,evnt,parent,id)
            %click during a distance analysis
            if logical(parent.getBistable().busy)
                return;
            end
            uiwait(msgbox('Please select a rigid link to rotate.','Error','error','modal'));
            popHandles=parent.getPopHandles();
            figure(popHandles.energyGUI);
        end
        
        function clickLinkFlexuralSynthesis(beam,src,evnt,parent,id)
            %click during a distance analysis
            if logical(parent.getBusy())
                return;
            end
            if evnt.Button == 1
                uiwait(msgbox('Please select a rigid link to rotate.','Error','error','modal'));
                popHandles=parent.getPopHandles();
                figure(popHandles.energyGUI);
            end
        end
        
        function beam=getStress(beam,workspace)
            beam.stressValues=zeros(1,100);
        end
   
    end
end