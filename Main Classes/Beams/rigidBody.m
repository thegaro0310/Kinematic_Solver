classdef rigidBody
    properties
        id;
        masterID=-1;
        isSlave=-1;
        degreesOfFreedom=0;
        nodes;
        joints=Joint.empty;
        drawJoints=[1 1];
        compliantGroup=[0 0];
        crossSection=CrossSection.empty;
        angle;
        angle0;
        lastAngle;
        additionalAngle=0;
        angleWithNeighbour=0;
        angleModifier=1;
        length;
        initialLength;
        % vector;
        initialGuess;
        type;
        plate=-1;
        fixed=0;
        %for line
        line=gobjects(0);
        sliderAngle=[];
        angleStorage=[];
        lengthStorage=[];
        selected=0;
        linearSpringValue=0;
    end
    
    methods
        function obj = rigidBody(idNumber,nodes,joints,angle,allNodes,masterID,crossSection,linearSpringValue)
            obj.id=idNumber;
            if masterID == -1
                obj.masterID=idNumber;
            else
                obj.masterID=masterID;
                obj.isSlave=1;
            end
            obj.crossSection=crossSection;
            obj.linearSpringValue=linearSpringValue; 
            switch joints(1)
                case  Joint.GroundSlider
                    switch joints(2)
                        case Joint.GroundPin
                            obj.type='allFixedSlider';
                            obj.joints(1)=joints(2);obj.joints(2)=Joint.GroundWelded;
                            obj.nodes(1)=nodes(2);obj.nodes(2)=nodes(1);
                            %obj.vector.one=vectorPris.one;
                            %obj.vector.two=vectorPris.two;
                            obj.degreesOfFreedom=1;
                            obj.sliderAngle=angle;
                        case Joint.GroundWelded
                            obj.type='allFixedSlider';
                            obj.joints(1)=joints(2);obj.joints(2)=joints(1);
                            obj.nodes(1)=nodes(2);obj.nodes(2)=nodes(1);
                            % obj.vector.one=vectorPris.one;
                            % obj.vector.two=vectorPris.two;
                            obj.degreesOfFreedom=1;
                            obj.sliderAngle=angle;
                        otherwise
                            disp('Error on input file.Please go home');
                    end
                case Joint.GroundPin
                    switch joints(2)
                        case Joint.GroundSlider
                            obj.type='allFixedSlider';
                            obj.joints(1)=Joint.GroundWelded;obj.joints(2)=joints(2);
                            obj.nodes(1)=nodes(1);obj.nodes(2)=nodes(2);
                            % obj.vector.one=vectorPris.one;
                            % obj.vector.two=vectorPris.two;
                            obj.sliderAngle=angle;
                            obj.degreesOfFreedom=1;
                        case Joint.GroundPin
                            obj.type='allFixedPinBeam';
                            obj.joints(1)=joints(1);obj.joints(2)=joints(2);
                            obj.nodes(1)=nodes(1);obj.nodes(2)=nodes(2);
                            obj.degreesOfFreedom=0;
                            obj.fixed=1;
                        case Joint.Welded
                            obj.type='fixedWeldedPinBeam';
                            obj.joints(1)=joints(1);obj.joints(2)=joints(2);
                            obj.nodes(1)=nodes(1);obj.nodes(2)=nodes(2);
                            obj.degreesOfFreedom=1;
                        case Joint.Pin
                            obj.type='fixedPinBeam';
                            obj.joints(1)=joints(1);obj.joints(2)=joints(2);
                            obj.nodes(1)=nodes(1);obj.nodes(2)=nodes(2);
                            obj.degreesOfFreedom=1;
                        case Joint.Slider
                            obj.type='fixedPinSlider';
                            obj.joints(1)=joints(1);obj.joints(2)=joints(2);
                            obj.nodes(1)=nodes(1);obj.nodes(2)=nodes(2);
                            obj.degreesOfFreedom=2;
                            % obj.vector.one=point1I;
                            % obj.vector.two=point2I;
                        otherwise
                            disp('Error on input file.Please go home');
                    end
                case Joint.GroundWelded
                    switch joints(2)
                        case Joint.GroundSlider
                            obj.type='allFixedSlider';
                            obj.joints(1)=joints(1);obj.joints(2)=joints(2);
                            obj.nodes(1)=nodes(1);obj.nodes(2)=nodes(2);
                            % obj.vector.one=vectorPris.one;
                            % obj.vector.two=vectorPris.two;
                            obj.degreesOfFreedom=1;
                            obj.sliderAngle=angle;
                        case Joint.Slider
                            obj.type='fixedSlider';
                            obj.joints(1)=joints(1);obj.joints(2)=joints(2);
                            obj.nodes(1)=nodes(1);obj.nodes(2)=nodes(2);
                            % obj.vector.one=point1I;
                            % obj.vector.two=point2I;
                            obj.degreesOfFreedom=1;
                        case Joint.Welded
                            obj.type='fixedToWallBeam';
                            obj.joints(1)=joints(1);obj.joints(2)=joints(2);
                            obj.nodes(1)=nodes(1);obj.nodes(2)=nodes(2);
                            obj.degreesOfFreedom=0;
                        case Joint.Pin
                            obj.type='fixedToWallPinBeam';
                            obj.joints(1)=joints(1);obj.joints(2)=joints(2);
                            obj.nodes(1)=nodes(1);obj.nodes(2)=nodes(2);
                            obj.degreesOfFreedom=0;
                        otherwise
                            disp('Error on input file.Please go home');
                    end
                case Joint.Welded
                    switch joints(2)
                        case Joint.GroundPin
                            obj.type='fixedWeldedPinBeam';
                            obj.joints(1)=joints(2);obj.joints(2)=joints(1);
                            obj.nodes(1)=nodes(2);obj.nodes(2)=nodes(1);
                            obj.degreesOfFreedom=1;
                        case Joint.GroundWelded
                            obj.type='fixedToWallBeam';
                            obj.joints(1)=joints(2);obj.joints(2)=joints(1);
                            obj.nodes(1)=nodes(2);obj.nodes(2)=nodes(1);
                            obj.degreesOfFreedom=0;
                        case Joint.Welded
                            obj.type='weldedBeam';
                            obj.joints(1)=joints(1);obj.joints(2)=joints(2);
                            obj.nodes(1)=nodes(1);obj.nodes(2)=nodes(2);
                            obj.degreesOfFreedom=0;
                        case Joint.Pin
                            obj.type='weldedPinBeam';
                            obj.joints(1)=joints(2);obj.joints(2)=joints(1);
                            obj.nodes(1)=nodes(2);obj.nodes(2)=nodes(1);
                            obj.degreesOfFreedom=1;
                        case Joint.Slider
                            obj.type='slider';
                            obj.joints(1)=joints(1);obj.joints(2)=joints(2);
                            obj.nodes(1)=nodes(1);obj.nodes(2)=nodes(2);
                            % obj.vector.one=point1I;
                            % obj.vector.two=point2I;
                            obj.degreesOfFreedom=1;
                        otherwise
                            disp('Error on input file.Please go home');
                    end
                case Joint.Pin
                    switch joints(2)
                        case Joint.GroundWelded
                            obj.type='fixedToWallPinBeam';
                            obj.joints(1)=joints(2);obj.joints(2)=joints(1);
                            obj.nodes(1)=nodes(2);obj.nodes(2)=nodes(1);
                            obj.degreesOfFreedom=0;
                        case Joint.GroundPin
                            obj.type='fixedPinBeam';
                            obj.joints(1)=joints(2);obj.joints(2)=joints(1);
                            obj.nodes(1)=nodes(2);obj.nodes(2)=nodes(1);
                            obj.degreesOfFreedom=1;
                        case Joint.Welded
                            obj.type='weldedPinBeam';
                            obj.joints(1)=joints(1);obj.joints(2)=joints(2);
                            obj.nodes(1)=nodes(1);obj.nodes(2)=nodes(2);
                            obj.degreesOfFreedom=1;
                        case Joint.Pin
                            obj.type='pinBeam';
                            obj.joints(1)=joints(1);obj.joints(2)=joints(2);
                            obj.nodes(1)=nodes(1);obj.nodes(2)=nodes(2);
                            obj.degreesOfFreedom=1;
                        case Joint.Slider
                            obj.type='pinSlider';
                            obj.joints(1)=joints(1);obj.joints(2)=joints(2);
                            obj.nodes(1)=nodes(1);obj.nodes(2)=nodes(2);
                            obj.degreesOfFreedom=2;
                        otherwise
                            disp('Error on input file.Please go home');
                    end
                case Joint.Slider
                    switch joints(2)
                        case  Joint.GroundPin
                            obj.type='fixedPinSlider';
                            obj.joints(1)=joints(2);obj.joints(2)=joints(1);
                            obj.nodes(1)=nodes(2);obj.nodes(2)=nodes(1);
                            obj.degreesOfFreedom=2;
                            % obj.vector.one=point2I;
                            % obj.vector.two=point1I;
                        case Joint.GroundWelded
                            obj.type='fixedSlider';
                            obj.joints(1)=joints(2);obj.joints(2)=joints(1);
                            obj.nodes(1)=nodes(2);obj.nodes(2)=nodes(1);
                            % obj.vector.one=point1I;
                            % obj.vector.two=point2I;
                            obj.degreesOfFreedom=1;
                        case Joint.Welded
                            obj.type='slider';
                            obj.joints(1)=joints(2);obj.joints(2)=joints(1);
                            obj.nodes(1)=nodes(2);obj.nodes(2)=nodes(1);
                            % obj.vector.one=point2I;
                            %obj.vector.two=point1I;
                            obj.degreesOfFreedom=1;
                        case Joint.Pin
                            obj.type='pinSlider';
                            obj.joints(1)=joints(2);obj.joints(2)=joints(1);
                            obj.nodes(1)=nodes(2);obj.nodes(2)=nodes(1);
                            obj.degreesOfFreedom=2;
                        otherwise
                            disp('Error on input file.Please go home');
                    end
                otherwise
                    disp('Error on input file.Please go home');
            end
            point1=allNodes(obj.nodes(1)).getNode();
            point2=allNodes(obj.nodes(2)).getNode();
            obj.angle0=formatAngle( atan2(point2.y-point1.y,point2.x-point1.x));
            obj.angle=formatAngle(atan2(point2.y-point1.y,point2.x-point1.x));
            obj.lastAngle=obj.angle;
            obj.length=sqrt((point2.y-point1.y)^2+(point2.x-point1.x)^2);
            obj.initialLength=obj.length;
            %adjust the degrees of freedom  for linear springs
            if obj.fixed ==1 || obj.masterID ~= obj.id || obj.degreesOfFreedom ==0
                if logical(obj.linearSpringValue)
                    obj.degreesOfFreedom=1;
                end
            else
                if logical(obj.linearSpringValue)
                    obj.degreesOfFreedom=2;
                end
            end
            obj=obj.calculateAngle(allNodes);
        end
        
        function initialGuess=calculateInitialGuess(obj)
            if obj.fixed ==1 || obj.masterID ~= obj.id || obj.degreesOfFreedom ==0 || (obj.degreesOfFreedom==1 && logical(obj.linearSpringValue))
                if logical(obj.linearSpringValue)
                    initialGuess=obj.length;
                else
                    initialGuess=[];
                end
            elseif strcmp(obj.type,'fixedPinSlider') || strcmp(obj.type,'pinSlider')
                initialGuess(1)=obj.angle;
                initialGuess(2)=obj.length;
            elseif obj.joints(2) == Joint.Slider
                initialGuess=obj.length;
            elseif obj.joints(2) == Joint.GroundSlider
                theta=obj.sliderAngle*pi/180;
                initialGuess=(obj.length*cos(obj.angle))*cos(theta)+(obj.length*sin(obj.angle))*sin(theta);
                %initialGuess=(nodes(RB.point2).x-nodes(RB.point1).x)*cos(theta)+(nodes(RB.point2).y-nodes(RB.point1).y)*sin(theta);
            else
                if logical(obj.linearSpringValue)
                    initialGuess(1)=obj.angle;
                    initialGuess(2)=obj.length;
                else
                    initialGuess=obj.angle;
                end
            end
        end
        
        function obj=calculateAngle(obj,allNodes)
            %angle is the angle between x axis and the vector pointing from
            %node 1 to node 2
            point1=allNodes(obj.nodes(1)).getNode();
            point2=allNodes(obj.nodes(2)).getNode();
            %check for jumps
            newAngle=atan2(point2.y-point1.y,point2.x-point1.x) ;
%             if  (mod(obj.lastAngle,2*pi)<= pi && mod(obj.lastAngle,2*pi) >pi/2 && newAngle >-pi &&newAngle <-pi/2) 
%                 %second region
%                 obj.additionalAngle=obj.additionalAngle+1; 
%             elseif  (mod(obj.lastAngle,2*pi)>= pi && mod(obj.lastAngle,2*pi) < 3/2*pi && newAngle >pi/2 &&newAngle <pi) 
%                 obj.additionalAngle=obj.additionalAngle-1 ;
%             end
            if  ((obj.lastAngle <= pi && obj.lastAngle >pi/2) || (obj.lastAngle > -3/2*pi && obj.lastAngle <= -pi &&  obj.additionalAngle ~= 0)) && newAngle >=-pi &&newAngle <-pi/2 
                %second region
                obj.additionalAngle=obj.additionalAngle+1;
            elseif  ( (obj.lastAngle < -3/2*pi && obj.lastAngle >= -pi) || (obj.lastAngle < 3/2*pi && obj.lastAngle >= pi &&  obj.additionalAngle ~= 0))  && newAngle >pi/2 &&newAngle <=pi
                obj.additionalAngle=obj.additionalAngle-1 ;
            end
            
            obj.angle=atan2(point2.y-point1.y,point2.x-point1.x)+obj.additionalAngle*2*pi;
            obj.lastAngle=obj.angle;
            obj.length=sqrt((point2.y-point1.y)^2+(point2.x-point1.x)^2);
            obj.initialGuess=obj.calculateInitialGuess();
        end
        
        
        %this function checks if the beam length is same after new nodes
        %are found
        function error=check(obj,newNodes,rigidBeamList)
            error=false;
            if  strcmp(obj.type,'slider')
                masterID=obj.masterID;
                point1=newNodes(rigidBeamList(masterID).nodes(1)).getNode();
                point2=newNodes(rigidBeamList(masterID).nodes(2)).getNode();
                vectorX=point2.x-point1.x;
                vectorY=point2.y-point1.y;
                neighbourAngle=formatAngle(atan2(vectorY,vectorX));
                point1=newNodes(rigidBeamList(masterID).nodes(1)).getNode();
                point2=newNodes(rigidBeamList(masterID).nodes(2)).getNode();
                vectorX2=point2.x-point1.x;
                vectorY2=point2.y-point1.y;
                currentAngle=formatAngle(atan2(vectorY2,vectorX2));
                angleWithNeighbourNew=currentAngle-neighbourAngle;
                if abs(abs(angleWithNeighbourNew) - abs(obj.angleWithNeighbour)) > 1e-6
                    error=true;
                    disp('Run for your life. Angles are not same');
                end
            elseif obj.masterID ~= obj.id
                masterID=find([rigidBeamList.id]==obj.masterID);
                point1=newNodes(rigidBeamList(masterID).nodes(1)).getNode();
                point2=newNodes(rigidBeamList(masterID).nodes(2)).getNode();
                vectorX=point2.x-point1.x;
                vectorY=point2.y-point1.y;
                masterAngle=formatAngle(atan2(vectorY,vectorX));
                point1=newNodes(rigidBeamList(masterID).nodes(1)).getNode();
                point2=newNodes(rigidBeamList(masterID).nodes(2)).getNode();
                vectorX2=point2.x-point1.x;
                vectorY2=point2.y-point1.y;
                %currentAngle=formatAngle(atan2(vectorY2,vectorX2))
                angleWithNeighbourNew=acos(dot([vectorX2 vectorY2],[vectorX vectorY])/(norm([vectorX2 vectorY2])*norm([vectorX vectorY])))*obj.angleModifier;
                %angleWithNeighbourNew=currentAngle-masterAngle;
                if abs(angleWithNeighbourNew) - abs(obj.angleWithNeighbour) > 1e-5
                    %  error=true;
                    disp('Run for your life. Angles are not same');
                    error=true;
                end
            elseif strcmp(obj.type,'fixedSlider') ~= 1 && strcmp(obj.type,'allFixedSlider') ~= 1&& strcmp(obj.type,'fixedPinSlider') ~= 1&& strcmp(obj.type,'pinSlider') ~= 1 && ~logical(obj.linearSpringValue)
                point1=newNodes(obj.nodes(1)).getNode();
                point2=newNodes(obj.nodes(2)).getNode();
                length=sqrt((point2.y-point1.y)^2+(point2.x-point1.x)^2);
                if abs(length - obj.length) > 1e-5
                    error=true;
                    length - obj.length
                    %disp('Run for your life. Lengths are not same.So please panic');
                end
            end
        end
        
        function obj=findNeighbour(obj,rigidBeamList)
            obj.initialLength=obj.length;
            if strcmp(obj.type,'slider')
                %neighbourID=find([rigidBeamList.connectionType2]==1);
                %for i=1:length(neighbourID)
                for i=1:length(rigidBeamList)
                    if (rigidBeamList(i).nodes(2))== obj.nodes(1)
                        %if (rigidBeamList(neighbourID(i)).point2)== RB.point1
                        obj.masterID=rigidBeamList(i).id;
                        neighbourAngle=rigidBeamList(i).angle;
                        currentAngle=obj.angle;
                        obj.angleWithNeighbour=currentAngle-neighbourAngle;
                        break;
                    end
                end
            elseif obj.masterID ~= obj.id
                neighbourAngle=rigidBeamList([rigidBeamList.id]==obj.masterID).angle;
                currentAngle=obj.angle;
                obj.angleWithNeighbour=currentAngle-neighbourAngle;
            end
        end
        
        
        function drawNoGUI(obj,currentWorkspace,fig,nodes)
            %draw without GUI mode
            if logical(obj.fixed)
                return;
            end
            figure(fig);
            limit=currentWorkspace.getLimit()*0.5;
            node1=nodes(obj.nodes(1,1)).getNode;
            node2=nodes(obj.nodes(1,2)).getNode;
            if logical(obj.linearSpringValue)
                linearSpring=1;
            else
                linearSpring=0;
            end
            [x,y]=DrawLink.drawLine([node1 node2],obj.joints,limit,linearSpring);
            [xJoint1,yJoint1,xJoint2,yJoint2]=DrawLink.drawJoints([node1 node2],obj.joints,limit,obj.sliderAngle);
            [xGround1,yGround1,xGround2,yGround2]=DrawLink.drawGrounds([node1 node2],obj.joints,limit,obj.sliderAngle);
            if ~isempty(obj.crossSection)
                lineWidthBeam=0.5;
            elseif logical(obj.compliantGroup(1)) || ~logical(sum(obj.drawJoints)) || sum(obj.drawJoints) < 0
                lineWidthBeam=3.5;
            else
                lineWidthBeam=2;
            end
            obj.line=gobjects(5,7);
            %line
            if ~isempty(x)
                obj.line(1,1)=plot(x,y,'LineWidth',lineWidthBeam);
            end
            %joint 1
            if ~isempty(xJoint1)
                obj.line(2,1)=patch(xJoint1,yJoint1,zeros(length(xJoint1),1),zeros(length(xJoint1),1),'FaceColor','none');
            end
            %joint 2
            if ~isempty(xJoint2)
                obj.line(3,1)=patch(xJoint2,yJoint2,zeros(length(xJoint2),1),zeros(length(xJoint2),1),'FaceColor','none');
            end
            %ground 1
            if obj.joints(1,1)== Joint.GroundPin
                obj.line(4,1)=patch([xGround1(1,1);xGround1(1,2);xGround1(1,3)],[yGround1(1,1);yGround1(1,2);yGround1(1,3)],zeros(3,1),zeros(3,1),'FaceColor','none');
                index=2;
            else
                index=1;
            end
            for i=index:length(xGround1)
                obj.line(4,i)=plot([xGround1(i,1) xGround1(i,2)],[yGround1(i,1) yGround1(i,2)]);
            end
            %ground 2
            if obj.joints(1,2)==Joint.GroundPin
                obj.line(5,1)=patch([xGround2(1,1);xGround2(1,2);xGround2(1,3)],[yGround2(1,1);yGround2(1,2);yGround2(1,3)],zeros(3,1),zeros(3,1),'FaceColor','none');
                index=2;
            else
                index=1;
            end
            for i=index:length(xGround2)
                obj.line(5,i)=plot([xGround2(i,1) xGround2(i,2)],[yGround2(i,1) yGround2(i,2)]);
            end
            %color the beam
            color=Colors.NormalLink.getColor();
            obj.colorLine(color);
        end
        
        
        function obj=drawLink(obj,currentAxis,nodes,limit,mode,parent)
            %draw the link
            %check if it is ground
            if logical(obj.fixed)
                return;
            end
            node1=nodes(obj.nodes(1,1)).getNode;
            node2=nodes(obj.nodes(1,2)).getNode;
            if logical(obj.linearSpringValue)
                linearSpring=1;
            else
                linearSpring=0;
            end
            [x,y]=DrawLink.drawLine([node1 node2],obj.joints,limit,linearSpring);
            [xJoint1,yJoint1,xJoint2,yJoint2]=DrawLink.drawJoints([node1 node2],obj.joints,limit,obj.sliderAngle);
            [xGround1,yGround1,xGround2,yGround2]=DrawLink.drawGrounds([node1 node2],obj.joints,limit,obj.sliderAngle);
            
            if ~isempty(obj.crossSection) 
                lineWidthBeam=0.5;
            elseif logical(obj.compliantGroup(1)) || ~logical(sum(obj.drawJoints)) || sum(obj.drawJoints) < 0
                lineWidthBeam=3.5;
            else
                lineWidthBeam=2;
            end
            if isempty(obj.line)
                %new
                obj.line=gobjects(5,7);
            end
            %line
            if ~isempty(x)
                if ~isgraphics(obj.line(1,1))
                    obj.line(1,1)=plot(currentAxis,x,y,'LineWidth',lineWidthBeam,'LineStyle','-');
                else
                    obj.line(1,1).XData=x;
                    obj.line(1,1).YData=y;
                    obj.line(1,1).LineWidth=lineWidthBeam;
                    obj.line(1,1).LineStyle='-';
                end
            else
                delete(obj.line(1,1));
            end
            
            %joint 1
            if logical(obj.drawJoints(1,1))
                if ~isempty(xJoint1)
                    if ~isgraphics(obj.line(2,1))
                        obj.line(2,1)=patch(currentAxis,xJoint1,yJoint1,zeros(length(xJoint1),1),zeros(length(xJoint1),1),'FaceColor','none');
                    else
                        obj.line(2,1).XData=xJoint1;
                        obj.line(2,1).YData=yJoint1;
                    end
                else
                    delete(obj.line(2,1));
                end
            end
            
            %joint 2
            if logical(obj.drawJoints(1,2))
                if ~isempty(xJoint2)
                    if ~isgraphics(obj.line(3,1))
                        obj.line(3,1)=patch(currentAxis,xJoint2,yJoint2,zeros(length(xJoint2),1),zeros(length(xJoint2),1),'FaceColor','none');
                    else
                        obj.line(3,1).XData=xJoint2;
                        obj.line(3,1).YData=yJoint2;
                    end
                else
                    delete(obj.line(3,1));
                end
            end
            
            %ground 1
            if logical(obj.drawJoints(1,1))
                if ~isempty(xGround1)
                    if obj.joints(1,1)== Joint.GroundPin
                        if ~isgraphics(obj.line(4,1))
                            obj.line(4,1)=patch(currentAxis,[xGround1(1,1);xGround1(1,2);xGround1(1,3)],[yGround1(1,1);yGround1(1,2);yGround1(1,3)],zeros(3,1),zeros(3,1),'FaceColor','none');
                        else
                            obj.line(4,1).XData=[xGround1(1,1);xGround1(1,2);xGround1(1,3)];
                            obj.line(4,1).YData=[yGround1(1,1);yGround1(1,2);yGround1(1,3)];
                        end
                        index=2;
                    else
                        index=1;
                    end
                    for i=index:length(xGround1)
                        if ~isgraphics(obj.line(4,i))
                            obj.line(4,i)=plot(currentAxis,[xGround1(i,1) xGround1(i,2)],[yGround1(i,1) yGround1(i,2)]);
                        else
                            obj.line(4,i).XData=[xGround1(i,1) xGround1(i,2)];
                            obj.line(4,i).YData=[yGround1(i,1) yGround1(i,2)];
                        end
                    end
                else
                    delete(obj.line(4,:));
                end
            end
            %ground 2
            if logical(obj.drawJoints(1,2))
                if ~isempty(xGround2)
                    if obj.joints(1,2)==Joint.GroundPin
                        if ~isgraphics(obj.line(5,1))
                            obj.line(5,1)=patch(currentAxis,[xGround2(1,1);xGround2(1,2);xGround2(1,3)],[yGround2(1,1);yGround2(1,2);yGround2(1,3)],zeros(3,1),zeros(3,1),'FaceColor','none');
                        else
                            obj.line(5,1).XData=[xGround2(1,1);xGround2(1,2);xGround2(1,3)];
                            obj.line(5,1).YData=[yGround2(1,1);yGround2(1,2);yGround2(1,3)];
                        end
                        index=2;
                    else
                        index=1;
                    end
                    for i=index:length(xGround2)
                        if ~isgraphics(obj.line(5,i))
                            obj.line(5,i)=plot(currentAxis,[xGround2(i,1) xGround2(i,2)],[yGround2(i,1) yGround2(i,2)]);
                        else
                            obj.line(5,i).XData=[xGround2(i,1) xGround2(i,2)];
                            obj.line(5,i).YData=[yGround2(i,1) yGround2(i,2)];
                        end
                    end
                else
                    delete(obj.line(5,:));
                end
            end
            
            %color the beam
            if logical(obj.selected)
                color=Colors.Selected.getColor(); 
            else
                color=Colors.NormalLink.getColor();
            end
            
            obj=obj.colorLine(color);
            
            obj=obj.clickEvent(mode,parent);
            obj=obj.rightClickMenu(mode,parent);
        end
        
        function obj=rightClickMenu(obj,mode,parent)
            %right click menu
            handles=parent.getHandles();
            switch(mode)
                case Module.PostProcessing
                    menu= uicontextmenu(handles.mainGUI);
                    m1 = uimenu(menu,'Label',['Set Link ' num2str(obj.nodes(1)) '-'  num2str(obj.nodes(2))  ' Angle°  as X Axis'],'Callback',@(source,callbackdata)parent.setLinkXAxis(source,callbackdata,obj.id,1));
                    m2 = uimenu(menu,'Label',['Set Link ' num2str(obj.nodes(1)) '-'  num2str(obj.nodes(2)) ' Length  as X Axis'],'Callback',@(source,callbackdata)parent.setLinkXAxis(source,callbackdata,obj.id,2));
                    m3 = uimenu(menu,'Label',['Add Link ' num2str(obj.nodes(1)) '-'  num2str(obj.nodes(2)) ' Angle°  to Y Axis'],'Callback',@(source,callbackdata)parent.setLinkYAxis(source,callbackdata,obj.id,1));
                    m4 = uimenu(menu,'Label',['Add Link ' num2str(obj.nodes(1)) '-'  num2str(obj.nodes(2)) ' Length  to Y Axis'],'Callback',@(source,callbackdata)parent.setLinkYAxis(source,callbackdata,obj.id,2));
                    for i=1:5
                        for j=1:7
                            if isgraphics(obj.line(i,j))
                                obj.line(i,j).UIContextMenu=menu;
                            end
                        end
                    end
                case Module.FlexuralStiffnessSynthesis
                    if logical(obj.compliantGroup(1))
                        menu= uicontextmenu(handles.mainGUI);
                        m1 = uimenu(menu,'Label','Optimization Variable','Callback',@(source,callbackdata)parent.flexuralSynthesisRight(source,callbackdata,obj.compliantGroup(1)));
                        if isempty(parent.findFlexuralUnknownPos(obj.compliantGroup(1)))
                            m1.Checked='off';
                        else
                            m1.Checked='on';
                        end
                        for i=1:5
                            for j=1:7
                                if isgraphics(obj.line(i,j))
                                    obj.line(i,j).UIContextMenu=menu;
                                end
                            end
                        end
                    end
                case Module.BiStableSynthesis
                    if logical(obj.compliantGroup(1))
                        menu= uicontextmenu(handles.mainGUI);
                        handles=parent.getHandles();
                        if handles.synthesisModePopup.Value == 1
                            m1 = uimenu(menu,'Label','Optimization Variable','Callback',@(source,callbackdata)parent.bistableSynthesisRight(source,callbackdata,obj.compliantGroup(1)));
                        end
                        if isempty(parent.findBistableUnknownPos(obj.compliantGroup(1)))
                            m1.Checked='off';
                        else
                            m1.Checked='on';
                        end
                        for i=1:5
                            for j=1:7
                                if isgraphics(obj.line(i,j))
                                    obj.line(i,j).UIContextMenu=menu;
                                end
                            end
                        end
                    end
            end
        end
        
        
        function obj=colorLine(obj,colorBeam)
            %color the line
            if  isgraphics(obj.line(1,1))
                obj.line(1,1).Color=colorBeam;
            end
            %color the joints
            for i=2:3
                if isgraphics(obj.line(i,1))
                    obj.line(i,1).EdgeColor=colorBeam;
                end
            end
            %color the ground
            if isgraphics(obj.line(4,1))&& obj.joints(1,1)== Joint.GroundPin
                obj.line(4,1).FaceColor=colorBeam;
                for i=2:7
                    obj.line(4,i).Color=colorBeam;
                end
            elseif isgraphics(obj.line(4,1))
                for i=1:6
                    obj.line(4,i).Color=colorBeam;
                end
            end
            
            if isgraphics(obj.line(5,1)) && obj.joints(1,2)==Joint.GroundPin
                obj.line(5,1).FaceColor=colorBeam;
                for i=2:7
                    obj.line(5,i).Color=colorBeam;
                end
            elseif isgraphics(obj.line(5,1))
                for i=1:6
                    obj.line(5,i).Color=colorBeam;
                end
            end
        end
        
        function obj=deleteLinkDrawing(obj)
            %delete link drawing
            delete(obj.line);
            obj.line=gobjects(0);
        end
        
        function obj=clickEvent(obj,mode,parent)
            %node click event
            switch(mode)
                case Module.Free
                    fnc=@(src,evnt)obj.clickLinkFree(src,evnt,parent,parent.getPosFromID(obj.id));
                case Module.Range
                    fnc=@(src,evnt)obj.clickLinkRange(src,evnt,parent,parent.getPosFromID(obj.id)); 
                case Module.Load
                    fnc=[];
                case Module.Distance
                    fnc=@(src,evnt)obj.clickLinkDistance(src,evnt,parent,parent.getPosFromID(obj.id));
                case Module.MechanicalAdvantage
                    fnc=[];
                case Module.EnergyPlot
                    fnc=@(src,evnt)obj.clickLinkBistable(src,evnt,parent,parent.getPosFromID(obj.id));
                case Module.PostProcessing
                    fnc=[];
                case Module.KinematicSynthesis
                    fnc=@(src,evnt)obj.clickLinkKinematicSynthesis(src,evnt,parent,parent.getPosFromID(obj.id));
                case Module.FlexuralStiffnessSynthesis
                    fnc=@(src,evnt)obj.clickLinkFlexuralSynthesis(src,evnt,parent,parent.getPosFromID(obj.id));
                case Module.BiStableSynthesis
                    fnc=[];
                case Module.ConstantForceSynthesis
                    fnc=[];
            end
            if  isgraphics(obj.line(1,1))
                obj.line(1,1).ButtonDownFcn=fnc;
            end
            %same for joints
            if  isgraphics(obj.line(2,1))
                obj.line(2,1).ButtonDownFcn=fnc;
            end
            if  isgraphics(obj.line(3,1))
                obj.line(3,1).ButtonDownFcn=fnc;
            end
            
        end
        
        function obj=setSelected(obj,selected)
            %set the selected
            obj.selected=selected;
        end
        
        function clickLinkBistable(obj,src,evnt,parent,id)
            %click during a bistable analysis
            if logical(parent.getBistable().busy)
                return;
            end 
            popHandles=parent.getPopHandles();
            if popHandles.type.Value <=2
                %idList for the selected nodes
                for i=1:length(parent.getBistable().static.kinematic.rigidLinkList)
                    parent.setSelectLink4(i,0);
                end
                parent.setSelectLink4(id,1);
                popHandles.component.BackgroundColor=Colors.StatusComplete.getColor();
                popHandles.component.String='SELECTED';
                %plot
                parent.plotEverything();
                %try to plot distances
                if isnan(str2double(popHandles.target1.String))
                    popHandles.target1.String='-30';
                end
                if isnan(str2double(popHandles.target2.String))
                    popHandles.target2.String='30';
                end
                switch popHandles.type.Value
                    case 1
                        type=2;
                        target=1;
                    case 2
                        type=2;
                        target=2;
                    case 3
                        type=1;
                        target=1;
                end
                target(2:3)=[str2double(popHandles.target1.String),str2double(popHandles.target2.String)];
                parent.addBistableType(type,target);
                
            end
            figure(popHandles.energyGUI);
        end
        
        function clickLinkDistance(obj,src,evnt,parent,id)
            %click during a distance analysis
            if logical(parent.getDistanceAnalysis().busy)
                return;
            end 
            popHandles=parent.getPopHandles();
            if popHandles.type.Value <=2
                %idList for the selected nodes
                for i=1:length(parent.getDistanceAnalysis().static.kinematic.rigidLinkList)
                    parent.setSelectLink3(i,0);
                end
                parent.setSelectLink3(id,1);
                popHandles.component.BackgroundColor=Colors.StatusComplete.getColor();
                popHandles.component.String='SELECTED';
                %plot
                parent.plotEverything();
                %draw the distance
                if isnan(str2double(popHandles.target1.String))
                    popHandles.target1.String='30';
                end
                
                switch popHandles.type.Value
                    case 1
                        type=2;
                        target=1;
                    case 2
                        type=2;
                        target=2;
                    case 3
                        type=1;
                        target=1;
                    case 4
                        type=1;
                        target=2;
                end
                target(2)=str2double(popHandles.target1.String);
                parent.addDistanceAnalysisType(type,target);
            end
            figure(popHandles.distanceGUI);
        end
        
        function clickLinkRange(obj,src,evnt,parent,id)
            %click during a range 
            %check if link stiffness is shown
            popHandles=parent.getPopHandles();
            %idList for the selected nodes
            for i=1:length(parent.getKinematics().rigidLinkList)
                parent.setSelectLink2(i,0);
            end
            parent.setSelectLink2(id,1);
            popHandles.link.BackgroundColor=Colors.StatusComplete.getColor();
            popHandles.link.String='SELECTED';
            popHandles.relative.Value=0;
            popHandles.target2.String=[];
            %get the degrees of freedom for the selected link
            if obj.fixed ==1 || obj.masterID ~= obj.id || obj.degreesOfFreedom ==0 || (obj.degreesOfFreedom==1 && logical(obj.linearSpringValue))
                if logical(obj.linearSpringValue)
                    value=obj.length;
                    string='Length';
                elseif  obj.masterID ~= obj.id
                    value=obj.angle*180/pi;
                    string='Angle';
                else
                    value=obj.angle*180/pi;
                    string='-';
                end
            elseif strcmp(obj.type,'fixedPinSlider') || strcmp(obj.type,'pinSlider')
                value=obj.angle*180/pi;
                string={'Angle','Length'};
            elseif obj.joints(2) == Joint.Slider
                value=obj.length;
                string={'Length'};
            elseif obj.joints(2) == Joint.GroundSlider
                theta=obj.sliderAngle*pi/180;
                value=(obj.length*cos(obj.angle))*cos(theta)+(obj.length*sin(obj.angle))*sin(theta);
                string={'Length'};
                %initialGuess=(nodes(RB.point2).x-nodes(RB.point1).x)*cos(theta)+(nodes(RB.point2).y-nodes(RB.point1).y)*sin(theta);
            else
                if logical(obj.linearSpringValue)
                    value=obj.angle*180/pi;
                    string={'Angle','Length'};
                else
                    value=obj.angle*180/pi;
                    string='Angle';
                end
            end
            popHandles.type.Value=1;
            popHandles.type.String=string;
            popHandles.target1.String=num2str(value);
            %plot
            parent.plotEverything();
            figure(popHandles.rangeGUI);
        end
        
        function clickLinkFree(obj,src,evnt,parent,id)
            %click during a range
            handles=parent.getHandles();
            currentID=id;
            %idList for the selected nodes
            for i=1:length(parent.getKinematics().rigidLinkList)
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
            if obj.fixed ==1 || obj.masterID ~= obj.id || obj.degreesOfFreedom ==0 || (obj.degreesOfFreedom==1 && logical(obj.linearSpringValue))
                if logical(obj.linearSpringValue)
                    mode=2;
                elseif  obj.masterID ~= obj.id
                    mode=1;
                else
                    mode=0;
                end
            elseif strcmp(obj.type,'fixedPinSlider') || strcmp(obj.type,'pinSlider')
                mode=3;
            elseif obj.joints(2) == Joint.Slider
                mode=2;
            elseif obj.joints(2) == Joint.GroundSlider
                mode=4;
                %initialGuess=(nodes(RB.point2).x-nodes(RB.point1).x)*cos(theta)+(nodes(RB.point2).y-nodes(RB.point1).y)*sin(theta);
            else
                if logical(obj.linearSpringValue)
                    mode=3;
                else
                    mode=1;
                end
            end
            link=id;
            if obj.joints(1) == Joint.GroundPin || obj.joints(2) == Joint.GroundSlider
                node=1;
            else
                %get the mouse position
                mousePos=get(handles.designPlot,'CurrentPoint');
                mouseX=mousePos(1,1);
                mouseY=mousePos(1,2);
                node1=parent.getKinematics().nodes(obj.nodes(1)).getNode();
                node2=parent.getKinematics().nodes(obj.nodes(2)).getNode();
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
        
        function clickLinkKinematicSynthesis(obj,src,evnt,parent,id)
            %click during a kinematic synthesis
            %plot
            parent.updateKinematicSynthesisLink(id);
            parent.plotEverything();

        end
        
        function clickLinkFlexuralSynthesis(obj,src,evnt,parent,id)
            %click during a distance analysis
            if logical(parent.getBusy())
                return;
            end 
            popHandles=parent.getPopHandles();
            if popHandles.type.Value <=2
                %idList for the selected nodes
                for i=1:length(parent.getFlexuralStiffnessSynthesis().static.kinematic.rigidLinkList)
                    parent.setSelectLink6(i,0);
                end
                parent.setSelectLink6(id,1);
                popHandles.component.BackgroundColor=Colors.StatusComplete.getColor();
                popHandles.component.String='SELECTED';
                %plot
                parent.setSelectedLinkFlexural(id);
                parent.plotEverything();
                %try to plot distances
                if ~isnan(str2double(popHandles.target1.String))  && strcmp(popHandles.component.String,'SELECTED')
                    switch popHandles.type.Value
                        case 1
                            type=2;
                            target=1;
                        case 2
                            type=2;
                            target=2;
                        case 3
                            type=1;
                            target=1;
                    end
                    target(2)=str2double(popHandles.target1.String);
                    parent.addFlexuralType(type,target);
                end
            end
            figure(popHandles.energyGUI);
        end
        
    end
    
end

