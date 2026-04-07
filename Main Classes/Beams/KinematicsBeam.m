classdef KinematicsBeam < Beam
    
    properties
        type;
        linearSpring=0;
        equation=struct('xPre',[],'xFunc',[],'xKnown',0,'yPre',[],'yFunc',[],'yKnown',0);
        fixed=0;
        theta;
        length;
        %slider parameters
        sliderAngle=[];
        Ycos=[];
        Ysin=[];
        indeces;
    end
    
    methods
        function beam=KinematicsBeam(id,nodes,joints,sliderAngle,angle,length,crossSection,linearSpring,allNodes)
            %constructor for the kinematic beam class
            beam@Beam(id,0,0,nodes,joints,angle,length,crossSection);
            if ~isempty(sliderAngle)
                beam.sliderAngle=sliderAngle;
                lengthX=allNodes(nodes(1,2)).getNode().x-allNodes(nodes(1,1)).getNode().x  ;
                lengthY=allNodes(nodes(1,2)).getNode().y-allNodes(nodes(1,1)).getNode().y  ;
                Y=-sin(sliderAngle)*lengthX+cos(sliderAngle)*lengthY ;
                beam.Ycos=Y*cos(sliderAngle);
                beam.Ysin=Y*sin(sliderAngle);
            end
            beam.linearSpring=linearSpring;
            switch joints(1)
                case  Joint.GroundSlider
                    switch joints(2)
                        case Joint.GroundPin
                            beam.type='allFixedSlider';
                            beam.degreesOfFreedom=1;
                        case Joint.GroundWelded
                            beam.type='allFixedSlider';
                            beam.degreesOfFreedom=1;
                        otherwise
                            disp('Invalid beam.type');
                    end
                case Joint.GroundPin
                    switch joints(2)
                        case Joint.GroundSlider
                            beam.type='allFixedSlider';
                            beam.degreesOfFreedom=1;
                        case Joint.GroundPin
                            beam.type='allFixedPinBeam';
                            beam.degreesOfFreedom=0;
                            beam.fixed=1;
                        case Joint.Welded
                            beam.type='fixedWeldedPinBeam';
                            beam.degreesOfFreedom=0;
                        case Joint.Pin
                            beam.type='fixedPinBeam';
                            beam.degreesOfFreedom=1;
                        case Joint.Slider
                            beam.type='fixedPinSlider';
                            beam.degreesOfFreedom=2;
                        otherwise
                            disp('Invalid beam.type');
                    end
                case Joint.GroundWelded
                    switch joints(2)
                        case Joint.GroundSlider
                            beam.type='allFixedSlider';
                            beam.degreesOfFreedom=1;
                        case Joint.Slider
                            beam.type='fixedSlider';
                            beam.degreesOfFreedom=1;
                        case Joint.Welded
                            beam.type='fixedToWallBeam';
                            beam.degreesOfFreedom=0;
                        case Joint.Pin
                            beam.type='fixedToWallPinBeam';
                            beam.degreesOfFreedom=0;
                        otherwise
                            disp('Invalid beam.type');
                    end
                case Joint.Welded
                    switch joints(2)
                        case Joint.GroundPin
                            beam.type='fixedWeldedPinBeam';
                            beam.degreesOfFreedom=0;
                        case Joint.GroundWelded
                            beam.type='fixedToWallBeam';
                            beam.degreesOfFreedom=0;
                        case Joint.Welded
                            beam.type='weldedBeam';
                            beam.degreesOfFreedom=0;
                        case Joint.Pin
                            beam.type='weldedPinBeam';
                            beam.degreesOfFreedom=0;
                        case Joint.Slider
                            beam.type='slider';
                            beam.degreesOfFreedom=1;
                        otherwise
                            disp('Invalid beam.type');
                    end
                case Joint.Pin
                    switch joints(2)
                        case Joint.GroundWelded
                            beam.type='fixedToWallPinBeam';
                            beam.degreesOfFreedom=0;
                        case Joint.GroundPin
                            beam.type='fixedPinBeam';
                            beam.degreesOfFreedom=1;
                        case Joint.Welded
                            beam.type='weldedPinBeam';
                            beam.degreesOfFreedom=0;
                        case Joint.Pin
                            beam.type='pinBeam';
                            beam.degreesOfFreedom=1;
                        case Joint.Slider
                            beam.type='pinSlider';
                            beam.degreesOfFreedom=2;
                        otherwise
                            disp('Invalid beam.type');
                    end
                case Joint.Slider
                    switch joints(2)
                        case  Joint.GroundPin
                            beam.type='fixedPinSlider';
                            beam.degreesOfFreedom=2;
                        case Joint.GroundWelded
                            beam.type='fixedSlider';
                            beam.degreesOfFreedom=1;
                        case Joint.Welded
                            beam.type='slider';
                            beam.degreesOfFreedom=1;
                        case Joint.Pin
                            beam.type='pinSlider';
                            beam.degreesOfFreedom=2;
                        otherwise
                            disp('Invalid beam.type');
                    end
                otherwise
                    disp('Invalid beam.type');
            end
            %adjust the degrees of freedom  for linear springs
            if beam.fixed ==1 || beam.degreesOfFreedom ==0
                if logical(beam.linearSpring)
                    beam.degreesOfFreedom=1;
                end
            else
                if logical(beam.linearSpring)
                    beam.degreesOfFreedom=2;
                end
            end
            %create kinematic equations
            beam=beam.createEquation();
            beam.theta=beam.theta0;
            if beam.joints(1,2) == Joint.GroundSlider || beam.joints(1,1) == Joint.GroundSlider
                beam.length=(beam.length0*cos(beam.theta0))*cos(beam.sliderAngle)+(beam.length0*sin(beam.theta0))*sin(beam.sliderAngle);
            else
                beam.length=beam.length0;                
            end
        end
        
        function beam=createEquation(beam)
            %kinematic equations for kinematics beam
            if strcmp(beam.type,'allFixedPinBeam') ||strcmp(beam.type,'fixedPinBeam') || strcmp(beam.type,'pinBeam') || strcmp(beam.type,'fixedWeldedPinBeam') ||strcmp(beam.type,'weldedBeam')|| strcmp(beam.type,'weldedPinBeam')|| strcmp(beam.type,'fixedToWallBeam')|| strcmp(beam.type,'fixedToWallPinBeam')
                if ~logical(beam.linearSpring)
                    beam.equation.xPre=beam.length0;
                    beam.equation.xFunc=2;
                    beam.equation.yPre=beam.length0;
                    beam.equation.yFunc=1;
                else
                    beam.equation.xPre=1;
                    beam.equation.xFunc=6;
                    beam.equation.yPre=1;
                    beam.equation.yFunc=7;
                end
                %check if prismatic joined element
            elseif strcmp(beam.type,'allFixedSlider')
                beam.equation.xPre=1*cos(beam.sliderAngle);
                beam.equation.xFunc=3;
                beam.equation.xKnown=-1*beam.Ysin;
                beam.equation.yPre=1*sin(beam.sliderAngle);
                beam.equation.yFunc=3;
                beam.equation.yKnown=1*beam.Ycos;
            elseif strcmp(beam.type,'slider')
                beam.equation.xPre=1;
                beam.equation.xFunc=4;
                beam.equation.yPre=1;
                beam.equation.yFunc=5;
            elseif strcmp(beam.type,'fixedSlider')
                beam.equation.xPre=1*cos(beam.angle);
                beam.equation.xFunc=3;
                beam.equation.yPre=1*sin(beam.angle);
                beam.equation.yFunc=3;
            elseif strcmp(beam.type,'fixedPinSlider')
                beam.equation.xPre=1;
                beam.equation.xFunc=6;
                beam.equation.yPre=1;
                beam.equation.yFunc=7;
            elseif strcmp(beam.type,'pinSlider')
                beam.equation.xPre=1;
                beam.equation.xFunc=6;
                beam.equation.yPre=1;
                beam.equation.yFunc=7;
            else
                errordlg('Error on beam type');
            end
        end
        
        function value=getValues(beam,inputs)
            %get values from the input matrix
            if strcmp(beam.type,'slider')
                value(1)=inputs(beam.id);
                value(2)=beam.theta;
                value(3)=beam.theta0;
            elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider')
                value(1)=inputs(beam.id);
                value(2)=inputs(beam.id+1);
            elseif strcmp(beam.type,'allFixedSlider')
                value=inputs(beam.id);
            elseif  ~logical(beam.linearSpring)
                value=inputs(beam.id);
            else
                value(1)=inputs(beam.id);
                value(2)=inputs(beam.id+1);
            end
        end
        
         function initialGuess=calculateInitialGuess(beam)
            if beam.fixed ==1 || logical(beam.frontMasterID) || logical(beam.endMasterID) || beam.degreesOfFreedom ==0 || (beam.degreesOfFreedom==1 && logical(beam.linearSpring))
                if logical(beam.linearSpring)
                    initialGuess=beam.length;
                else
                    initialGuess=[];
                end
            elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider')
                initialGuess(1)=beam.theta;
                initialGuess(2)=beam.length;
            elseif beam.joints(2) == Joint.Slider
                initialGuess=beam.length;
            elseif beam.joints(2) == Joint.GroundSlider
                %theta=beam.sliderAngle;
                %initialGuess=(abs(beam.length)*cos(beam.theta))*cos(theta)+(abs(beam.length)*sin(beam.theta))*sin(theta);
                initialGuess=beam.length;
                %initialGuess=(nodes(RB.point2).x-nodes(RB.point1).x)*cos(theta)+(nodes(RB.point2).y-nodes(RB.point1).y)*sin(theta);
            else
                if logical(beam.linearSpring)
                    initialGuess(1)=beam.theta;
                    initialGuess(2)=beam.length;
                else
                    initialGuess=beam.theta;
                end
            end
        end
        
        function beam=updateBeam(beam,inputs,index)
            %update the beam
            beam.indeces=index(beam.id);
            if strcmp(beam.type,'slider')
                beam.length=inputs(beam.id);
                beam.indeces=0;
            elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider')
                beam.theta=inputs(beam.id);
                beam.length=inputs(beam.id+1);
            elseif strcmp(beam.type,'allFixedSlider')
                beam.length=inputs(beam.id);
                beam.indeces=0;
            elseif  ~logical(beam.linearSpring)
                beam.theta=inputs(beam.id);
              %  beam.theta0=inputs(beam.id);
            else
                beam.theta=inputs(beam.id);
                beam.length=inputs(beam.id+1); 
            end
        end
        
        
        function x = getX(beam,outputMatrix,distance)
            %x coordinate value
            value=beam.getValues(outputMatrix);
            x=beam.equation.xKnown+beam.equation.xPre*KinematicsBeam.preDefinedFunctions(beam.equation.xFunc,value,0);
            x=distance/100*x;
        end
        
        function y = getY(beam,outputMatrix,distance)
            %y coordinate value
            value=beam.getValues(outputMatrix);
            y=beam.equation.yKnown+beam.equation.yPre*KinematicsBeam.preDefinedFunctions(beam.equation.yFunc,value,0);
            y=distance/100*y;
        end
        
        function gradientX = getGradientX(beam,outputMatrix,index,distance)
            %gradient in x coordinates
            gradientX=zeros(max(index),1);
            value=beam.getValues(outputMatrix);
            xDerivative=beam.equation.xPre*KinematicsBeam.preDefinedFunctions(beam.equation.xFunc,value,1);
            if strcmp(beam.type,'slider')
                if logical(index(beam.id))
                    gradientX(index(beam.id),1)=xDerivative(1);
                end
                if logical(index(beam.frontMasterID))
                    gradientX(index(beam.frontMasterID),1)=xDerivative(2);
                end
            elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider') || logical(beam.linearSpring)
                if logical(index(beam.id))
                    gradientX(index(beam.id),1)=xDerivative(1);
                end
                if logical(index(beam.id+1))
                    gradientX(index(beam.id+1),1)=xDerivative(2);
                end
            elseif strcmp(beam.type,'allFixedSlider')
                if logical(index(beam.id))
                    gradientX(index(beam.id),1)=xDerivative(1);
                end
            else
                if logical(index(beam.id))
                    gradientX(index(beam.id),1)=xDerivative(1);
                end
                if logical(index(beam.id+1))
                    gradientX(index(beam.id+1),1)=xDerivative(2);
                end
            end
            gradientX=distance/100*gradientX;
        end
        
        function gradientY = getGradientY(beam,outputMatrix,index,distance)
            %gradient in y coordinates
            gradientY=zeros(max(index),1);
            value=beam.getValues(outputMatrix);
            yDerivative=beam.equation.yPre*KinematicsBeam.preDefinedFunctions(beam.equation.yFunc,value,1);
            if strcmp(beam.type,'slider')
                if logical(index(beam.id))
                    gradientY(index(beam.id),1)=yDerivative(1);
                end
                if logical(index(beam.frontMasterID))
                    gradientY(index(beam.frontMasterID),1)=yDerivative(2);
                end
            elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider') || logical(beam.linearSpring)
                if logical(index(beam.id))
                    gradientY(index(beam.id),1)=yDerivative(1);
                end
                if logical(index(beam.id+1))
                    gradientY(index(beam.id+1),1)=yDerivative(2);
                end
            elseif strcmp(beam.type,'allFixedSlider')
                if logical(index(beam.id))
                    gradientY(index(beam.id),1)=yDerivative(1);
                end
            else
                if logical(index(beam.id))
                    gradientY(index(beam.id),1)=yDerivative(1);
                end
                if logical(index(beam.id+1))
                    gradientY(index(beam.id+1),1)=yDerivative(2);
                end
            end
            gradientY=distance/100*gradientY;
        end
        
        function hessianX = getHessianX(beam,outputMatrix,index,distance)
            %gradient in x coordinates
            hessianX=zeros(max(index),max(index));
            value=beam.getValues(outputMatrix);
            xHessian=beam.equation.xPre*KinematicsBeam.preDefinedFunctions(beam.equation.xFunc,value,2);
            if strcmp(beam.type,'slider')
                if logical(index(beam.id))
                    hessianX(index(beam.id),index(beam.id))=xHessian(1);
                end
                if logical(index(beam.frontMasterID))
                    hessianX(index(beam.frontMasterID),index(beam.frontMasterID))=xHessian(2);
                end
                if logical(index(beam.id)) && logical(index(beam.frontMasterID))
                    hessianX(index(beam.id),index(beam.frontMasterID))=xHessian(3);
                    hessianX(index(beam.frontMasterID),index(beam.id))=xHessian(3);
                end
            elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider') || logical(beam.linearSpring)
                if logical(index(beam.id))
                    hessianX(index(beam.id),index(beam.id))=xHessian(1);
                end
                if logical(index(beam.id+1))
                    hessianX(index(beam.id+1),index(beam.id+1))=xHessian(2);
                end
                if logical(index(beam.id)) && logical(index(beam.id+1))
                    hessianX(index(beam.id),index(beam.id+1))=xHessian(3);
                    hessianX(index(beam.id+1),index(beam.id))=xHessian(3);
                end
            elseif strcmp(beam.type,'allFixedSlider')
                if logical(index(beam.id))
                    hessianX(index(beam.id),index(beam.id))=xHessian(1);
                end
            else
                if logical(index(beam.id))
                    hessianX(index(beam.id),index(beam.id))=xHessian(1);
                end
            end
            hessianX=distance/100*hessianX;
        end
        
        function hessianY = getHessianY(beam,outputMatrix,index,distance)
            %gradient in y coordinates
            hessianY=zeros(max(index),max(index));
            value=beam.getValues(outputMatrix);
            yHessian=beam.equation.yPre*KinematicsBeam.preDefinedFunctions(beam.equation.yFunc,value,2);
            if strcmp(beam.type,'slider')
                if logical(index(beam.id))
                    hessianY(index(beam.id),index(beam.id))=yHessian(1);
                end
                if logical(index(beam.frontMasterID))
                    hessianY(index(beam.frontMasterID),index(beam.frontMasterID))=yHessian(2);
                end
                if logical(index(beam.id)) && logical(index(beam.frontMasterID))
                    hessianY(index(beam.id),index(beam.frontMasterID))=yHessian(3);
                    hessianY(index(beam.frontMasterID),index(beam.id))=yHessian(3);
                end
            elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider') || logical(beam.linearSpring)
                if logical(index(beam.id))
                    hessianY(index(beam.id),index(beam.id))=yHessian(1);
                end
                if logical(index(beam.id+1))
                    hessianY(index(beam.id+1),index(beam.id+1))=yHessian(2);
                end
                if logical(index(beam.id)) && logical(index(beam.id+1))
                    hessianY(index(beam.id),index(beam.id+1))=yHessian(3);
                    hessianY(index(beam.id+1),index(beam.id))=yHessian(3);
                end
            elseif strcmp(beam.type,'allFixedSlider')
                if logical(index(beam.id))
                    hessianY(index(beam.id),index(beam.id))=yHessian(1);
                end
            else
                if logical(index(beam.id))
                    hessianY(index(beam.id),index(beam.id))=yHessian(1);
                end
            end
            hessianY=distance/100*hessianY;
        end
        %%
        %Energy Functions
        %prbBeams=struct('length0',[],'length',[],'theta',0,'theta0',0,'linearSpring',0,'index',[0 0]);
        %torsionSprings=struct('magnitude',[]);
        function energy = getEnergy(beam,outputMatrix,index,workspace)
            %get energy
            value=beam.getValues(outputMatrix);
            energy=0;
            if logical(beam.linearSpring)
                energy=1/2*(beam.linearSpring*workspace.linearSpringFactor())*((value(2)-beam.length0)*workspace.lengthFactor())^2;
            end
        end
        
        function gradient=getGradient(beam,outputMatrix,index,workspace)
            %get energy gradient
            value=beam.getValues(outputMatrix);
            gradient=zeros(max(index),1);
            if logical(beam.linearSpring) && logical(index(beam.id+1))
                gradient(index(beam.id+1),1)=(beam.linearSpring*workspace.linearSpringFactor())*((value(2)-beam.length0)*workspace.lengthFactor()^2);
            end
        end
        
        function hessian=getHessian(beam,outputMatrix,index,workspace)
            %get energy hessian
            hessian=sparse([],[],[],max(index),max(index));
            if logical(beam.linearSpring) && logical(index(beam.id+1))
                hessian(index(beam.id+1),index(beam.id+1))=(beam.linearSpring*workspace.linearSpringFactor()*workspace.lengthFactor()^2);
            end
        end
        
        function [angle,index]=getAngle(beam,outputMatrix,indexMatrix,~)
            %get angle and index
            angle=outputMatrix(beam.id);
            index=indexMatrix(beam.id);
        end
        

        
        %% miscellaneous functions
         function beam=drawLinkNoGUI(beam,currentAxis,nodes,limit,workspace)
            %draw the link
            %check if it is ground
            if logical(beam.fixed)
                return;
            end
            node1=nodes(beam.nodes(1,1)).getNode;
            node2=nodes(beam.nodes(1,2)).getNode;
            [x,y]=DrawLink.drawLine([node1 node2],beam.joints,limit,beam.linearSpring);
            [xJoint1,yJoint1,xJoint2,yJoint2]=DrawLink.drawJoints([node1 node2],beam.joints,limit,beam.sliderAngle*180/pi);
            [xGround1,yGround1,xGround2,yGround2]=DrawLink.drawGrounds([node1 node2],beam.joints,limit,beam.sliderAngle*180/pi);
            
            if ~isempty(beam.crossSection)
                lineWidthBeam=0.5;
            elseif  ~logical(sum(beam.drawJoints)) || sum(beam.drawJoints) < 0
                lineWidthBeam=3.5;
            else
                lineWidthBeam=2;
            end
            if isempty(beam.line)
                %new
                beam.line=gobjects(5,7);
            end
            %line
            if ~isempty(x)
                if ~isgraphics(beam.line(1,1))
                    beam.line(1,1)=plot(currentAxis,x,y,'LineWidth',lineWidthBeam,'LineStyle','-');
                else
                    beam.line(1,1).XData=x;
                    beam.line(1,1).YData=y;
                    beam.line(1,1).LineWidth=lineWidthBeam;
                    beam.line(1,1).LineStyle='-';
                end
            else
                delete(beam.line(1,1));
            end
            
            %joint 1
            if logical(beam.drawJoints(1,1))
                if ~isempty(xJoint1)
                    if ~isgraphics(beam.line(2,1))
                        beam.line(2,1)=patch(currentAxis,xJoint1,yJoint1,zeros(length(xJoint1),1),zeros(length(xJoint1),1),'FaceColor','none');
                    else
                        beam.line(2,1).XData=xJoint1;
                        beam.line(2,1).YData=yJoint1;
                    end
                else
                    delete(beam.line(2,1));
                end
            end
            
            %joint 2
            if logical(beam.drawJoints(1,2))
                if ~isempty(xJoint2)
                    if ~isgraphics(beam.line(3,1))
                        beam.line(3,1)=patch(currentAxis,xJoint2,yJoint2,zeros(length(xJoint2),1),zeros(length(xJoint2),1),'FaceColor','none');
                    else
                        beam.line(3,1).XData=xJoint2;
                        beam.line(3,1).YData=yJoint2;
                    end
                else
                    delete(beam.line(3,1));
                end
            end
            
            %ground 1
            if logical(beam.drawJoints(1,1))
                if ~isempty(xGround1)
                    if beam.joints(1,1)== Joint.GroundPin
                        if ~isgraphics(beam.line(4,1))
                            beam.line(4,1)=patch(currentAxis,[xGround1(1,1);xGround1(1,2);xGround1(1,3)],[yGround1(1,1);yGround1(1,2);yGround1(1,3)],zeros(3,1),zeros(3,1),'FaceColor','none');
                        else
                            beam.line(4,1).XData=[xGround1(1,1);xGround1(1,2);xGround1(1,3)];
                            beam.line(4,1).YData=[yGround1(1,1);yGround1(1,2);yGround1(1,3)];
                        end
                        index=2;
                    else
                        index=1;
                    end
                    for i=index:length(xGround1)
                        if ~isgraphics(beam.line(4,i))
                            beam.line(4,i)=plot(currentAxis,[xGround1(i,1) xGround1(i,2)],[yGround1(i,1) yGround1(i,2)]);
                        else
                            beam.line(4,i).XData=[xGround1(i,1) xGround1(i,2)];
                            beam.line(4,i).YData=[yGround1(i,1) yGround1(i,2)];
                        end
                    end
                else
                    delete(beam.line(4,:));
                end
            end
            %ground 2
            if logical(beam.drawJoints(1,2))
                if ~isempty(xGround2)
                    if beam.joints(1,2)==Joint.GroundPin
                        if ~isgraphics(beam.line(5,1))
                            beam.line(5,1)=patch(currentAxis,[xGround2(1,1);xGround2(1,2);xGround2(1,3)],[yGround2(1,1);yGround2(1,2);yGround2(1,3)],zeros(3,1),zeros(3,1),'FaceColor','none');
                        else
                            beam.line(5,1).XData=[xGround2(1,1);xGround2(1,2);xGround2(1,3)];
                            beam.line(5,1).YData=[yGround2(1,1);yGround2(1,2);yGround2(1,3)];
                        end
                        index=2;
                    else
                        index=1;
                    end
                    for i=index:length(xGround2)
                        if ~isgraphics(beam.line(5,i))
                            beam.line(5,i)=plot(currentAxis,[xGround2(i,1) xGround2(i,2)],[yGround2(i,1) yGround2(i,2)]);
                        else
                            beam.line(5,i).XData=[xGround2(i,1) xGround2(i,2)];
                            beam.line(5,i).YData=[yGround2(i,1) yGround2(i,2)];
                        end
                    end
                else
                    delete(beam.line(5,:));
                end
            end
            
            %color the beam
            if logical(beam.selected)
                color=Colors.Selected.getColor();
            else
                color=Colors.NormalLink.getColor();
            end
            
            beam=beam.colorLine(color);
        end
       
        
        function beam=drawLink(beam,currentAxis,nodes,limit,mode,parent)
            %draw the link
            %check if it is ground
            if logical(beam.fixed)
                return;
            end
            node1=nodes(beam.nodes(1,1)).getNode;
            node2=nodes(beam.nodes(1,2)).getNode;
            [x,y]=DrawLink.drawLine([node1 node2],beam.joints,limit,beam.linearSpring);
            [xJoint1,yJoint1,xJoint2,yJoint2]=DrawLink.drawJoints([node1 node2],beam.joints,limit,beam.sliderAngle*180/pi);
            [xGround1,yGround1,xGround2,yGround2]=DrawLink.drawGrounds([node1 node2],beam.joints,limit,beam.sliderAngle*180/pi);
            
            if ~isempty(beam.crossSection)
                lineWidthBeam=0.5;
            elseif  ~logical(sum(beam.drawJoints)) || sum(beam.drawJoints) < 0
                lineWidthBeam=3.5;
            else
                lineWidthBeam=2;
            end
            if isempty(beam.line)
                %new
                beam.line=gobjects(5,7);
            end
            %line
            if ~isempty(x)
                if ~isgraphics(beam.line(1,1))
                    beam.line(1,1)=plot(currentAxis,x,y,'LineWidth',lineWidthBeam,'LineStyle','-');
                else
                    beam.line(1,1).XData=x;
                    beam.line(1,1).YData=y;
                    beam.line(1,1).LineWidth=lineWidthBeam;
                    beam.line(1,1).LineStyle='-';
                end
            else
                delete(beam.line(1,1));
            end
            
            %joint 1
            if logical(beam.drawJoints(1,1))
                if ~isempty(xJoint1)
                    if ~isgraphics(beam.line(2,1))
                        beam.line(2,1)=patch(currentAxis,xJoint1,yJoint1,zeros(length(xJoint1),1),zeros(length(xJoint1),1),'FaceColor','none');
                    else
                        beam.line(2,1).XData=xJoint1;
                        beam.line(2,1).YData=yJoint1;
                    end
                else
                    delete(beam.line(2,1));
                end
            end
            
            %joint 2
            if logical(beam.drawJoints(1,2))
                if ~isempty(xJoint2)
                    if ~isgraphics(beam.line(3,1))
                        beam.line(3,1)=patch(currentAxis,xJoint2,yJoint2,zeros(length(xJoint2),1),zeros(length(xJoint2),1),'FaceColor','none');
                    else
                        beam.line(3,1).XData=xJoint2;
                        beam.line(3,1).YData=yJoint2;
                    end
                else
                    delete(beam.line(3,1));
                end
            end
            
            %ground 1
            if logical(beam.drawJoints(1,1))
                if ~isempty(xGround1)
                    if beam.joints(1,1)== Joint.GroundPin
                        if ~isgraphics(beam.line(4,1))
                            beam.line(4,1)=patch(currentAxis,[xGround1(1,1);xGround1(1,2);xGround1(1,3)],[yGround1(1,1);yGround1(1,2);yGround1(1,3)],zeros(3,1),zeros(3,1),'FaceColor','none');
                        else
                            beam.line(4,1).XData=[xGround1(1,1);xGround1(1,2);xGround1(1,3)];
                            beam.line(4,1).YData=[yGround1(1,1);yGround1(1,2);yGround1(1,3)];
                        end
                        index=2;
                    else
                        index=1;
                    end
                    for i=index:length(xGround1)
                        if ~isgraphics(beam.line(4,i))
                            beam.line(4,i)=plot(currentAxis,[xGround1(i,1) xGround1(i,2)],[yGround1(i,1) yGround1(i,2)]);
                        else
                            beam.line(4,i).XData=[xGround1(i,1) xGround1(i,2)];
                            beam.line(4,i).YData=[yGround1(i,1) yGround1(i,2)];
                        end
                    end
                else
                    delete(beam.line(4,:));
                end
            end
            %ground 2
            if logical(beam.drawJoints(1,2))
                if ~isempty(xGround2)
                    if beam.joints(1,2)==Joint.GroundPin
                        if ~isgraphics(beam.line(5,1))
                            beam.line(5,1)=patch(currentAxis,[xGround2(1,1);xGround2(1,2);xGround2(1,3)],[yGround2(1,1);yGround2(1,2);yGround2(1,3)],zeros(3,1),zeros(3,1),'FaceColor','none');
                        else
                            beam.line(5,1).XData=[xGround2(1,1);xGround2(1,2);xGround2(1,3)];
                            beam.line(5,1).YData=[yGround2(1,1);yGround2(1,2);yGround2(1,3)];
                        end
                        index=2;
                    else
                        index=1;
                    end
                    for i=index:length(xGround2)
                        if ~isgraphics(beam.line(5,i))
                            beam.line(5,i)=plot(currentAxis,[xGround2(i,1) xGround2(i,2)],[yGround2(i,1) yGround2(i,2)]);
                        else
                            beam.line(5,i).XData=[xGround2(i,1) xGround2(i,2)];
                            beam.line(5,i).YData=[yGround2(i,1) yGround2(i,2)];
                        end
                    end
                else
                    delete(beam.line(5,:));
                end
            end
            
            %color the beam
            if logical(beam.selected)
                color=Colors.Selected.getColor();
            else
                color=Colors.NormalLink.getColor();
            end
            
            beam=beam.colorLine(color);
            
            beam=beam.clickEvent(mode,parent);
            beam=beam.rightClickMenu(mode,parent);
        end
        
        
        function clickLinkBistable(beam,src,evnt,parent,id)
            %click during a bistable analysis
            if logical(parent.getBistable().busy)
                return;
            end
            popHandles=parent.getPopHandles();
            if ~isempty(popHandles) && popHandles.type.Value <=2
                %idList for the selected nodes
                for i=1:length(parent.getBistable().static.kinematic.allBeams)
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
                figure(popHandles.energyGUI);
            end
            
        end
        
        function clickLinkDistance(beam,src,evnt,parent,id)
            %click during a distance analysis
            if logical(parent.getDistanceAnalysis().busy)
                return;
            end
            popHandles=parent.getPopHandles();
            if ~isempty(popHandles) && popHandles.type.Value <=2
                %idList for the selected nodes
                for i=1:length(parent.getDistanceAnalysis().static.kinematic.allBeams)
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
                figure(popHandles.distanceGUI);
            end
        end
        
        function clickLinkRange(beam,src,evnt,parent,id)
            %click during a range
            %check if link stiffness is shown
            popHandles=parent.getPopHandles();
            if ~isempty(popHandles)
                %idList for the selected nodes
                for i=1:length(parent.getKinematics().allBeams)
                    parent.setSelectLink2(i,0);
                end
                parent.setSelectLink2(id,1);
                popHandles.link.BackgroundColor=Colors.StatusComplete.getColor();
                popHandles.link.String='SELECTED';
                popHandles.relative.Value=0;
                popHandles.target2.String=[];
                %get the degrees of freedom for the selected link
                if beam.fixed ==1 || logical(beam.frontMasterID) || logical(beam.endMasterID) || beam.degreesOfFreedom ==0 || (beam.degreesOfFreedom==1 && logical(beam.linearSpring))
                    if logical(beam.linearSpring)
                        value=beam.length;
                        string='Length';
                    elseif  logical(beam.frontMasterID) || logical(beam.endMasterID)
                        value=beam.theta*180/pi;
                        string='Angle';
                    else
                        value=beam.angle*180/pi;
                        string='-';
                    end
                elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider')
                    value=beam.theta*180/pi;
                    string={'Angle','Length'};
                elseif beam.joints(2) == Joint.Slider
                    value=beam.length;
                    string={'Length'};
                elseif beam.joints(2) == Joint.GroundSlider
                    theta=beam.sliderAngle;
                    value=(beam.length*cos(beam.theta))*cos(theta)+(beam.length*sin(beam.theta))*sin(theta);
                    string={'Length'};
                    %initialGuess=(nodes(RB.point2).x-nodes(RB.point1).x)*cos(theta)+(nodes(RB.point2).y-nodes(RB.point1).y)*sin(theta);
                else
                    if logical(beam.linearSpring)
                        value=beam.theta*180/pi;
                        string={'Angle','Length'};
                    else
                        value=beam.theta*180/pi;
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
        end
        
        
        function clickLinkKinematicSynthesis(beam,src,evnt,parent,id)
            %click during a kinematic synthesis
            %plot
            parent.updateKinematicSynthesisLink(id);
            parent.plotEverything();
            
        end
        
        function clickLinkFlexuralSynthesis(beam,src,evnt,parent,id)
            %click during a distance analysis
            if logical(parent.getBusy())
                return;
            end
            popHandles=parent.getPopHandles();
            if ~isempty(popHandles) && popHandles.type.Value <=2
                %idList for the selected nodes
                for i=1:length(parent.getFlexuralStiffnessSynthesis().static.kinematic.allBeams)
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
                figure(popHandles.energyGUI);
            end
        end
        
    end
    methods(Static)
        
        function output = preDefinedFunctions(index,value,isDiff )
            %FUNCTIONS returns commonly used functions
            switch index
                case 0
                    if isDiff== 1
                        %gradient
                        output(1)=0;
                        output(2)=0;
                    elseif isDiff== 2
                        %hessian
                        output(1)=0;
                        output(2)=0;
                        output(3)=0;
                    else
                        output=0;
                    end
                case 1
                    if isDiff== 1
                        output= cos(value(1));
                    elseif isDiff== 2
                        %hessian
                        output= -sin(value(1));
                    else
                        output=sin(value(1));
                    end
                case 2
                    if isDiff==1
                        output= -sin(value(1));
                    elseif isDiff== 2
                        %hessian
                        output= -cos(value(1));
                    else
                        output=cos(value(1));
                    end
                case 3
                    if isDiff == 1
                        output=1;
                    elseif isDiff== 2
                        %hessian
                        output= 0;
                    else
                        output=value(1);
                    end
                case 4
                    if isDiff == 1
                        output(1)=sign(value(1))*cos(value(2)+value(3));
                        output(2)=-value(1)*sin(value(2)+value(3));
                    elseif isDiff== 2
                        %hessian
                        output(1)=0;
                        output(2)=-value(1)*cos(value(2)+value(3));
                        output(3)=-sign(value(1))*sin(value(2)+value(3));
                    else
                        output=value(1)*cos(value(2)+value(3));
                    end
                case 5
                    if isDiff == 1
                        output(1)=sign(value(1))*sin(value(2)+value(3));
                        output(2)=value(1)*cos(value(2)+value(3));
                    elseif isDiff== 2
                        %hessian
                        output(1)=0;
                        output(2)=-value(1)*sin(value(2)+value(3));
                        output(3)=sign(value(1))*cos(value(2)+value(3));
                    else
                        output=value(1)*sin(value(2)+value(3));
                    end
                case 6
                    value(2)=abs(value(2));
                    if isDiff == 1
                        output(1)=-value(2)*sin(value(1));
                        output(2)=sign(value(2))*cos(value(1));
                    elseif isDiff== 2
                        %hessian
                        output(1)=-value(2)*cos(value(1));
                        output(2)=0;
                        output(3)=-sign(value(2))*sin(value(1));
                    else
                        output=value(2)*cos(value(1));
                    end
                case 7
                    value(2)=abs(value(2));
                    if isDiff == 1
                        output(1)=value(2)*cos(value(1));
                        output(2)=sign(value(2))*sin(value(1));
                    elseif isDiff== 2
                        %hessian
                        output(1)=-value(2)*sin(value(1));
                        output(2)=0;
                        output(3)=sign(value(2))*cos(value(1));
                    else
                        output=value(2)*sin(value(1));
                    end
                otherwise
                    errordlg('Error on Predefined Kinematic Equations');
            end
            
        end
        
    end
    
end
