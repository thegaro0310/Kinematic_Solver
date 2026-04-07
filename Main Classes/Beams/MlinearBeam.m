classdef MlinearBeam < Beam
    properties
        linearBeams=LinearBeam.empty;
        segments;
    end
    
    methods
        function beam=MlinearBeam(id,nodes,joints,angle,lengthBeam,crossSection)
            %constructor for the CBCM beam class
            beam@Beam(id,0,0,nodes,joints,angle,lengthBeam,crossSection);
            for i=1:length(crossSection.segments)
                beam.linearBeams(i)=LinearBeam(id+(i-1)*4,nodes,joints,angle,lengthBeam*crossSection.segments(i),crossSection);
                beam.linearBeams(i).noOfData=round(crossSection.segments(i)*100);
            end
            beam.degreesOfFreedom=length(crossSection.segments)*4;
            beam.segments=length(crossSection.segments);
        end
        
        function beam=updateBeam(beam,inputs,index)
            %update bcm beams from inputs
            for i=1:beam.segments
                beam.linearBeams(i)=beam.linearBeams(i).updateBeam(inputs,index);
            end
        end
        
        function initialGuess=getInitialGuess(beam,id)
            %get initial guess for the beam
            initialGuess=beam.linearBeams(id).getInitialGuess();
        end
        
        %%
        %Kinematic Functions
        
        function x = getX(beam,outputMatrix,distance)
            %x coordinate value
            if distance < 0
                x=-(beam.getX(outputMatrix,100)-beam.getX(outputMatrix,100+distance));
            elseif distance == 0
                x=0;
            else
                currentDistance=0;
                x=0;
                for i=1:beam.segments
                    singleDistance=beam.linearBeams(i).length0/beam.length0*100;
                    if currentDistance + singleDistance >= distance
                        modifier=(distance-currentDistance)/singleDistance;
                        x=x+modifier*beam.linearBeams(i).getX(outputMatrix,100);
                        return;
                    else
                        x=x+beam.linearBeams(i).getX(outputMatrix,100);
                        currentDistance=currentDistance+singleDistance;
                    end
                end
            end
        end
        
        function y = getY(beam,outputMatrix,distance)
            %y coordinate value
            if distance < 0
                y=-(beam.getY(outputMatrix,100)-beam.getY(outputMatrix,100+distance));
            elseif distance == 0
                y=0;
            else
                currentDistance=0;
                y=0;
                for i=1:beam.segments
                    singleDistance=beam.linearBeams(i).length0/beam.length0*100;
                    if currentDistance + singleDistance >= distance
                        modifier=(distance-currentDistance)/singleDistance;
                        y=y+modifier*beam.linearBeams(i).getY(outputMatrix,100);
                        return;
                    else
                        y=y+beam.linearBeams(i).getY(outputMatrix,100);
                        currentDistance=currentDistance+singleDistance;
                    end
                end
            end
        end
        
        function gradientX = getGradientX(beam,outputMatrix,index,distance)
            %gradient in x coordinates
            if distance < 0
                gradientX=-(beam.getGradientX(outputMatrix,index,100)-beam.getGradientX(outputMatrix,index,100+distance));
            elseif distance == 0
                gradientX=zeros(max(index),1);
            else
                currentDistance=0;
                gradientX=zeros(max(index),1);
                for i=1:beam.segments
                    singleDistance=beam.linearBeams(i).length0/beam.length0*100;
                    if currentDistance + singleDistance >= distance
                        modifier=(distance-currentDistance)/singleDistance;
                        gradientX=gradientX+modifier*beam.linearBeams(i).getGradientX(outputMatrix,index,100);
                        return;
                    else
                        gradientX=gradientX+beam.linearBeams(i).getGradientX(outputMatrix,index,100);
                        currentDistance=currentDistance+singleDistance;
                    end
                end
            end
        end
        
        function gradientY = getGradientY(beam,outputMatrix,index,distance)
            %gradient in y coordinates
            if distance < 0
                gradientY=-(beam.getGradientY(outputMatrix,index,100)-beam.getGradientY(outputMatrix,index,100+distance));
            elseif distance == 0
                gradientY=zeros(max(index),1);
            else
                currentDistance=0;
                gradientY=zeros(max(index),1);
                for i=1:beam.segments
                    singleDistance=beam.linearBeams(i).length0/beam.length0*100;
                    if currentDistance + singleDistance >= distance
                        modifier=(distance-currentDistance)/singleDistance;
                        gradientY=gradientY+modifier*beam.linearBeams(i).getGradientY(outputMatrix,index,100);
                        return;
                    else
                        gradientY=gradientY+beam.linearBeams(i).getGradientY(outputMatrix,index,100);
                        currentDistance=currentDistance+singleDistance;
                    end
                end
            end
        end
        
        function hessianX = getHessianX(beam,outputMatrix,index,distance)
            %hessian in x coordinates
            if distance < 0
                hessianX=-(beam.getHessianX(outputMatrix,index,100)-beam.getHessianX(outputMatrix,index,100+distance));
            elseif distance == 0
                hessianX=sparse(max(index),max(index));
            else
                currentDistance=0;
                hessianX=sparse(max(index),max(index));
                for i=1:beam.segments
                    singleDistance=beam.linearBeams(i).length0/beam.length0*100;
                    if currentDistance + singleDistance >= distance
                        modifier=(distance-currentDistance)/singleDistance;
                        hessianX=hessianX+modifier*beam.linearBeams(i).getHessianX(outputMatrix,index,100);
                        return;
                    else
                        hessianX=hessianX+beam.linearBeams(i).getHessianX(outputMatrix,index,100);
                        currentDistance=currentDistance+singleDistance;
                    end
                end
            end
        end
        
        function hessianY = getHessianY(beam,outputMatrix,index,distance)
            %gradient in x coordinates
            if distance < 0
                hessianY=-(beam.getHessianY(outputMatrix,index,100)-beam.getHessianY(outputMatrix,index,100+distance));
            elseif distance == 0
                hessianY=sparse(max(index),max(index));
            else
                currentDistance=0;
                hessianY=sparse(max(index),max(index));
                for i=1:beam.segments
                    singleDistance=beam.linearBeams(i).length0/beam.length0*100;
                    if currentDistance + singleDistance >= distance
                        modifier=(distance-currentDistance)/singleDistance;
                        hessianY=hessianY+modifier*beam.linearBeams(i).getHessianY(outputMatrix,index,100);
                        return;
                    else
                        hessianY=hessianY+beam.linearBeams(i).getHessianY(outputMatrix,index,100);
                        currentDistance=currentDistance+singleDistance;
                    end
                end
            end
        end
        
        
        %%
        %Energy Functions
        function energy = getEnergy(beam,workspace)
            energy=0;
            for i=1:beam.segments
                energy=energy+beam.linearBeams(i).getEnergy(workspace);
            end
        end
        
        function gradient=getGradient(beam,index,workspace)
            gradient=zeros(max(index),1);
            for i=1:beam.segments
                gradient=gradient+beam.linearBeams(i).getGradient(index,workspace);
            end
        end
        
        function hessian=getHessian(beam,index,workspace)
            hessian=sparse(max(index),max(index));
            for i=1:beam.segments
                hessian=hessian+beam.linearBeams(i).getHessian(index,workspace);
            end
        end
        
        function beam=updateInternalID(beam,modifier)
            %adjust bcm ids
            for i=1:beam.segments
                beam.linearBeams(i).id=beam.linearBeams(i).id+modifier;
            end
        end
        
        function [angle,index]=getAngle(beam,distance)
            currentDistance=0;
            for i=1:beam.segments
                singleDistance=beam.linearBeams(i).length0/beam.length0*100;
                if currentDistance + singleDistance >= distance
                    angle=beam.linearBeams(i).theta; 
                    index=beam.linearBeams(i).index(4); 
                    return;
                end
                currentDistance=currentDistance+singleDistance;
            end
            %numerical error check
            angle=beam.linearBeams(end).theta;
            index=beam.linearBeams(end).index(4);
        end
        
        %%
        %miscelleneous files
        
        function beam=getStressShape(beam,workspace)
            %fill the stress values
            beam.stressValues=[];
            beam.xValues=[];
            beam.yValues=[];
            for i=1:beam.segments
                [x,y,stress]=beam.linearBeams(i).getShape(workspace);
                beam.stressValues=[beam.stressValues;stress];
                beam.xValues=[beam.xValues;x];
                beam.yValues=[beam.yValues;y];
            end
        end
        
        function force=getForce(beam,workspace)
            %get tip force in local coordinate
            deltaX=beam.linearBeams(end).deltaX*workspace.lengthFactor();
            deltaY=beam.linearBeams(end).deltaY*workspace.lengthFactor();
            deltaTheta=beam.linearBeams(end).getDeltaTheta();
            EI=beam.linearBeams(end).crossSection.E*workspace.EFactor()*beam.linearBeams(end).crossSection.getI()*workspace.lengthFactor()^4;
            L=beam.linearBeams(end).length0*workspace.lengthFactor();
            t=beam.linearBeams(end).crossSection.thickness*workspace.lengthFactor();
            %to global coordinate
            forces=[12*deltaX*EI/(t^2*L);...
                6*EI/L^3*(2*deltaY-L*deltaTheta);....
                -2*EI/L^2*(3*deltaY-2*L*deltaTheta)];
            force(1,1)=(forces(1)*cos(beam.linearBeams(end).masterAngle)-forces(2)*sin(beam.linearBeams(end).masterAngle))/workspace.forceFactor();
            force(1,2)=(forces(1)*sin(beam.linearBeams(end).masterAngle)+forces(2)*cos(beam.linearBeams(end).masterAngle))/workspace.forceFactor();
            force(1,3)=forces(3)/workspace.forceFactor()/workspace.lengthFactor();
        end
        
        function beam=drawLink(beam,currentAxis,nodes,limit,mode,parent)
            %draw the link
            node1=nodes(beam.nodes(1,1)).getNode;
            node2=nodes(beam.nodes(1,2)).getNode;
            %[x,y]=DrawLink.drawLine([Point(node1.x,node1.y),Point(node2.x,node2.y)],[Joint.Pin Joint.Pin],limit,0);
            x={};y={};bendingStress={};
            deltaX=node1.x;
            deltaY=node1.y;
            for i=1:beam.segments
                [xx,yy,stress]=beam.linearBeams(i).getShape(parent.getWorkspace());
                x{i}=xx+deltaX;
                y{i}=yy+deltaY;
                bendingStress{i}=stress;
                deltaX=deltaX+xx(end);
                deltaY=deltaY+yy(end);
            end

            [xJoint1,yJoint1,xJoint2,yJoint2]=DrawLink.drawJoints([node1 node2],beam.joints,limit,[]);
            [xGround1,yGround1,xGround2,yGround2]=DrawLink.drawGrounds([node1 node2],beam.joints,limit,[]);
            
            if ~isempty(beam.crossSection)
                lineWidthBeam=beam.crossSection.thickness;
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
                    for i=1:beam.segments
                        beam.line(1,i)=surf('XData',[x{i} x{i}],...
                            'YData',[y{i} y{i}],...
                            'ZData',zeros(length(y{i}),2),...
                            'CData',[bendingStress{i}/1e6 bendingStress{i}/1e6],...
                            'FaceColor','none',...
                            'EdgeColor','interp',...
                            'Marker','none','LineWidth',lineWidthBeam,...
                            'Parent',currentAxis);
                    end
                    
                    colormap(currentAxis,'jet');
                    beam.colorBar=colorbar(currentAxis,'east');
                    beam.colorBar.Label.String = 'Stress (MPa)';
                else
                    for i=1:beam.segments
                        beam.line(1,i).XData=[x{i} x{i}];
                        beam.line(1,i).YData=[y{i} y{i}];
                        beam.line(1,i).CData=[bendingStress{i}/1e6 bendingStress{i}/1e6];
                    end
                end
            else
                for i=1:length(beam.prbBeams)
                    delete(beam.line(1,i));
                end
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
        
        function beam=drawLinkNoGUI(beam,currentAxis,nodes,limit,workspace)
            %draw the link
            node1=nodes(beam.nodes(1,1)).getNode;
            node2=nodes(beam.nodes(1,2)).getNode;
            %[x,y]=DrawLink.drawLine([Point(node1.x,node1.y),Point(node2.x,node2.y)],[Joint.Pin Joint.Pin],limit,0);
            x={};y={};bendingStress={};
            deltaX=node1.x;
            deltaY=node1.y;
            for i=1:beam.segments
                [xx,yy,stress]=beam.linearBeams(i).getShape(workspace);
                x{i}=xx+deltaX;
                y{i}=yy+deltaY;
                bendingStress{i}=stress;
                deltaX=deltaX+xx(end);
                deltaY=deltaY+yy(end);
            end

            [xJoint1,yJoint1,xJoint2,yJoint2]=DrawLink.drawJoints([node1 node2],beam.joints,limit,[]);
            [xGround1,yGround1,xGround2,yGround2]=DrawLink.drawGrounds([node1 node2],beam.joints,limit,[]);
            
            if ~isempty(beam.crossSection)
                lineWidthBeam=beam.crossSection.thickness;
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
                    for i=1:beam.segments
                        beam.line(1,i)=surf('XData',[x{i} x{i}],...
                            'YData',[y{i} y{i}],...
                            'ZData',zeros(length(y{i}),2),...
                            'CData',[bendingStress{i}/1e6 bendingStress{i}/1e6],...
                            'FaceColor','none',...
                            'EdgeColor','interp',...
                            'Marker','none','LineWidth',lineWidthBeam,...
                            'Parent',currentAxis);
                    end
                    
                    colormap(currentAxis,'jet');
                    beam.colorBar=colorbar(currentAxis,'east');
                    beam.colorBar.Label.String = 'Stress (MPa)';
                else
                    for i=1:beam.segments
                        beam.line(1,i).XData=[x{i} x{i}];
                        beam.line(1,i).YData=[y{i} y{i}];
                        beam.line(1,i).CData=[bendingStress{i}/1e6 bendingStress{i}/1e6];
                    end
                end
            else
                for i=1:length(beam.prbBeams)
                    delete(beam.line(1,i));
                end
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
            
            %color the beam
            if logical(beam.selected)
                color=Colors.Selected.getColor();
            else
                color=Colors.NormalLink.getColor();
            end
            
            beam=beam.colorLine(color);
        end
        
        function obj=rightClickMenu(obj,mode,parent)
            switch(mode)
                
                case Module.PostProcessing
                    handles=parent.getHandles();
                    menu= uicontextmenu(handles.mainGUI);
                    uimenu(menu,'Label','Set Link-Tip Fx as X Axis','Callback',@(source,callbackdata)parent.setLinkXAxis(source,callbackdata,obj.id,3));
                    uimenu(menu,'Label','Set Link-Tip Fx as Y Axis','Callback',@(source,callbackdata)parent.setLinkYAxis(source,callbackdata,obj.id,3));
                    uimenu(menu,'Label','Set Link-Tip Fy as X Axis','Callback',@(source,callbackdata)parent.setLinkXAxis(source,callbackdata,obj.id,4));
                    uimenu(menu,'Label','Set Link-Tip Fy as Y Axis','Callback',@(source,callbackdata)parent.setLinkYAxis(source,callbackdata,obj.id,4));
                    uimenu(menu,'Label','Set Link-Tip M as X Axis','Callback',@(source,callbackdata)parent.setLinkXAxis(source,callbackdata,obj.id,5));
                    uimenu(menu,'Label','Set Link-Tip M as Y Axis','Callback',@(source,callbackdata)parent.setLinkYAxis(source,callbackdata,obj.id,5));
                    for i=1:size(obj.line,1)
                        for j=1:size(obj.line,2)
                            if isgraphics(obj.line(i,j))
                                obj.line(i,j).UIContextMenu=menu;
                            end
                        end
                    end
            end
        end
    end
end
