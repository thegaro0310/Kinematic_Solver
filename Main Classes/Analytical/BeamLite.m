classdef BeamLite < EulerBeam
    properties
        %force drawing handles
        forceLine=gobjects(2,2);
        forceText=gobjects(2,1);
        %moment drawing handles
        momentLine=gobjects(1,2);
        momentText=gobjects(1,1);
    end
        
    methods
        function beam=BeamLite(E,length,thickness,width)
            beam@EulerBeam(length,E,thickness,width,0.0,0.0,0.0);
        end
        
        
        function beam=setLoad(beam,Fx,Fy,M)
            %set the loads
            beam.Fx=Fx;
            beam.Fy=Fy;
            beam.M=M;
        end
        
        %%%
        %data plotting functions
        %%
        
        function beam=draw(beam,figureHandle,drawArray)
            if sum(drawArray) == 1
                %one plot only 
                switch find(drawArray == 1,1)
                    case 1
                        %draw shape
                        beam.drawShape(figureHandle);
                    case 2
                        %draw curvature
                        beam.drawCurvatureFigure(figureHandle);
                    case 3
                        %draw internal bending moment
                        beam.drawMomentFigure(figureHandle);
                    case 4
                        %draw internal bending stress
                        beam.drawStressFigure(figureHandle);
                end
            elseif sum(drawArray) > 0
                %multiple plots
                %check which plots are selected
                subIndex=1;
                axes(figureHandle);
                if logical(drawArray(1))
                    sP=subplot(sum(drawArray),1,subIndex);
                    beam.drawShape(sP);
                    subIndex=subIndex+1;
                end
                if logical(drawArray(2))
                    sP=subplot(sum(drawArray),1,subIndex);
                    beam.drawCurvatureFigure(sP);
                    subIndex=subIndex+1;
                end
                if logical(drawArray(3))
                    sP=subplot(sum(drawArray),1,subIndex);
                    beam.drawMomentFigure(sP);
                    subIndex=subIndex+1;
                end
                if logical(drawArray(4))
                    sP=subplot(sum(drawArray),1,subIndex);
                    beam.drawStressFigure(sP);
                end
            else
                %draw shape
                beam.drawShape(figureHandle);
            end
        end
        
        function drawShape(beam,figureHandle)
            %draw the shape and the stress information
            hold(figureHandle,'on');
            surf('XData',[beam.x' beam.x'],...
                'YData',[beam.y' beam.y'],...
                'ZData',zeros(length(beam.y(:)),2),...
                'CData',[beam.internalStress'/1e6 beam.internalStress'/1e6],...
                'FaceColor','none',...
                'EdgeColor','interp',...
                'Marker','none','LineWidth',5,'Parent',figureHandle);
            axis(figureHandle,beam.lengthBeam*[-1.5 1.5 -1.5 1.5]);
            colormap jet;
            c=colorbar(figureHandle,'south');
            c.Label.String = 'Stress (MPa)';
            xlabel(figureHandle,'x (m)');ylabel(figureHandle,'y (m)');
            %draw force and moment
            node.x=beam.x(end);
            node.y=beam.y(end);
            beam.drawForce(node,figureHandle,beam.lengthBeam*1.25,beam.Fx,beam.Fy);
            beam.drawMoment(node,figureHandle,beam.lengthBeam*1.25,beam.M);
            hold(figureHandle,'off');
        end
        
        function drawCurvatureFigure(beam,figureHandle)
            %draw the curvature information
            s=linspace(0,beam.lengthBeam,length(beam.curvature));
            plot(figureHandle,s,beam.curvature);
            xlabel(figureHandle,'s');ylabel(figureHandle,'Curvature (1/m^{-1})');
        end
        
        function drawMomentFigure(beam,figureHandle)
            %draw internal bending moment
            s=linspace(0,beam.lengthBeam,length(beam.curvature));
            plot(figureHandle,s,beam.internalMoment);
            xlabel(figureHandle,'s');ylabel(figureHandle,'Internal Moment (N.m)');
        end
        
        function drawStressFigure(beam,figureHandle)
            %draw internal bending stress
            s=linspace(0,beam.lengthBeam,length(beam.curvature));
            plot(figureHandle,s,beam.internalStress/1e6);
            xlabel(figureHandle,'s');ylabel(figureHandle,'Absolute Internal Bending Stress (MPa)');
        end
        
        function beam=drawForce(beam,node,ax,limit,xValue,yValue)
            %function for drawing forces
            unit='N';
            stretch=limit/20;
            %force in x direction
            if xValue > 0
                %plot the straight forceLine
                if isempty(beam.forceLine(1,1)) || ~isgraphics(beam.forceLine(1,1))
                    beam.forceLine(1,1)=plot(ax,[node.x node.x+stretch*3],[node.y node.y],'linewidth',2.5);
                else
                    beam.forceLine(1,1).XData=[node.x node.x+stretch*3];
                    beam.forceLine(1,1).YData=[node.y node.y];
                end
                
                angle=-pi/2;
                rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
                pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch];
                for i=1:3
                    pointNew=rotation*[pointsX(i);pointsY(i)];
                    pointsX(i)=node.x+stretch*3+pointNew(1);pointsY(i)=node.y+pointNew(2);
                end
                %plot the hat and the text
                if isempty(beam.forceLine(1,2)) || ~isgraphics(beam.forceLine(1,2))
                    beam.forceLine(1,2)=plot(ax,pointsX,pointsY,'linewidth',2.5);
                    %plot the text
                    beam.forceText(1)=text(ax,node.x+stretch,node.y-stretch, [num2str(abs(xValue)),' ',unit]);
                else
                    beam.forceLine(1,2).XData=pointsX;
                    beam.forceLine(1,2).YData=pointsY;
                    beam.forceText(1).Position=[node.x+stretch,node.y-stretch];
                    beam.forceText(1).String=[num2str(abs(xValue)),' ',unit];
                end
                %negative x force
            elseif xValue < 0
                %plot the straight forceLine
                if isempty(beam.forceLine(1,1)) || ~isgraphics(beam.forceLine(1,1))
                    beam.forceLine(1,1)=plot(ax,[node.x node.x-stretch*3],[node.y node.y],'linewidth',2.5);
                else
                    beam.forceLine(1,1).XData=[node.x node.x-stretch*3];
                    beam.forceLine(1,1).YData=[node.y node.y];
                end
                
                angle=pi/2;
                rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
                pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch];
                for i=1:3
                    pointNew=rotation*[pointsX(i);pointsY(i)];
                    pointsX(i)=node.x-stretch*3+pointNew(1);
                    pointsY(i)=node.y+pointNew(2);
                end
                %plot the hat and the text
                if isempty(beam.forceLine(1,2)) || ~isgraphics(beam.forceLine(1,2))
                    beam.forceLine(1,2)=plot(ax,pointsX,pointsY,'linewidth',2.5);
                    %plot the text
                    beam.forceText(1)=text(ax,node.x-2*stretch,node.y-stretch, [num2str(abs(xValue)),' ',unit]);
                else
                    beam.forceLine(1,2).XData=pointsX;
                    beam.forceLine(1,2).YData=pointsY;
                    beam.forceText(1).Position=[node.x-2*stretch,node.y-stretch];
                    beam.forceText(1).String=[num2str(abs(xValue)),' ',unit];
                end
            end
            %positive y force
            if yValue > 0
                %plot the straight forceLine
                if isempty(beam.forceLine(2,1)) || ~isgraphics(beam.forceLine(2,1))
                    beam.forceLine(2,1)=plot(ax,[node.x node.x],[node.y node.y+stretch*3],'linewidth',2.5);
                else
                    beam.forceLine(2,1).XData=[node.x node.x];
                    beam.forceLine(2,1).YData=[node.y node.y+stretch*3];
                end
                
                angle=0;
                rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
                pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch];
                for i=1:3
                    pointNew=rotation*[pointsX(i);pointsY(i)];
                    pointsX(i)=node.x+pointNew(1);pointsY(i)=node.y+pointNew(2)+stretch*3;
                end
                %plot the hat and the text
                if isempty(beam.forceLine(2,2)) || ~isgraphics(beam.forceLine(2,2))
                    beam.forceLine(2,2)=plot(ax,pointsX,pointsY,'linewidth',2.5);
                    %plot the text
                    beam.forceText(2)=text(ax,node.x,node.y+stretch*3.75, [num2str(abs(yValue)),' ',unit],'HorizontalAlignment','center');
                else
                    beam.forceLine(2,2).XData=pointsX;
                    beam.forceLine(2,2).YData=pointsY;
                    beam.forceText(2).Position=[node.x,node.y+stretch*3.75];
                    beam.forceText(2).String=[num2str(abs(yValue)),' ',unit];
                end
                %negative y force
            elseif yValue < 0
                %plot the straight forceLine
                if isempty(beam.forceLine(2,1)) || ~isgraphics(beam.forceLine(2,1))
                    beam.forceLine(2,1)=plot(ax,[node.x node.x],[node.y node.y+-stretch*3],'linewidth',2.5);
                else
                    beam.forceLine(2,1).XData=[node.x node.x];
                    beam.forceLine(2,1).YData=[node.y node.y-stretch*3];
                end
                angle=pi;
                rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
                pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
                pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch];
                for i=1:3
                    pointNew=rotation*[pointsX(i);pointsY(i)];
                    pointsX(i)=node.x+pointNew(1);pointsY(i)=node.y-stretch*3+pointNew(2);
                end
                %plot the hat and the text
                if isempty(beam.forceLine(2,2)) || ~isgraphics(beam.forceLine(2,2))
                    beam.forceLine(2,2)=plot(ax,pointsX,pointsY,'linewidth',2.5);
                    %plot the text
                    beam.forceText(2)=text(ax,node.x,node.y-stretch*3.75, [num2str(abs(yValue)),' ',unit],'HorizontalAlignment','center');
                else
                    beam.forceLine(2,2).XData=pointsX;
                    beam.forceLine(2,2).YData=pointsY;
                    beam.forceText(2).Position=[node.x,node.y-stretch*3.75];
                    beam.forceText(2).String=[num2str(abs(yValue)),' ',unit];
                end
            end
            %color
            if ~isempty(beam.forceText(1)) && isgraphics(beam.forceText(1))
                beam.forceText(1).Color=Colors.Blue.getColor();
            end
            if ~isempty(beam.forceText(2)) && isgraphics(beam.forceText(2))
                beam.forceText(2).Color=Colors.Blue.getColor();
            end
            for i=1:size(beam.forceLine,1)
                for j=1:2
                    if ~isempty(beam.forceLine(i,j)) && isgraphics(beam.forceLine(i,j))
                        beam.forceLine(i,j).Color=Colors.Blue.getColor();
                    end
                end
            end
        end
        
        function beam=drawMoment(beam,node,ax,limit,magnitude)
            %function for drawing moment
            unit='N.m';
            stretch=limit/20;
            angles=linspace(3.1*pi/2,pi/2+2*pi,100);
            x=cos(angles)*stretch;
            y=sin(angles)*stretch*1.5;
            %negative moment
            if magnitude <0
                rotation=[cos(pi) -sin(pi);sin(pi) cos(pi)];
                for i=1:length(x)
                    pointNew=rotation*[x(i);y(i)];
                    x(i)=node.x*1.02+pointNew(1);
                    y(i)=node.y+pointNew(2);
                end
            else
                %if positive
                x=x+node.x*0.98;
                y=y+node.y;
            end
            if isempty(beam.momentLine(1,1)) || ~isgraphics(beam.momentLine(1,1))
                beam.momentLine(1,1)=plot(ax,x,y,'linewidth',2.5);
            else
                beam.momentLine(1,1).XData=x;
                beam.momentLine(1,1).YData=y;
            end
            %straight line is drawn
            %now draw the hat
            if magnitude >0
                multiplier=1.2;
            else
                multiplier=0.8;
            end
            angle=atan2(y(end)-y(end-1),x(end)-x(end-1))-pi*multiplier/2;
            rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];
            pointsX=[-cos(pi/3)*0.5*stretch 0 cos(pi/3)*0.5*stretch] ;
            pointsY=[-0.5*sin(pi/3)*stretch 0 -0.5*sin(pi/3)*stretch] ;
            for i=1:3
                pointNew=rotation*[pointsX(i);pointsY(i)];
                if magnitude >0
                    pointsX(i)=x(end)+pointNew(1);pointsY(i)=y(end)+pointNew(2);
                else
                    pointsX(i)=x(1)+pointNew(1);pointsY(i)=y(1)+pointNew(2);
                end
            end
            if isempty(beam.momentLine(1,2)) || ~isgraphics(beam.momentLine(1,2))
                beam.momentLine(1,2)=plot(ax,pointsX,pointsY,'linewidth',2.5);
                %plot the text
                beam.momentText(1)=text(ax,pointsX(end)*1.05,pointsY(end), [num2str(abs(magnitude)),' ',unit]);
            else
                beam.momentLine(1,2).XData=pointsX;
                beam.momentLine(1,2).YData=pointsY;
                beam.momentText(1).Position=[pointsX(end)*1.05,pointsY(end)];
                beam.momentText(1).String=[num2str(abs(magnitude)),' ',unit];
            end
            %color
            if ~isempty(beam.momentText(1)) && isgraphics(beam.momentText(1))
                beam.momentText(1).Color=Colors.PanelText.getColor();
            end
            for i=1:size(beam.momentLine,1)
                for j=1:2
                    if ~isempty(beam.momentLine(i,j)) && isgraphics(beam.momentLine(i,j))
                        beam.momentLine(i,j).Color=Colors.PanelText.getColor();
                    end
                end
            end
        end
        
    end
    
end

