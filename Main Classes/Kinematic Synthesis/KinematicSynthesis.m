classdef KinematicSynthesis
    %kinematic synthesis class
    
    properties
        kinematic=Kinematics.empty;
        parent;
        origin;
        coupler=1;
        driver=1;
        currentCouplerPoints=[];
        currentCouplerLine=gobjects(0);
        originalTargetCouplerPoints=[];
        targetCouplerPoints=[];
        targetCouplerLine=gobjects(0);
        targetCouplerLine2=gobjects(0);
        targetCouplerText=gobjects(0);
        curveLoaded=0;
        selectedPoint=0;
        noOfDescriptors=5;
        deltas=[0.5 1];
        nodeRanges=[];
        mode=1;
        selectedNode=0;
        closedCurve=0;
    end
    
    methods
        function obj=KinematicSynthesis(links,nodes,parent)
            %constructor
            obj.kinematic=Kinematics(links,nodes);
            %prapare the kinematics
            obj.kinematic=obj.kinematic.mainPreparation();
            %find coupler and driver
            obj=obj.findCouplerDriver();
            obj.currentCouplerPoints=obj.findCurrentCoupler();
            obj.targetCouplerPoints=obj.currentCouplerPoints;
            obj.originalTargetCouplerPoints=obj.targetCouplerPoints;
            obj.curveLoaded=0;
            %set the node limits
            limit=parent.getWorkspace().getLimit()*0.5;
            obj.nodeRanges(1,:)=repmat(-limit,1,length(obj.kinematic.nodes));
            obj.nodeRanges(2,:)=repmat(-limit,1,length(obj.kinematic.nodes));
            obj.nodeRanges(3,:)=repmat(limit,1,length(obj.kinematic.nodes));
            obj.nodeRanges(4,:)=repmat(limit,1,length(obj.kinematic.nodes));
        end
        
        function obj=drawAll(obj,mainFig,limit,mode,parent)
            %draw all links
            handles=parent.getHandles();
            %parent.setWorkspace(obj.changeWorkspace(parent));
            obj.kinematic=obj.kinematic.drawAll(mainFig,limit,mode,parent);
            %plot nodes
            for i=1:length(obj.kinematic.nodes)
                %plot nodes
                obj.kinematic.nodes(i)=obj.kinematic.nodes(i).drawNode(mainFig,limit,mode,parent);
            end
            %color the node and the link
            if obj.mode ~= 3 
                obj.kinematic.nodes(obj.coupler)=obj.kinematic.nodes(obj.coupler).colorNode(Colors.Point1.getColor());
                obj.kinematic.rigidLinkList(obj.driver)=obj.kinematic.rigidLinkList(obj.driver).colorLine(Colors.Point1.getColor());
            end
            %plot current coupler
            delete(obj.currentCouplerLine);
            if obj.mode ~= 3 
                obj.currentCouplerLine=plot(obj.currentCouplerPoints(1,:),obj.currentCouplerPoints(2,:),'LineWidth',0.3,'Color',Colors.Blue.getColor());
            end
            %plot target coupler
            delete(obj.targetCouplerLine);
            delete(obj.targetCouplerText);
            delete(obj.targetCouplerLine2);
            %target coupler curve
            if obj.mode > 1 && obj.mode ~= 3 
                if logical(obj.closedCurve)
                    obj.targetCouplerLine2=plot([obj.targetCouplerPoints(1,:) obj.targetCouplerPoints(1,1)],[obj.targetCouplerPoints(2,:) obj.targetCouplerPoints(2,1)],'LineStyle','--','Color',Colors.Constrained.getColor());
                else
                    obj.targetCouplerLine2=plot(obj.targetCouplerPoints(1,:),obj.targetCouplerPoints(2,:),'LineStyle','--','Color',Colors.Constrained.getColor());
                end
                for i=1:size(obj.targetCouplerPoints,2)
                    obj.targetCouplerLine(i)=plot(obj.targetCouplerPoints(1,i),obj.targetCouplerPoints(2,i),'LineStyle','none','Color',Colors.Constrained.getColor(),'Marker','+');
                    obj.targetCouplerLine(i).ButtonDownFcn=@(src,evnt)obj.clickCouplerKinematicSynthesis(src,evnt,parent,i);
                    obj.targetCouplerText(i)=text(obj.targetCouplerPoints(1,i),obj.targetCouplerPoints(2,i)+limit/25,num2str(i));
                    obj.targetCouplerText(i).ButtonDownFcn=@(src,evnt)obj.clickCouplerKinematicSynthesis(src,evnt,parent,i);
                    if logical(handles.dataLabels.Value)
                        obj.targetCouplerText(i).Color=Colors.Constrained.getColor();
                    else
                        obj.targetCouplerText(i).Color='none';
                    end
                end 
            end
        end
        
        
        function obj=changePanel(obj,parent)
            %change panel
            handles=parent.getHandles();
            parent.setBusy(0);
            handles.distancePanel.Visible='off';
            handles.unknownForcePanel.Visible='off';
            handles.previousNextPanel.Visible='off';
            handles.constantForceSynthesisMainPanel.Visible='off';
            handles.rangeValuesPanel.Visible='off';
            handles.couplerPanel.Visible='off';
            handles.kinSynthesisPanel.Visible='off';
            handles.nextButton1.Visible='on';
            handles.nextButton2.Visible='on';
            handles.previousButton1.Visible='on';
            handles.previousButton2.Visible='on';
            handles.analysisTab.Visible='off';
            handles.kinSynthesisPanel.Visible='off';
            switch obj.mode
                case 1 %select coupler and 
                    handles.unknownForcePanel.Visible='on';
                    handles.previousButton1.Visible='off';
                    handles.unknownForcePanel.Title='Driver and Coupler';
                    handles.helpStatic.String='Select the driver link and the coupler node by clicking nodes and links.';
                    handles.constantForceTopStatic.String='Select the driver link and the coupler node by clicking nodes and links.';
                case 2%torsion spring stiffness
                    handles.unknownForcePanel.Visible='off'; 
                    handles.couplerPanel.Visible='on';
                    handles.couplerPanel.Position(1:2)=[96 29];
                    handles.previousNextPanel.Visible='on';
                    handles.previousNextPanel.Position(1:2)=[100 23];
                    handles.helpStatic.String='Import target coupler curve or drag the current coupler points.';
                case 3%linear spring stiffness
                    handles.unknownForcePanel.Visible='on';
                    handles.unknownForcePanel.Title='Optimization Ranges - Nodes';
                    handles.helpStatic.String='Select optimization variable ranges by clicking design nodes.';
                    handles.constantForceTopStatic.String='Select optimization variable ranges by clicking design nodes.';
                case 4%compliant member
                    handles.analysisTab.Visible='on';
                    handles.kinSynthesisPanel.Visible='on';
                    handles.previousNextPanel.Visible='on';
                    handles.nextButton2.Visible='off';
                    handles.previousNextPanel.Position(1:2)=[110 18];
            end
            handles.panelListStatic.String=[num2str(obj.mode) '/4'];
            handles.panelListStatic2.String=[num2str(obj.mode) '/4'];
        end
        
        
        function fillData(obj,parent)
            %fill data during range limits
            handles=parent.getHandles();
            handles.lowerConstantText.Enable='on';
            handles.upperConstantText.Enable='on';
            handles.currentConstantText.Visible='off';
            handles.currentConstantStatic.Visible='off';
            handles.addConstantButton.String='Add';
            if handles.typeConstantPopup.Value == 1
                %x
                handles.lowerConstantText.String=num2str(obj.nodeRanges(1,obj.selectedNode));
                handles.upperConstantText.String=num2str(obj.nodeRanges(3,obj.selectedNode));
            else
                %y
                handles.lowerConstantText.String=num2str(obj.nodeRanges(2,obj.selectedNode));
                handles.upperConstantText.String=num2str(obj.nodeRanges(4,obj.selectedNode));
            end
        end
        
        function obj=saveData(obj,parent)
            %save range data
            handles=parent.getHandles();
            lowerLimit=str2double(handles.lowerConstantText.String);
            upperLimit=str2double(handles.upperConstantText.String);
            if handles.typeConstantPopup.Value == 1
                %x
                if ~isnan(lowerLimit)
                    obj.nodeRanges(1,obj.selectedNode)=lowerLimit;
                else
                    handles.lowerConstantText.String=num2str(obj.nodeRanges(1,obj.selectedNode));
                end
                if ~isnan(upperLimit)
                    obj.nodeRanges(3,obj.selectedNode)=upperLimit;
                else
                    handles.upperConstantText.String=num2str(obj.nodeRanges(3,obj.selectedNode));
                end
            else
                %y
                if ~isnan(lowerLimit)
                    obj.nodeRanges(2,obj.selectedNode)=lowerLimit;
                else
                    handles.lowerConstantText.String=num2str(obj.nodeRanges(2,obj.selectedNode));
                end
                if ~isnan(upperLimit)
                    obj.nodeRanges(4,obj.selectedNode)=upperLimit;
                else
                    handles.upperConstantText.String=num2str(obj.nodeRanges(4,obj.selectedNode));
                end    
            end
        end
        
        function obj=processCouplerCurve(obj,limit,value)
            %process coupler curve to have more or less points
            if value > 0
                %we will increase number of points
                maxDistance=0.02*limit/value;
                increment=0;
                obj.targetCouplerPoints=obj.originalTargetCouplerPoints;
                for i=1:size(obj.originalTargetCouplerPoints,2)-1
                    distance=sqrt((obj.originalTargetCouplerPoints(1,i+1)-obj.originalTargetCouplerPoints(1,i))^2+(obj.originalTargetCouplerPoints(2,i+1)-obj.originalTargetCouplerPoints(2,i))^2);
                    if distance > maxDistance
                        obj.targetCouplerPoints=[obj.targetCouplerPoints(:,1:increment+i) [(obj.originalTargetCouplerPoints(1,i+1)+obj.originalTargetCouplerPoints(1,i))/2;...
                            (obj.originalTargetCouplerPoints(2,i+1)+obj.originalTargetCouplerPoints(2,i))/2 ] obj.targetCouplerPoints(:,increment+i+1:end)];
                        increment=increment+1;
                    end
                end
            elseif value < 0
                %we will decrease number of points
                minDistance=abs(value)*limit*0.2; 
                currentPoint=1;
                delete=[];
                for i=2:size(obj.originalTargetCouplerPoints,2)
                    distance=sqrt((obj.originalTargetCouplerPoints(1,currentPoint)-obj.originalTargetCouplerPoints(1,i))^2+(obj.originalTargetCouplerPoints(2,currentPoint)-obj.originalTargetCouplerPoints(2,i))^2);
                    if distance < minDistance
                        delete(end+1)=i;
                    else
                        currentPoint=i;
                    end
                end
                obj.targetCouplerPoints=obj.originalTargetCouplerPoints;
                obj.targetCouplerPoints(:,delete)=[];
                obj.targetCouplerLine2.XData=obj.targetCouplerPoints(1,:);
                obj.targetCouplerLine2.YData=obj.targetCouplerPoints(2,:);
            end
        end
        
        function clickCouplerKinematicSynthesis(obj,src,evnt,parent,id)
            %click event on a coupler node for kinematic synthesis
            if obj.mode == 2
                parent.setSelectedCoupler(id);
            end
        end
        
        
        function obj=setSelectedNode(obj,id,select)
            %set selected value of a node
            obj.kinematic.nodes(id)=obj.kinematic.nodes(id).setSelected(select);
        end
        
        function obj=setSelectedLink(obj,id,select)
            %set selected value of a link
            obj.kinematic.rigidLinkList(id).selected=select;
        end
        
        function obj=deleteAll(obj)
            %delete all links
            obj.kinematic=obj.kinematic.deleteAll();
            %delete all nodes
            for i=1:length(obj.kinematic.nodes)
                obj.kinematic.nodes(i)=obj.kinematic.nodes(i).deleteNodeDrawing();
            end
            delete(obj.currentCouplerLine);
            delete(obj.targetCouplerLine);
            delete(obj.targetCouplerText);
            delete(obj.targetCouplerLine2);
        end
        
        function obj=findCouplerDriver(obj)
            %find coupler point and driver link
            couplerArray=zeros(1,length(obj.kinematic.nodes));
            for i=1:length(obj.kinematic.rigidLinkList)
                %node 1
                if couplerArray(obj.kinematic.rigidLinkList(i).nodes(1,1)) >=0
                    %check if it is ground
                    if obj.kinematic.rigidLinkList(i).joints(1,1) == Joint.GroundWelded || obj.kinematic.rigidLinkList(i).joints(1,1) == Joint.GroundPin
                        couplerArray(obj.kinematic.rigidLinkList(i).nodes(1,1))=-1;
                    elseif obj.kinematic.rigidLinkList(i).joints(1,1) == Joint.GroundSlider
                        continue;
                    else
                        couplerArray(obj.kinematic.rigidLinkList(i).nodes(1,1))=couplerArray(obj.kinematic.rigidLinkList(i).nodes(1,1))+1;
                    end
                end
                %node 2
                if couplerArray(obj.kinematic.rigidLinkList(i).nodes(1,2)) >=0
                    %check if it is ground
                    if obj.kinematic.rigidLinkList(i).joints(1,2) == Joint.GroundWelded || obj.kinematic.rigidLinkList(i).joints(1,2) == Joint.GroundPin
                        couplerArray(obj.kinematic.rigidLinkList(i).nodes(1,2))=-1;
                    elseif obj.kinematic.rigidLinkList(i).joints(1,2) == Joint.GroundSlider
                        continue;
                    else
                        couplerArray(obj.kinematic.rigidLinkList(i).nodes(1,2))=couplerArray(obj.kinematic.rigidLinkList(i).nodes(1,2))+1;
                    end
                end
            end
            %decide on the coupler point
            i=1;
            while i < 10
                nodes=find(couplerArray == i);
                if ~isempty(nodes)
                    obj.coupler=nodes(1);
                    break;
                else
                    i=i+1;
                end
            end
            %find the driver
            driverArray=[];
            for i=1:length(obj.kinematic.rigidLinkList)
                if obj.kinematic.rigidLinkList(i).joints(1,2) == Joint.GroundSlider && obj.coupler == obj.kinematic.rigidLinkList(i).nodes(1,2)
                    obj.driver=i;
                    driverArray=[];
                    break;
                elseif obj.kinematic.rigidLinkList(i).joints(1,1) == Joint.GroundPin
                    driverArray(end+1)=i;
                end
            end
            if ~isempty(driverArray)
                obj.driver=driverArray(1);
            end
        end
        
        function coupler=findCurrentCoupler(obj)
            %find current coupler points
            if obj.kinematic.rigidLinkList(obj.driver).joints(1,2) == Joint.GroundSlider
                couplerPoints=obj.kinematic.sliderCoupler(obj.coupler,obj.driver);
            else
                couplerPoints=obj.kinematic.crankCoupler(obj.coupler,obj.driver);
            end
            coupler=[];
            for i=1:length(couplerPoints)
                coupler(:,i)=[couplerPoints{i}.x;couplerPoints{i}.y];
            end
        end
        
        function workspace=changeWorkspace(obj,parent)
            %change the workspace depending on curves
            maxX=max(abs(obj.currentCouplerPoints(1,:)))*1.2;
            maxY=max(abs(obj.currentCouplerPoints(2,:)))*1.2;
            workspace=parent.getWorkspace();
            workspace.sizeX=max([maxY maxX workspace.sizeX workspace.sizeY]);
            workspace.sizeY=max([maxY maxX workspace.sizeX workspace.sizeY]);
        end
        
        %for calculating curve descriptors
        function c=calculatePDescriptors(obj,points,kLimit)
            n=size(points,2);
            c=zeros(1,kLimit*2);
            for k=-kLimit:kLimit
                sum=0;
                for j=1:n-1
                    point1=points(1,j)+1i*points(2,j) ;
                    point2=points(1,j+1)+1i*points(2,j+1);
                    slope=point2-point1;
                    slope=slope/abs(slope);
                    sum=sum+slope*exp(-2*pi*1i*(j-1)*k/(n-1));
                end
                c(1,k+kLimit+1)=sum/(n-1);
            end
        end
        
        function error=calculateError(obj,desc1,desc2)
            %calculate error between two descriptors
            error=0;
            for i=1:length(desc1)
                error= error+ ( abs(desc1(i))-abs(desc2(i)))^2;
            end
            error=sqrt(error);
        end
        
        function couplerLength=calculateCouplerLength(obj,coupler,startIndex,endIndex)
            %calculate the coupler length
            couplerLength=0;
            for i=startIndex:endIndex-1
                xDif=coupler(1,startIndex)-coupler(1,startIndex+1);
                yDif=coupler(2,startIndex)-coupler(2,startIndex+1);
                couplerLength=couplerLength+sqrt(xDif^2+yDif^2);
            end
        end
        
        
        function f=mainKinematicSynthesis(obj,x,targetDescriptor,targetLength)
            %main functon for kinematic synthesis
            %update nodes
            j=1;
            for i=1:length(obj.kinematic.nodes)
                obj.kinematic.nodes(i)=obj.kinematic.nodes(i).setNode(Point(x(j),x(j+1)));
                j=j+2;
            end
            %update links
            for j=1:length(obj.kinematic.rigidLinkList)
                obj.kinematic.rigidLinkList(j)=obj.kinematic.rigidLinkList(j).calculateAngle(obj.kinematic.nodes);
            end
            for j=1:length(obj.kinematic.rigidLinkList)
                obj.kinematic.rigidLinkList(j)=obj.kinematic.rigidLinkList(j).findNeighbour(obj.kinematic.rigidLinkList);
            end
            obj.kinematic=obj.kinematic.mainPreparation();
            coupler=obj.findCurrentCoupler();
            %calculate the error
            f=1000*obj.calculateError(targetDescriptor,obj.calculatePDescriptors(coupler,obj.noOfDescriptors));
            if ~logical(obj.closedCurve)
                for i=1:size(coupler,2)-1
                    index1=0;
                    for j=i+1:size(coupler,2)
                        if targetLength <= obj.calculateCouplerLength(coupler,i,j)
                            index1=i;
                            index2=j-1;
                            newF=1000*obj.calculateError(targetDescriptor,obj.calculatePDescriptors(coupler(:,index1:index2),obj.noOfDescriptors));
                            f=min([newF f]);
                            break;
                        end
                    end
                    if ~logical(index1)
                        break;
                    end
                end
            end
        end
        
        function stop=outFunc(obj,x,optimValues,state,parent,id,target)
            %ouput function
            if target == 1e3
                targetString='-';
            else
                targetString=num2str(target);
            end
            handles=parent.getHandles();
            if logical(parent.getBusy())
                stop=0;
                handles.runStatusStatic2.String=['Run: ' num2str(id) ' Iter: ' num2str(optimValues.iteration) ' Error: ' num2str(optimValues.fval) '(' targetString ')'];
            else
                stop=1;
                handles.runStatusStatic2.String=['Stopped at Run: ' num2str(id) ' Iter: ' num2str(optimValues.iteration) ' Error: ' num2str(optimValues.fval) '(' targetString ')'];
            end
            drawnow;
        end
        
        function obj=analyze(obj,mainFig,limit,mode,updateMode,parent)
            %main
            handles=parent.getHandles();
            handles.runStatusStatic2.String='Started';
            drawnow;
            parent.setBusy(1);
            runs=str2double(handles.times3Text.String);
            randomness=str2double(handles.randomText.String);
            target=1e3;
            inputNodes=obj.kinematic.nodes;
            for i=1:runs
                if ~logical(parent.getBusy())
                    break;
                end
                
                j=1;
                for k=1:length(obj.kinematic.nodes)
                    point=inputNodes(k).getNode();
                    initial(j)=point.x+randomness*limit/100*(rand*2-1);
                    initial(j+1)=point.y+randomness*limit/100*(rand*2-1);
                    j=j+2;
                end
                
                outputFunc=@(x,optimValues,state)obj.outFunc(x,optimValues,state,parent,i,target);
                
                poolobj = gcp('nocreate');
                if isempty(poolobj)
                    parallel='never' ;
                else
                    parallel='always' ;
                end
                opt=optimset('Display','iter-detailed','MaxFunEvals',400,'MaxIter',100,'OutputFcn', outputFunc,'Useparallel',parallel);
                %lower and upper bound
                lb=zeros(size(obj.nodeRanges,2)*2,1);
                ub=zeros(size(obj.nodeRanges,2)*2,1);
                for k=1:size(obj.nodeRanges,2)
                    lb(2*k-1)=obj.nodeRanges(1,k);
                    lb(2*k)=obj.nodeRanges(2,k);
                    ub(2*k-1)=obj.nodeRanges(3,k);
                    ub(2*k)=obj.nodeRanges(4,k);
                end
                targetLength=obj.calculateCouplerLength(obj.targetCouplerPoints,1,size(obj.targetCouplerPoints,2));
                try
                    if logical(obj.closedCurve)
                        [x,fval,exitflag,output,lambda,grad,hessian] =fmincon(@(x)obj.mainKinematicSynthesis(x,obj.calculatePDescriptors([obj.targetCouplerPoints,obj.targetCouplerPoints(:,1)],obj.noOfDescriptors),targetLength),initial,[],[],[],[],lb,ub,[],opt);
                    else
                        [x,fval,exitflag,output,lambda,grad,hessian] =fmincon(@(x)obj.mainKinematicSynthesis(x,obj.calculatePDescriptors(obj.targetCouplerPoints,obj.noOfDescriptors),targetLength),initial,[],[],[],[],lb,ub,[],opt);
                    end
                        %[x,fval,exitflag,output]=fminsearch(,initial,opt);
                    %check if this is the best yet
                    if fval < target
                        %update nodes
                        j=1;
                        for k=1:length(obj.kinematic.nodes)
                            obj.kinematic.nodes(k)=obj.kinematic.nodes(k).setNode(Point(x(j),x(j+1)));
                            j=j+2;
                        end
                        %update input nodes
                        if logical(updateMode)
                            inputNodes=obj.kinematic.nodes;
                        end
                        %update links
                        for j=1:length(obj.kinematic.rigidLinkList)
                            obj.kinematic.rigidLinkList(j)=obj.kinematic.rigidLinkList(j).calculateAngle(obj.kinematic.nodes);
                        end
                        for j=1:length(obj.kinematic.rigidLinkList)
                            obj.kinematic.rigidLinkList(j)=obj.kinematic.rigidLinkList(j).findNeighbour(obj.kinematic.rigidLinkList);
                        end
                        obj.kinematic=obj.kinematic.mainPreparation();
                        obj.currentCouplerPoints=obj.findCurrentCoupler();
                        %post process
                        obj=obj.postProcessNodes(obj.currentCouplerPoints,obj.targetCouplerPoints,targetLength);
                        %update links
                        for j=1:length(obj.kinematic.rigidLinkList)
                            obj.kinematic.rigidLinkList(j)=obj.kinematic.rigidLinkList(j).calculateAngle(obj.kinematic.nodes);
                        end
                        for j=1:length(obj.kinematic.rigidLinkList)
                            obj.kinematic.rigidLinkList(j)=obj.kinematic.rigidLinkList(j).findNeighbour(obj.kinematic.rigidLinkList);
                        end
                        obj.kinematic=obj.kinematic.mainPreparation();
                        obj.currentCouplerPoints=obj.findCurrentCoupler();
                        obj=obj.drawAll(mainFig,limit,mode,parent);
                        drawnow;
                        target=fval;
                    end
                catch
                end
            end
            handles.runStatusStatic2.String=['Final Error: ' num2str(target)];
            handles.stopKinSynButton.Visible='off';
            obj=obj.restore(parent);
            drawnow;
        end
        
        
        function [state,options,optchanged]=outFuncGlobal(obj,options,state,flag,parent)
            %ouput function
            handles=parent.getHandles();
            optchanged=0;
            if isempty(state.Best)
                state.Best=Inf;
            end
            if logical(parent.getBusy())
                handles.runStatusStatic4.String=['Generation: ' num2str(state.Generation) ' Error: ' num2str(state.Best(end)) ];
            else
                state.StopFlag='1';
                handles.runStatusStatic4.String=['Stopped at Generation: ' num2str(state.Generation) ' Error: ' num2str(state.Best(end) ) ];
            end
            drawnow;
        end
        
        function obj=analyzeGlobal(obj,mainFig,limit,mode,stopMode,stopValue,parent)
            %main
            handles=parent.getHandles();
            handles.runStatusStatic4.String='Started';
            drawnow;
            parent.setBusy(1);
            poolobj = gcp('nocreate');
            if isempty(poolobj)
                parallel=false;
            else
                parallel=true;
            end
            outputFunc=@(options,state,flag)obj.outFuncGlobal(options,state,flag,parent);
            %stopping criteria
            hybridopts = optimoptions('fmincon','Display','iter');
            switch stopMode
                case 1
                    opt=gaoptimset('Display','iter','Generations',stopValue,'Useparallel',parallel,'OutputFcn', outputFunc,'HybridFcn',{@fmincon,hybridopts});
                case 2
                    opt=gaoptimset('Display','iter','TimeLimit',stopValue,'Useparallel',parallel,'OutputFcn', outputFunc,'HybridFcn',{@fmincon,hybridopts});
                case 3
                    opt=gaoptimset('Display','iter','Useparallel',parallel,'Useparallel',parallel,'OutputFcn', outputFunc,'HybridFcn',{@fmincon,hybridopts});
            end
            %lower and upper bound
            lb=zeros(size(obj.nodeRanges,2)*2,1);
            ub=zeros(size(obj.nodeRanges,2)*2,1);
            for k=1:size(obj.nodeRanges,2)
                lb(2*k-1)=obj.nodeRanges(1,k);
                lb(2*k)=obj.nodeRanges(2,k);
                ub(2*k-1)=obj.nodeRanges(3,k);
                ub(2*k)=obj.nodeRanges(4,k);
            end
            targetLength=obj.calculateCouplerLength(obj.targetCouplerPoints,1,size(obj.targetCouplerPoints,2));
            [x,fval,exitflag,output,population,scores]  =ga(@(x)obj.mainKinematicSynthesis(x,obj.calculatePDescriptors(obj.targetCouplerPoints,obj.noOfDescriptors),targetLength),size(obj.nodeRanges,2)*2,[],[],[],[],lb,ub,[],opt);
            %[x,fval,exitflag,output]=fminsearch(,initial,opt);
            %check if this is the best yet
            
            %update nodes
            j=1;
            for k=1:length(obj.kinematic.nodes)
                obj.kinematic.nodes(k)=obj.kinematic.nodes(k).setNode(Point(x(j),x(j+1)));
                j=j+2;
            end
            %update links
            for j=1:length(obj.kinematic.rigidLinkList)
                obj.kinematic.rigidLinkList(j)=obj.kinematic.rigidLinkList(j).calculateAngle(obj.kinematic.nodes);
            end
            for j=1:length(obj.kinematic.rigidLinkList)
                obj.kinematic.rigidLinkList(j)=obj.kinematic.rigidLinkList(j).findNeighbour(obj.kinematic.rigidLinkList);
            end
            obj.kinematic=obj.kinematic.mainPreparation();
            obj.currentCouplerPoints=obj.findCurrentCoupler();
            %post process
            obj=obj.postProcessNodes(obj.currentCouplerPoints,obj.targetCouplerPoints,targetLength);
            %update links
            for j=1:length(obj.kinematic.rigidLinkList)
                obj.kinematic.rigidLinkList(j)=obj.kinematic.rigidLinkList(j).calculateAngle(obj.kinematic.nodes);
            end
            for j=1:length(obj.kinematic.rigidLinkList)
                obj.kinematic.rigidLinkList(j)=obj.kinematic.rigidLinkList(j).findNeighbour(obj.kinematic.rigidLinkList);
            end
            obj.kinematic=obj.kinematic.mainPreparation();
            obj.currentCouplerPoints=obj.findCurrentCoupler();
            obj=obj.drawAll(mainFig,limit,mode,parent);
            drawnow;
            handles.runStatusStatic4.String=['Final Error: ' num2str(fval)];
            handles.stopKinSynButton2.Visible='off';
            obj=obj.restore(parent);
            drawnow;
        end
        
        
        
        function analysis=analyzeCoupler(obj,coupler,type)
            %analyze the coupler
            switch type
                case 1
                    %find the range
                    rangeX=max(coupler(1,:))-min(coupler(1,:));
                    rangeY=max(coupler(2,:))-min(coupler(2,:));
                    analysis.range=0.5*(rangeX+rangeY);
                case 2
                    %find angle
                    [maxX,maxIndex]=max(coupler(1,:));
                    [minX,minIndex]=min(coupler(1,:));
                    yMax=0;
                    for i=1:length(maxIndex)
                        yMax=yMax+coupler(2,maxIndex(i));
                    end
                    yMax=yMax/length(maxIndex);
                    
                    yMin=0;
                    for i=1:length(minIndex)
                        yMin=yMin+coupler(2,minIndex(i));
                    end
                    yMin=yMin/length(minIndex);
                    
                    analysis.angle=atan2(yMax-yMin,maxX(1)-minX(1));
                case 3
                    %find origin
                    analysis.aveX=mean(coupler(1,:));
                    analysis.aveY=mean(coupler(2,:));
            end
        end
        
        function obj=postProcessNodes(obj,current,target,targetLength)
            %post process nodes after optimization
            %first find which portion is the best
            f=1000*obj.calculateError(obj.calculatePDescriptors(target,obj.noOfDescriptors),obj.calculatePDescriptors(current,obj.noOfDescriptors));
            start=1;
            finish=size(current,2);
                      
            if ~logical(obj.closedCurve)
                for i=1:size(current,2)-1
                    startIndex=0;
                    for j=i+1:size(current,2)
                        if targetLength < obj.calculateCouplerLength(current,i,j)
                            startIndex=i;
                            finishIndex=j-1;
                            newF=1000*obj.calculateError(obj.calculatePDescriptors(target,obj.noOfDescriptors),obj.calculatePDescriptors(current(:,startIndex:finishIndex),obj.noOfDescriptors));
                            if f > newF
                                start=i;
                                finish=j-1;
                                f=newF;
                            end
                            break;
                        end
                    end
                    if ~logical(startIndex)
                        break;
                    end
                end
            end
      
            
            %first expand the nodes
                        expansion=obj.analyzeCoupler(target,1).range/obj.analyzeCoupler(current(:,start:finish),1).range;
                        for i=1:length(obj.kinematic.nodes)
                            node=obj.kinematic.nodes(i).getNode();
                            obj.kinematic.nodes(i)=obj.kinematic.nodes(i).setNode(Point(node.x*expansion,node.y*expansion));
                        end
                        %expand the current curve
                        current=current*expansion;
   
            
            %rotate the current curve
            rotation=obj.analyzeCoupler(target,2).angle-obj.analyzeCoupler(current(:,start:finish),2).angle;
            if obj.kinematic.rigidLinkList(obj.driver).joints(1,2) == Joint.GroundSlider
                %slider is the driver
                obj.kinematic.rigidLinkList(obj.driver).sliderAngle=obj.kinematic.rigidLinkList(obj.driver).sliderAngle+rotation*180/pi;
                current=obj.findCurrentCoupler();
                start=1;
                finish=length(current);
            else
                rotationMatrix=[cos(rotation) -sin(rotation); sin(rotation) cos(rotation)];
                for i=1:length(obj.kinematic.nodes)
                    node=[obj.kinematic.nodes(i).getNode().x;obj.kinematic.nodes(i).getNode().y];
                    node=rotationMatrix*node;
                    obj.kinematic.nodes(i)=obj.kinematic.nodes(i).setNode(Point(node(1),node(2)));
                end
                for i=1:size(current,2)
                    current(:,i)=rotationMatrix*current(:,i);
                end
            end
            
            %translate the nodes
            translationX=obj.analyzeCoupler(target,3).aveX-obj.analyzeCoupler(current(:,start:finish),3).aveX ;
            translationY=obj.analyzeCoupler(target,3).aveY-obj.analyzeCoupler(current(:,start:finish),3).aveY  ; 
            for i=1:length(obj.kinematic.nodes)
                node=obj.kinematic.nodes(i).getNode();
                obj.kinematic.nodes(i)=obj.kinematic.nodes(i).setNode(Point(node.x+translationX,node.y+translationY));
            end
            
        end
        
        function obj=restore(obj,parent)
            %restore the nodes
            parent.setBusy(0);
            parent.updateNodesAfterSynthesis(obj.kinematic.nodes);
        end

        
    end
end