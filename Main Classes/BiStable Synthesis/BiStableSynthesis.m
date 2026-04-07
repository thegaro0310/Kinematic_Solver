classdef BiStableSynthesis
    %bistable synthesis class
    properties
        bistable=BiStable.empty;
        crankList={};
        sliderList={};
        unknowns=[];
        originalLinks=Link.empty;
        originalRigidLinks=rigidBody.empty;
        originalNodes=Node.empty;
        firstNodes=Node.empty;
        originalSprings=TorsionSpring.empty;
        selectedDriver=[0 0 0];
        selectedMode=1;
        newValues=[];
        annotation=gobjects(0);
    end
    
    methods
        function obj=BiStableSynthesis(links,nodes,torsionSprings,parent)
            %constructor function
            popHandles=parent.getPopHandles();
            obj.bistable=BiStable(links,nodes,torsionSprings,parent.getWorkspace());
            %add compliant members to the group
            for i=1:length(obj.bistable.static.kinematic.allBeams)
                if ~isempty(obj.bistable.static.kinematic.allBeams{i}.crossSection)
                    obj.unknowns(end+1)=obj.bistable.static.kinematic.allBeams{i}.id;
                end
            end
            handles=parent.getHandles();
            if isempty(obj.unknowns)
                popHandles.synthesize.Enable='off';
            end
            %analyze
            obj=obj.analyzeMechanism(parent);
            if ~isempty(obj.crankList)
                obj.selectedDriver=[1 1 obj.crankList{1,2}];
            elseif ~isempty(obj.sliderList)
                obj.selectedDriver=[2 1 obj.sliderList{1,2}];
            end
            dataCount=size(obj.crankList,1)+size(obj.sliderList,1);
            if logical(dataCount)
                if dataCount == 1
                    %one bistable behaviour
                    handles.helpStatic.String='The mechanism has one driver that will result in bistable position.';
                else
                    handles.helpStatic.String=['The mechanism have ' num2str(dataCount) ' drivers that will result in bistable position.'];
                end
                popHandles.driver.Value=1;
                popHandles.driver.String=num2str((1:dataCount)');
                %set the selected data
                obj=obj.setSelectedData(parent);
            else
                %no biStable behaviour
                handles.helpStatic.String='The mechanism has no driver that will result in bistable position.';
                popHandles.driver.Value=1;
                popHandles.driver.String='-';
                popHandles.synthesize.Enable='off';
            end
            %original
            obj.originalLinks=links;
            obj.firstNodes=nodes;
            obj.originalNodes=obj.bistable.static.nodes;
            obj.originalSprings=torsionSprings;
        end
        
        function obj=drawAll(obj,mainFig,limit,mode,parent)
            %draw all links
            obj.bistable.static.kinematic=obj.bistable.static.kinematic.drawAll(mainFig,limit,mode,parent);
        end
        
        function obj=deleteAll(obj,parent)
            %draw all links
            obj.bistable.static.kinematic=obj.bistable.static.kinematic.deleteAll();
            %delete nodes
            for i=1:length(obj.bistable.static.nodes)
                obj.bistable.static.nodes(i)=obj.bistable.static.nodes(i).deleteNodeDrawing;
            end
            if ~logical(parent.getBusy)
                delete(obj.annotation);
            end
        end
        
        function obj=analyzeMechanism(obj,parent)
            %analyze the current mechanism to find driver link
            for i=1:length(obj.bistable.static.kinematic.allBeams)
                if  isa(obj.bistable.static.kinematic.allBeams{i},'KinematicsBeam')
                    if logical(strcmp(obj.bistable.static.kinematic.allBeams{i}.type,'fixedPinBeam'))
                        obj.crankList{end+1,1}=i;obj.crankList{end,2}=-1;
                        obj.crankList{end+1,1}=i;obj.crankList{end,2}=1;
                    elseif logical(strcmp(obj.bistable.static.kinematic.allBeams{i}.type,'allFixedSlider'))
                        obj.sliderList{end+1,1}=i;obj.sliderList{end,2}=-1;
                        obj.sliderList{end+1,1}=i;obj.sliderList{end,2}=1;
                    end
                end
            end
            
            %analyze the mechanism
            total=size(obj.sliderList,1)+size(obj.crankList,1);
            wait = waitbar(0,'Analyzing the mechanism...');
            %first cranks
            delete=[];
            for i=1:size(obj.crankList,1)
                stablePoints=obj.bistable.bistableAnalysisLink(obj.crankList{i,1},obj.crankList{i,2},parent);
                if length(stablePoints) == 2
                    %bistable found
                    obj.crankList{i,3}=stablePoints;
                else
                    delete(end+1)=i;
                end
                waitbar(i/total);
            end
            obj.crankList(delete,:)=[];
            %then sliders
            delete=[];
            for i=1:size(obj.sliderList,1)
                stablePoints=obj.bistable.bistableAnalysisSlider(obj.sliderList{i,1},obj.sliderList{i,2},parent);
                if length(stablePoints) == 2
                    %bistable found
                    obj.sliderList{i,3}=stablePoints;
                else
                    delete(end+1)=i;
                end
                waitbar(i+size(obj.sliderList,1)/total);
            end
            obj.sliderList(delete,:)=[];
            close(wait);
        end
        
        function obj=setSelectedData(obj,parent)
            %set selected driver
            popHandles=parent.getPopHandles();
            if ~logical(obj.selectedDriver(1))
                %no driver
                return;
            end
            %deselect all links
            for i=1:length(obj.bistable.static.kinematic.allBeams)
                obj.bistable.static.kinematic.allBeams{i}.selected=0;
            end
            index=popHandles.driver.Value;
            if index <= length(obj.crankList)
                %crank
                obj.selectedDriver(1)=1;
                obj.selectedDriver(2)=index;
                obj.selectedDriver(3)=obj.crankList{index,2};
                obj.bistable.static.kinematic.allBeams{obj.crankList{index,1}}.selected=1;
            else
                %slider
                obj.selectedDriver(1)=2;
                obj.selectedDriver(2)=index-length(obj.crankList);
                obj.selectedDriver(3)=obj.sliderList{index-length(obj.crankList),2};
                obj.bistable.static.kinematic.allBeams{obj.sliderList{index-length(obj.crankList),1}}.selected=1;
            end
            %plot
            obj=obj.drawPlot(parent);
            obj=obj.setSelectedMode(parent);
        end
        
        function obj=setSelectedMode(obj,parent)
            %for setting the selected mode
            popHandles=parent.getPopHandles();
            %get the data
            if ~logical(obj.selectedDriver(1))
                %no driver
                return;
            elseif obj.selectedDriver(1) == 1
                %crank
                data=obj.crankList{obj.selectedDriver(2),3};
            else
                %slider
                data=obj.sliderList{obj.selectedDriver(2),3};
            end
            
            switch popHandles.type.Value
                case 1
                    %critical force
                    popHandles.current.String=num2str(data(1).maxLoad);
                    popHandles.target.String=num2str(data(1).maxLoad);
                    if obj.selectedDriver(1) == 1
                        %crank
                        popHandles.unit1.String=popHandles.unitStr;
                        popHandles.unit2.String=popHandles.unitStr;
                    else
                        %slider
                        popHandles.unit1.String=popHandles.unitStr2;
                        popHandles.unit2.String=popHandles.unitStr2;
                    end
                case 2
                    %bistable position
                    popHandles.current.String=num2str(data(1).distance);
                    popHandles.target.String=num2str(data(1).distance);
                    if obj.selectedDriver(1) == 1
                        %crank
                        popHandles.unit1.String='°';
                        popHandles.unit2.String='°';
                    else
                        %slider
                        popHandles.unit1.String=popHandles.unitStr3;
                        popHandles.unit2.String=popHandles.unitStr3;
                    end
                case 3
                    %stable position
                    popHandles.current.String=num2str(data(2).distance);
                    popHandles.target.String=num2str(data(2).distance);
                    if obj.selectedDriver(1) == 1
                        %crank
                        popHandles.unit1.String='°';
                        popHandles.unit2.String='°';
                    else
                        %slider
                        popHandles.unit1.String=popHandles.unitStr3;
                        popHandles.unit2.String=popHandles.unitStr3;
                    end
            end
        end
        
        function obj=drawPlot(obj,parent)
            %draw the energy and torque values
            popHandles=parent.getPopHandles();
            if ~logical(parent.getBusy)
                delete(obj.annotation);
                if obj.selectedDriver(3) == 1
                    obj.annotation=annotation(popHandles.plotArea,'textarrow',[0.2 0.9],[0.49 0.49],'String','Motion','Color',Colors.Point2.getColor(),'FontWeight','bold');
                else
                    obj.annotation=annotation(popHandles.plotArea,'textarrow',[0.8 0.1],[0.49 0.49],'String','Motion','Color',Colors.Point2.getColor(),'FontWeight','bold');
                end
            end
            if obj.selectedDriver(1) == 1
                %link type
                data=obj.crankList{obj.selectedDriver(2),3};
                plot1=subplot(2,1,1,'Parent',popHandles.plotArea);
                plot(plot1,data(1).input,data(1).energyList,'Color',Colors.Text.getColor());
                xlabel(plot1,'input link angle(deg)','FontSize',8);
                xlim(plot1,[min(data(1).input) max(data(1).input)] );
                if parent.getWorkspace().momentFactor() == 1e6
                    ylabel(plot1,'Megajoule','FontSize',8);
                elseif parent.getWorkspace().momentFactor() == 1e3
                    ylabel(plot1,'Kilojoule','FontSize',8);
                elseif parent.getWorkspace().momentFactor() == 1
                    ylabel(plot1,'Joule','FontSize',8);
                elseif parent.getWorkspace().momentFactor() == 1e-3
                    ylabel(plot1,'Millijoule','FontSize',8);
                elseif parent.getWorkspace().momentFactor() == 1e-6
                    ylabel(plot1,'Microjoule','FontSize',8);
                elseif parent.getWorkspace().momentFactor() == 1e-9
                    ylabel(plot1,'Nanojoule','FontSize',8);
                else
                    lengthString=LengthUnit.getString(parent.getWorkspace().unitLength);
                    loadString=ForceUnit.getString(parent.getWorkspace().unitForce);
                    ylabel(plot1,[loadString '.' lengthString],'FontSize',8);
                end
                title(plot1,'Total Potential Energy','FontSize',8);
                plot2=subplot(2,1,2,'Parent',popHandles.plotArea);
                plot(plot2,data(1).input,data(1).loadList,'Color',Colors.Constrained.getColor());
                xlabel(plot2,'input link angle(deg)','FontSize',8);
                xlim(plot2,[min(data(1).input) max(data(1).input)] );
                lengthString=LengthUnit.getString(parent.getWorkspace().unitLength);
                loadString=ForceUnit.getString(parent.getWorkspace().unitForce);
                ylabel(plot2,[loadString '.' lengthString],'FontSize',8);
                title(plot2,'Torque Magnitude','FontSize',8);
                
            else
                %node type
                data=obj.sliderList{obj.selectedDriver(2),3};
                plot1=subplot(2,1,1,'Parent',handles.plotPanel);
                plot(plot1,data(1).input,data(1).energyList,'Color',Colors.Text.getColor());
                xlim(plot1,[min(data(1).input) max(data(1).input)] );
                lengthString=LengthUnit.getString(parent.getWorkspace().unitLength);
                xlabel(plot1,['slider motion','(',lengthString,')'],'FontSize',8);
                if parent.getWorkspace().momentFactor() == 1
                    ylabel(plot1,'Joule','FontSize',8);
                elseif parent.getWorkspace().momentFactor() == 1e-3
                    ylabel(plot1,'mili-Joule','FontSize',8);
                else
                    lengthString=LengthUnit.getString(parent.getWorkspace().unitLength);
                    loadString=ForceUnit.getString(parent.getWorkspace().unitForce);
                    ylabel(plot1,[loadString '.' lengthString],'FontSize',8);
                end
                title(plot1,'Total Potential Energy','FontSize',8);
                plot2=subplot(2,1,2,'Parent',handles.plotPanel);
                plot(plot2,data(1).input,data(1).loadList,'Color',Colors.Constrained.getColor());
                xlim(plot2,[min(data(1).input) max(data(1).input)] );
                lengthString=LengthUnit.getString(parent.getWorkspace().unitLength);
                loadString=ForceUnit.getString(parent.getWorkspace().unitForce);
                ylabel(plot2,[loadString '.' lengthString],'FontSize',8);
                xlabel(plot2,['slider motion','(',lengthString,')'],'FontSize',8);
                title(plot2,'Force Magnitude');
            end
            grid(plot1,'on');grid(plot2,'on');
            grid(plot1,'minor');grid(plot2,'minor');
            
        end
        
        function links=findLinksFromRigidLink(obj)
            %converts rigidlinklist id to link id
            links=zeros(1,length(obj.unknowns));
            for j=1:length(obj.unknowns)
                for i=1:length(obj.bistable.static.kinematic.allBeams)
                    if obj.bistable.static.kinematic.allBeams{i}.id == obj.unknowns(j)
                        links(j)=i;
                        break;
                    end
                end
            end
        end
        
        function stop = outputFunction(obj,x, optimValues, state,parent,original)
            %draw the config
            progressHandles=parent.getProgressBar();
            popHandles=parent.getPopHandles();
            if ~logical(parent.getBusy())
                stop=true;
                progressHandles.progressText.String='Aborted';
            else
                stop=false;
            end
            
            if  isempty(optimValues.fval)
                stop=false;
            elseif  abs(optimValues.fval) <1e-6
                stop=true;
                progressHandles.progressText.String=['Iteration: ' num2str(optimValues.iteration)];                
            end
            progressHandles.progressText.String=['Iteration: ' num2str(optimValues.iteration)];   
            obj=parent.getBistableSynthesis();
            if obj.selectedDriver(1) == 1
                %crank
                data=obj.crankList{obj.selectedDriver(2),3};
            else
                %slider
                data=obj.sliderList{obj.selectedDriver(2),3};
            end
            switch popHandles.type.Value
                case 1
                    %critical force
                    popHandles.current.String=num2str(data(1).maxLoad);
                case 2
                    %bistable position
                    popHandles.current.String=num2str(data(1).distance);
                case 3
                    %stable position
                    popHandles.current.String=num2str(data(2).distance);
            end
            try
                patch([0,abs(original-optimValues.fval)/original,abs(original-optimValues.fval)/original,0],[0,0,1,1],Colors.StatusComplete.getColor(),'Parent',progressHandles.progress);
                figure(progressHandles.progressGUI);
            catch err
                display(err);
                progressHandles
                display('Error displaying progress bar');
            end
            obj.drawPlot(parent);
            drawnow;
            figure(popHandles.bistableGUI);
        end
        
        function f=criticalForceMain(obj,x,originalNodes,originalLinks,unknowns,target,parent)
            %main function changing the critical force
            %update original links
            for i=1:length(unknowns)
                xSection=originalLinks(unknowns(i)).getCrossSection();
                xSection.width=12;
                xSection.thickness=1;
                xSection.E=abs(x(i));
                originalLinks(unknowns(i))=originalLinks(unknowns(i)).setCrossSection(xSection);
            end
            bistable=BiStable(originalLinks,originalNodes,obj.originalSprings,parent.getWorkspace());
            %check if crank or slider
            if obj.selectedDriver(1) == 1
                %crank
                current=bistable.bistableAnalysisLink(obj.crankList{obj.selectedDriver(2),1},obj.crankList{obj.selectedDriver(2),2},parent);
            else%slider
                current=bistable.bistableAnalysisSlider(obj.sliderList{obj.selectedDriver(2),1},obj.sliderList{obj.selectedDriver(2),2},parent);
            end
            %error
            if length(current) == 2
                f=(current(1).maxLoad-target)^2/(parent.getWorkspace().forceFactor()^2) ;
                parent.updateBistableSynthesisBistable(bistable,current);
            else
                f=1000;
            end
        end
        
        function f=biStablePosMain(obj,x,originalNodes,originalLinks,unknowns,target,parent,type)
            %main function changing the critical force
            %update nodes
            j=1;
            for i=1:length(originalNodes)
                originalNodes(i)=originalNodes(i).setNode(Point(x(j),x(j+1)));
                j=j+2;
            end
            bistable=BiStable(originalLinks,originalNodes,obj.originalSprings,parent.getWorkspace());
            %check if crank or slider
            if obj.selectedDriver(1) == 1
                %crank
                current=bistable.bistableAnalysisLink(obj.crankList{obj.selectedDriver(2),1},obj.crankList{obj.selectedDriver(2),2},parent);
            else%slider
                current=bistable.bistableAnalysisSlider(obj.sliderList{obj.selectedDriver(2),1},obj.sliderList{obj.selectedDriver(2),2},parent);
            end
            %error
            if length(current) == 2
                if type == 1
                    %bistable position
                    f=(current(1).distance-target)^2;
                else
                    %stable position
                    f=(current(2).distance-target)^2;
                end
                parent.updateBistableSynthesisBistable(bistable,current);
            else
                f=1000;
            end
        end
        
        function obj=synthesize(obj,parent)
            %main function
            handles=parent.getHandles();
            popHandles=parent.getPopHandles();
            %iter string
            handles.helpStatic.String='Synthesis is running...';
            figure(popHandles.bistableGUI);
            links=obj.findLinksFromRigidLink(); 
            %initial config
            switch popHandles.type.Value
                case 1
                    %max load
                    fMincon=@(x)obj.criticalForceMain(x,obj.firstNodes,obj.originalLinks,links,str2double(popHandles.target.String),parent);
                    %set the initial conditions
                    initialGuess=zeros(1,length(links));
                    for i=1:length(links)
                        initialGuess(i)=obj.originalLinks(links(i)).getCrossSection().E*obj.originalLinks(links(i)).getCrossSection().getI();
                    end
                case 2
                    %bistable pos
                    fMincon=@(x)obj.biStablePosMain(x,obj.firstNodes,obj.originalLinks,links,str2double(popHandles.target.String),parent,1);
                    %set the initial conditions
                    initialGuess=zeros(1,length(obj.firstNodes)*2);
                    j=1;
                    for k=1:length(obj.firstNodes)
                        point=obj.firstNodes(k).getNode();
                        initialGuess(j)=point.x;
                        initialGuess(j+1)=point.y;
                        j=j+2;
                    end
            case 3
                    %stable pos
                    fMincon=@(x)obj.biStablePosMain(x,obj.firstNodes,obj.originalLinks,links,str2double(popHandles.target.String),parent,2);
                    %set the initial conditions
                    initialGuess=zeros(1,length(obj.firstNodes)*2);
                    j=1;
                    for k=1:length(obj.firstNodes)
                        point=obj.firstNodes(k).getNode();
                        initialGuess(j)=point.x;
                        initialGuess(j+1)=point.y;
                        j=j+2;
                    end
            end
            original=(str2double(popHandles.current.String)-str2double(popHandles.target.String))^2;
            outputFnc=@(x, optimValues, state)obj.outputFunction(x, optimValues, state,parent,original);
            opt=optimset('Display','iter-detailed','MaxFunEvals',1e4,'MaxIter',1e4,'OutputFcn',outputFnc);
            [x,fval,exitflag,output] =  fminunc(fMincon,initialGuess,opt);
            %save data
            obj=parent.getBistableSynthesis();
            obj.newValues=x;
            %restore
            handles.helpStatic.String='Synthesis is complete and reanalyzing.';
        end
        
        
        
        function obj=restore(obj,parent)
            %restore the flexural synthesis
            mainHandles=parent.getPopHandles();
            parent.setBusy(0);
            obj=parent.getBistableSynthesis();
            obj=obj.deleteAll(parent);
            switch mainHandles.type.Value
                case 1
                    %critical force
                    parent.saveBistableSynthesis(obj.findLinksFromRigidLink(),obj.newValues);    
                case 2
                    %bistable position                    
                    parent.saveBistableSynthesis2(obj.newValues);
                    
                case 3
                    %stable position
                    parent.saveBistableSynthesis2(obj.newValues);
            end
        end
        
    end
end