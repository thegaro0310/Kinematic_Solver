classdef PrbBeam < Beam
    properties
        prbBeams=struct('length0',[],'length',[],'theta',0,'theta0',0,'linearSpring',0,'index',[0 0]);
        torsionSprings=struct('magnitude',[]);
    end
    
    methods(Static)
        function text = getText()
            %get summary text
            text={['Pseudo Rigid Body Model (PRBM) approach replaces flexible elements with rigid-body replacements.'...
                ' Since rigid-body kinematics are well studied, the PRB approach is usually more intuitive for the purpose of analysis and design. PRBM models can handle large deflections.'...
                ]; ['The PRBMs with linear springs allow beams to increase and decrease in length.' ...
                ' For more information visit:']; ....
                [ char(9) 'http:\\complaintanalysis.com\PRBM']};
        end
        
        function accuracy=getAccuracy()
            %get analysis accuracy
            accuracy=[6 10];
        end
        
        function speed=getSpeed()
            %get analysis speed
            speed=[7 9];
        end
        
        function ext=getExpansion()
            %get allows expansion or not
            ext=true;
        end
    end
    
    
    methods
        
        function beam=PrbBeam(id,nodes,joints,angle,lengthBeam,crossSection,workspace)
            %constructor for the prb beam class
            beam@Beam(id,0,0,nodes,joints,angle,lengthBeam,crossSection);
            crossSection=beam.crossSection;
            beam.degreesOfFreedom= sum(~isinf(crossSection.prbModel(:,3)))+size(crossSection.prbModel,1);  %number of angles plus linear strings
            if beam.joints(1,1) == Joint.GroundWelded
                beam.degreesOfFreedom=beam.degreesOfFreedom-1;
            end
            %setup the prb matrix
            for i=1:size(crossSection.prbModel,1)
                %torsion spring
                if ~isinf(crossSection.prbModel(i,2))
                    torsionSpringMagnitude=crossSection.prbModel(i,2)*crossSection.E*workspace.EFactor()*beam.crossSection.getI()/beam.length0*workspace.lengthFactor()^3;
                    torsionSpringMagnitude=torsionSpringMagnitude/workspace.torsionSpringFactor();
                else
                    torsionSpringMagnitude=0;
                end
                if ~isinf(crossSection.prbModel(i,3))
                    linearSpringMagnitude=crossSection.prbModel(i,3)*crossSection.E*workspace.EFactor()*beam.crossSection.getA()/beam.length0*workspace.lengthFactor();
                    linearSpringMagnitude=linearSpringMagnitude/workspace.linearSpringFactor();
                else
                    linearSpringMagnitude=0;
                end
                beam.prbBeams(i)=struct('length0',crossSection.prbModel(i,1)*beam.length0,'length',crossSection.prbModel(i,1)*beam.length0...
                    ,'theta',beam.theta0,'theta0',beam.theta0,'linearSpring',linearSpringMagnitude,'index',[0 0]);
                beam.torsionSprings(i)=struct('magnitude',torsionSpringMagnitude);
            end
        end
        
        function beam=updateBeam(beam,inputs,index)
            %updateBeam prb beams from inputs
            inputIndex=beam.id;
            %first beam angle
            if beam.joints(1,1) == Joint.GroundWelded
                beam.prbBeams(1).theta=beam.prbBeams(1).theta0;
                beam.prbBeams(1).index(1)=0;
            else
                beam.prbBeams(1).theta=inputs(inputIndex);
                beam.prbBeams(1).index(1)=index(inputIndex);
                inputIndex=inputIndex+1;
            end
            %first beam length
            if ~isinf(beam.crossSection.prbModel(1,3))
                beam.prbBeams(1).length=inputs(inputIndex);
                beam.prbBeams(1).index(2)=index(inputIndex);
                inputIndex=inputIndex+1;
            else
                beam.prbBeams(1).length=beam.prbBeams(1).length0;
                beam.prbBeams(1).index(2)=0;
            end
            %remaining beams
            for i=2:size(beam.crossSection.prbModel,1)
                %angle
                beam.prbBeams(i).theta=inputs(inputIndex);
                beam.prbBeams(i).index(1)=index(inputIndex);
                inputIndex=inputIndex+1;
                %linear spring
                if ~isinf(beam.crossSection.prbModel(i,3))
                    beam.prbBeams(i).length=inputs(inputIndex);
                    beam.prbBeams(i).index(2)=index(inputIndex);
                    inputIndex=inputIndex+1;
                else
                    beam.prbBeams(i).length=beam.prbBeams(i).length0;
                    beam.prbBeams(i).index(2)=0;
                end
            end
        end
        
        function initialGuess=getInitialGuess(beam)
            %get initial guess for the beam
            initialGuess=[];
            %first beam angle
            if beam.joints(1,1) ~= Joint.GroundWelded
                initialGuess(end+1)=beam.theta0;
            end
            %first beam length
            if ~isinf(beam.crossSection.prbModel(1,3))
                initialGuess(end+1)=beam.prbBeams(1).length;
            end
            %remainung beams
            for i=2:size(beam.crossSection.prbModel,1)
                %angle
                initialGuess(end+1)=beam.prbBeams(i).theta;
                %linear spring
                if ~isinf(beam.crossSection.prbModel(i,3))
                    initialGuess(end+1)=beam.prbBeams(i).length;
                end
            end
        end
        
        %%
        %Kinematic Functions
        
        function x = getX(beam,outputMatrix,distance)
            %x coordinate value
            beam=beam.updateBeam(outputMatrix,outputMatrix);
            if distance < 0
                x=-(beam.getX(outputMatrix,100)-beam.getX(outputMatrix,100+distance));
            elseif distance == 0
                x=0;
            else
                currentDistance=0;
                x=0;
                for i=1:length(beam.prbBeams)
                    prbDistance=beam.crossSection.prbModel(i,1)*100;
                    if currentDistance + prbDistance >= distance || i == length(beam.prbBeams)
                        modifier=(distance-currentDistance)/prbDistance;
                        x=x+modifier*beam.prbBeams(i).length*cos(beam.prbBeams(i).theta);
                        return;
                    else
                        x=x+beam.prbBeams(i).length*cos(beam.prbBeams(i).theta);
                        currentDistance=currentDistance+prbDistance;
                    end
                end
            end
        end
        
        function y = getY(beam,outputMatrix,distance)
            %y coordinate value
            beam=beam.updateBeam(outputMatrix,outputMatrix);
            if distance < 0
                y=-(beam.getY(outputMatrix,100)-beam.getY(outputMatrix,100+distance));
            elseif distance == 0
                y=0;
            else
                currentDistance=0;
                y=0;
                for i=1:length(beam.prbBeams)
                    prbDistance=beam.crossSection.prbModel(i,1)*100;
                    if currentDistance + prbDistance >= distance || i == length(beam.prbBeams)
                        modifier=(distance-currentDistance)/prbDistance;
                        y=y+modifier*beam.prbBeams(i).length*sin(beam.prbBeams(i).theta);
                        return;
                    else
                        y=y+beam.prbBeams(i).length*sin(beam.prbBeams(i).theta);
                        currentDistance=currentDistance+prbDistance;
                    end
                end
            end
        end
        
        function gradientX = getGradientX(beam,index,distance)
            %gradient in x coordinates
            if distance < 0
                gradientX=-(beam.getGradientX(index,100)-beam.getGradientX(index,100+distance));
            elseif distance == 0
                gradientX=zeros(max(index),1);
            else
                gradientX=zeros(max(index),1);
                currentDistance=0;
                for i=1:length(beam.prbBeams)
                    prbDistance=beam.crossSection.prbModel(i,1)*100;
                    if currentDistance + prbDistance >= distance || i == length(beam.prbBeams)
                        modifier=(distance-currentDistance)/prbDistance;
                        if logical(beam.prbBeams(i).index(1))
                            gradientX(beam.prbBeams(i).index(1),1)=-modifier*beam.prbBeams(i).length*sin(beam.prbBeams(i).theta);
                        end
                        if logical(beam.prbBeams(i).index(2))
                            gradientX(beam.prbBeams(i).index(2),1)=modifier*cos(beam.prbBeams(i).theta);
                        end
                        return;
                    else
                        if logical(beam.prbBeams(i).index(1))
                            gradientX(beam.prbBeams(i).index(1),1)=-beam.prbBeams(i).length*sin(beam.prbBeams(i).theta);
                        end
                        if logical(beam.prbBeams(i).index(2))
                            gradientX(beam.prbBeams(i).index(2),1)=cos(beam.prbBeams(i).theta);
                        end
                        currentDistance=currentDistance+prbDistance;
                    end
                end
            end
        end
        
        function gradientY = getGradientY(beam,index,distance)
            %gradient in x coordinates
            if distance < 0
                gradientY=-(beam.getGradientY(index,100)-beam.getGradientY(index,100+distance));
            elseif distance == 0
                gradientY=zeros(max(index),1);
            else
                gradientY=zeros(max(index),1);
                currentDistance=0;
                for i=1:length(beam.prbBeams)
                    prbDistance=beam.crossSection.prbModel(i,1)*100;
                    if currentDistance + prbDistance >= distance || i == length(beam.prbBeams)
                        modifier=(distance-currentDistance)/prbDistance;
                        if logical(beam.prbBeams(i).index(1))
                            gradientY(beam.prbBeams(i).index(1),1)=modifier*beam.prbBeams(i).length*cos(beam.prbBeams(i).theta);
                        end
                        if logical(beam.prbBeams(i).index(2))
                            gradientY(beam.prbBeams(i).index(2),1)=modifier*sin(beam.prbBeams(i).theta);
                        end
                        return;
                    else
                        if logical(beam.prbBeams(i).index(1))
                            gradientY(beam.prbBeams(i).index(1),1)=beam.prbBeams(i).length*cos(beam.prbBeams(i).theta);
                        end
                        if logical(beam.prbBeams(i).index(2))
                            gradientY(beam.prbBeams(i).index(2),1)=sin(beam.prbBeams(i).theta);
                        end
                        currentDistance=currentDistance+prbDistance;
                    end
                end
            end
        end
        
        function hessianX = getHessianX(beam,index,distance)
            %gradient in x coordinates
            if distance < 0
                hessianX=-(beam.getHessianX(index,100)-beam.getHessianX(index,100+distance));
            elseif distance == 0
                hessianX=sparse(max(index),max(index));
            else
                ii=[];j=[];v=[];
                currentDistance=0;
                for i=1:length(beam.prbBeams)
                    prbDistance=beam.crossSection.prbModel(i,1)*100;
                    if currentDistance + prbDistance >= distance || i == length(beam.prbBeams)
                        modifier=(distance-currentDistance)/prbDistance;
                        if logical(beam.prbBeams(i).index(1))
                            ii(end+1)=beam.prbBeams(i).index(1);j(end+1)=beam.prbBeams(i).index(1);
                            v(end+1)=-modifier*beam.prbBeams(i).length*cos(beam.prbBeams(i).theta);
                        end
                        if logical(beam.prbBeams(i).index(2)) && logical(beam.prbBeams(i).index(1))
                            ii(end+1)=beam.prbBeams(i).index(1);j(end+1)=beam.prbBeams(i).index(2);
                            v(end+1)=-modifier*sin(beam.prbBeams(i).theta);
                            ii(end+1)=beam.prbBeams(i).index(2);j(end+1)=beam.prbBeams(i).index(1);
                            v(end+1)=-modifier*sin(beam.prbBeams(i).theta);
                        end
                        hessianX=sparse(ii,j,v,max(index),max(index));
                        return;
                    else
                        if logical(beam.prbBeams(i).index(1))
                            ii(end+1)=beam.prbBeams(i).index(1);j(end+1)=beam.prbBeams(i).index(1);
                            v(end+1)=-beam.prbBeams(i).length*cos(beam.prbBeams(i).theta);
                        end
                        if logical(beam.prbBeams(i).index(2)) && logical(beam.prbBeams(i).index(1))
                            ii(end+1)=beam.prbBeams(i).index(1);j(end+1)=beam.prbBeams(i).index(2);
                            v(end+1)=-sin(beam.prbBeams(i).theta);
                            ii(end+1)=beam.prbBeams(i).index(2);j(end+1)=beam.prbBeams(i).index(1);
                            v(end+1)=-sin(beam.prbBeams(i).theta);
                        end
                        currentDistance=currentDistance+prbDistance;
                    end
                end
            end
        end
        
        
        function hessianY = getHessianY(beam,index,distance)
            %hessian in y coordinates
            if distance < 0
                hessianY=-(beam.getHessianY(index,100)-beam.getHessianY(index,100+distance));
            elseif distance == 0
                hessianY=sparse(max(index),max(index));
            else
                ii=[];j=[];v=[];
                currentDistance=0;
                for i=1:length(beam.prbBeams)
                    prbDistance=beam.crossSection.prbModel(i,1)*100;
                    if currentDistance + prbDistance >= distance || i == length(beam.prbBeams)
                        modifier=(distance-currentDistance)/prbDistance;
                        if logical(beam.prbBeams(i).index(1))
                            ii(end+1)=beam.prbBeams(i).index(1);j(end+1)=beam.prbBeams(i).index(1);
                            v(end+1)=-modifier*beam.prbBeams(i).length*sin(beam.prbBeams(i).theta);
                        end
                        if logical(beam.prbBeams(i).index(2)) && logical(beam.prbBeams(i).index(1))
                            ii(end+1)=beam.prbBeams(i).index(1);j(end+1)=beam.prbBeams(i).index(2);
                            v(end+1)=modifier*cos(beam.prbBeams(i).theta);
                            ii(end+1)=beam.prbBeams(i).index(2);j(end+1)=beam.prbBeams(i).index(1);
                            v(end+1)=modifier*cos(beam.prbBeams(i).theta);
                        end
                        hessianY=sparse(ii,j,v,max(index),max(index));
                        return;
                    else
                        if logical(beam.prbBeams(i).index(1))
                            ii(end+1)=beam.prbBeams(i).index(1);j(end+1)=beam.prbBeams(i).index(1);
                            v(end+1)=-beam.prbBeams(i).length*sin(beam.prbBeams(i).theta);
                        end
                        if logical(beam.prbBeams(i).index(2)) && logical(beam.prbBeams(i).index(1))
                            ii(end+1)=beam.prbBeams(i).index(1);j(end+1)=beam.prbBeams(i).index(2);
                            v(end+1)=cos(beam.prbBeams(i).theta);
                            ii(end+1)=beam.prbBeams(i).index(2);j(end+1)=beam.prbBeams(i).index(1);
                            v(end+1)=cos(beam.prbBeams(i).theta);
                        end
                        currentDistance=currentDistance+prbDistance;
                    end
                end
                
            end
        end
        %%
        %Energy Functions
        %prbBeams=struct('length0',[],'length',[],'theta',0,'theta0',0,'linearSpring',0,'index',[0 0]);
        %torsionSprings=struct('magnitude',[]);
        function energy = getEnergy(beam,workspace)
            %get energy
            %torsion springs
            energy=1/2*beam.torsionSprings(1).magnitude*(beam.prbBeams(1).theta-beam.prbBeams(1).theta0)^2;
            for i=2:length(beam.torsionSprings)
                energy=energy+1/2*beam.torsionSprings(i).magnitude*workspace.torsionSpringFactor()*((beam.prbBeams(i).theta-beam.prbBeams(i).theta0)-(beam.prbBeams(i-1).theta-beam.prbBeams(i-1).theta0))^2;
            end
            %linear springs
            for i=1:length(beam.prbBeams)
                if logical(beam.prbBeams(i).linearSpring)
                    energy=energy+1/2*beam.prbBeams(i).linearSpring*workspace.linearSpringFactor()*((beam.prbBeams(i).length-beam.prbBeams(i).length0)*workspace.lengthFactor())^2;
                end
            end
        end
        
        function gradient=getGradient(beam,index,workspace)
            %get energy gradient
            gradient=zeros(max(index),1);
            %torsion springs
            if logical(beam.prbBeams(1).index(1))
                gradient(beam.prbBeams(1).index(1))= beam.torsionSprings(1).magnitude*workspace.torsionSpringFactor()*(beam.prbBeams(1).theta-beam.prbBeams(1).theta0);
            end
            for i=2:length(beam.torsionSprings)
                if logical(beam.prbBeams(i).index(1))
                    gradient(beam.prbBeams(i).index(1))= gradient(beam.prbBeams(i).index(1))+beam.torsionSprings(i).magnitude*workspace.torsionSpringFactor()*((beam.prbBeams(i).theta-beam.prbBeams(i).theta0)-(beam.prbBeams(i-1).theta-beam.prbBeams(i-1).theta0));
                end
                if logical(beam.prbBeams(i-1).index(1))
                    gradient(beam.prbBeams(i-1).index(1))= gradient(beam.prbBeams(i-1).index(1)) - beam.torsionSprings(i).magnitude*workspace.torsionSpringFactor()*((beam.prbBeams(i).theta-beam.prbBeams(i).theta0)-(beam.prbBeams(i-1).theta-beam.prbBeams(i-1).theta0));
                end
            end
            %linear springs
            for i=1:length(beam.prbBeams)
                if logical(beam.prbBeams(i).linearSpring)
                    if logical(beam.prbBeams(i).index(2))
                        gradient(beam.prbBeams(i).index(2))= gradient(beam.prbBeams(i).index(2))+beam.prbBeams(i).linearSpring*workspace.linearSpringFactor()*(beam.prbBeams(i).length-beam.prbBeams(i).length0)*workspace.lengthFactor()^2;
                    end
                end
            end
        end
        
        function hessian=getHessian(beam,index,workspace)
            %get energy hessian
            ii=[];j=[];v=[];
            %torsion springs
            if logical(beam.prbBeams(1).index(1))
                ii(end+1)=beam.prbBeams(1).index(1);j(end+1)=beam.prbBeams(1).index(1);
                v(end+1)=beam.torsionSprings(1).magnitude;
            end
            for i=2:length(beam.torsionSprings)
                if logical(beam.prbBeams(i).index(1))
                    ii(end+1)=beam.prbBeams(i).index(1);j(end+1)=beam.prbBeams(i).index(1);
                    v(end+1)=beam.torsionSprings(i).magnitude*workspace.torsionSpringFactor();
                end
                if logical(beam.prbBeams(i-1).index(1))
                    ii(end+1)=beam.prbBeams(i-1).index(1);j(end+1)=beam.prbBeams(i-1).index(1);
                    v(end+1)=beam.torsionSprings(i).magnitude*workspace.torsionSpringFactor();
                end
                if logical(beam.prbBeams(i).index(1)) && logical(beam.prbBeams(i-1).index(1))
                    ii(end+1)=beam.prbBeams(i-1).index(1);j(end+1)=beam.prbBeams(i).index(1);
                    v(end+1)=-beam.torsionSprings(i).magnitude*workspace.torsionSpringFactor();
                    ii(end+1)=beam.prbBeams(i).index(1);j(end+1)=beam.prbBeams(i-1).index(1);
                    v(end+1)=-beam.torsionSprings(i).magnitude*workspace.torsionSpringFactor();
                end
            end
            %linear springs
            for i=1:length(beam.prbBeams)
                if logical(beam.prbBeams(i).linearSpring)
                    if logical(beam.prbBeams(i).index(2))
                        ii(end+1)=beam.prbBeams(i).index(2);j(end+1)=beam.prbBeams(i).index(2);
                        v(end+1)=beam.prbBeams(i).linearSpring*workspace.linearSpringFactor()*workspace.lengthFactor()^2;
                    end
                end
            end
            hessian=sparse(ii,j,v,max(index),max(index));
        end
        
        function [angle,index]=getAngle(beam,distance)
            %get angle and index
            currentDistance=0;
            angle=[];
            for i=1:length(beam.prbBeams)
                prbDistance=beam.crossSection.prbModel(i,1)*100;
                if currentDistance + prbDistance >= distance
                    angle=beam.prbBeams(i).theta;
                    index=beam.prbBeams(i).index(1);
                    return;
                end
            end
            %numerical error check
            if isempty(angle)
                angle=beam.prbBeams(end).theta;
                index=beam.prbBeams(end).index(1);
            end
        end
        
        
        %inverse problem - solve for force
        
        function jacobian=getJacobian(beam,workspace)
            %get jacobian
            n=size(beam.crossSection.prbModel,1);
            jacobian=zeros(3,length(find(beam.crossSection.prbModel(:,2:end)~= Inf)));
            index=1;
            for i=1:n
                if beam.crossSection.prbModel(i,2) ~= Inf
                    sinSum=0;
                    cosSum=0;
                    for j=i:n
                        sinSum=sinSum-beam.crossSection.prbModel(j,1)*sin(beam.prbBeams(j).theta-beam.prbBeams(1).theta);
                        cosSum=cosSum+beam.crossSection.prbModel(j,1)*cos(beam.prbBeams(j).theta-beam.prbBeams(1).theta);
                    end
                    jacobian(1,index)=sinSum;
                    jacobian(2,index)=cosSum;
                    jacobian(3,index)=1;
                    index=index+1;
                end
                if beam.crossSection.prbModel(i,3) ~= Inf
                    jacobian(1,index)=cos(beam.prbBeams(i).theta-beam.prbBeams(1).theta)/(beam.length0*workspace.lengthFactor());
                    jacobian(2,index)=sin(beam.prbBeams(i).theta-beam.prbBeams(1).theta)/(beam.length0*workspace.lengthFactor());
                    jacobian(3,index)=0;
                    index=index+1;
                end
            end
        end
        
        function forceVector=getForceVector(beam,workspace)
            %get force vector
            n=size(beam.crossSection.prbModel,1);
            forceVector=zeros(length(find(beam.crossSection.prbModel(:,2:end)~= Inf)),1);
            index=1;
            for i=1:n
                if beam.crossSection.prbModel(i,2) ~= Inf
                    forceVector(index,1)=beam.torsionSprings(i).magnitude*workspace.torsionSpringFactor()*(beam.prbBeams(i).theta-beam.prbBeams(i-1).theta);
                    index=index+1;
                end
                if beam.crossSection.prbModel(i,3) ~= Inf
                    forceVector(index,1)=beam.prbBeams(i).linearSpring*workspace.linearSpringFactor()*(beam.prbBeams(i).length-beam.prbBeams(i).length0)*workspace.lengthFactor()^2;
                    index=index+1;
                end
            end
        end
        
        function force=getForceInverse(beam,workspace)
            %find the force using inverse jacobian
            dof=length(find(beam.crossSection.prbModel(:,2:3)~= Inf));
            if dof <3
                dofAssigned=0;
                index=1;
                while dofAssigned < 3-dof
                    if  isinf(beam.crossSection.prbModel(index,3))
                        beam.crossSection.prbModel(index,3)=1e3;
                        dofAssigned=dofAssigned+1;
                    end
                    index=index+1;
                end
                %                beam=beam.solvePRB();
            end
            if sum(abs(beam.getForceVector(workspace))) > 1e-10
                force=pinv(beam.getJacobian(workspace)')*beam.getForceVector(workspace);
                %            force=beam.getForceVector(workspace)\beam.getJacobian(workspace)'
            else
                force=[0;0;0];
            end
            if logical(isnan(force))
                force=[0;0;0];
            else
                force(1:2)=force(1:2)/(beam.length0*workspace.lengthFactor());
            end
        end
        
        %% miscellaneous functions
        
        function rotationMatrix=getRotationMatrix(beam)
            %get the rotation matrix
            realAngle=beam.prbBeams(1).theta;
            rotationMatrix=[cos(realAngle) -sin(realAngle);sin(realAngle) cos(realAngle)];
        end
        
        function beam=getStressShape(beam,workspace)
            %fill the stress values
            forces=beam.getForceInverse(workspace);
            euler=EulerBeam(beam.length0*workspace.lengthFactor(),beam.crossSection.E*workspace.EFactor()...
                ,beam.crossSection.thickness*workspace.lengthFactor()...
                ,beam.crossSection.width*workspace.lengthFactor(),...
                forces(1),forces(2),forces(3));
            euler=euler.analytical();
            beam.stressValues=abs(euler.internalStress());
            beam.xValues=zeros(length(euler.x),1);
            beam.yValues=zeros(length(euler.x),1);
            for i=1:length(euler.x)
                pos=beam.getRotationMatrix()*[euler.x(i);euler.y(i)];
                beam.xValues(i)=pos(1)/workspace.lengthFactor();
                beam.yValues(i)=pos(2)/workspace.lengthFactor();
            end
        end
        
        function [x,y,bendingStress]=getShape(beam,workspace,node1)
            %get beam shape
            forces=beam.getForceInverse(workspace);
            euler=EulerBeam(beam.length0*workspace.lengthFactor(),beam.crossSection.E*workspace.EFactor()...
                ,beam.crossSection.thickness*workspace.lengthFactor()...
                ,beam.crossSection.width*workspace.lengthFactor(),...
                forces(1),forces(2),forces(3));
            euler=euler.analytical();
            x=zeros(length(euler.x),1);
            y=zeros(length(x),1);
            for i=1:length(x)
                pos=beam.getRotationMatrix()*[euler.x(i);euler.y(i)];
                x(i)=pos(1)/workspace.lengthFactor()+node1.x;
                y(i)=pos(2)/workspace.lengthFactor()+node1.y;
            end
            bendingStress=abs(euler.internalStress());
        end
        
        function force=getForce(beam,workspace)
            %get tip force in local coordinate
            forces=beam.getForceInverse(workspace);
            %to global coordinate
            force(1,1)=(forces(1)*cos(beam.theta0)-forces(2)*sin(beam.theta0))/workspace.forceFactor();
            force(1,2)=(forces(1)*sin(beam.theta0)+forces(2)*cos(beam.theta0))/workspace.forceFactor();
            force(1,3)=forces(3)/workspace.forceFactor()/workspace.lengthFactor();
        end
        
        
        function beam=drawLink(beam,currentAxis,nodes,limit,mode,parent)
            %draw the link
            node1=nodes(beam.nodes(1,1)).getNode;
            node2=nodes(beam.nodes(1,2)).getNode;
            nodeX=node1.x;
            nodeY=node1.y;
            for i=1:length(beam.prbBeams)
                deltaX=beam.prbBeams(i).length*cos(beam.prbBeams(i).theta);
                deltaY=beam.prbBeams(i).length*sin(beam.prbBeams(i).theta);
                [xx,yy]=DrawLink.drawLine([Point(nodeX,nodeY),Point(nodeX+deltaX,nodeY+deltaY)],[Joint.Pin Joint.Pin],limit,beam.prbBeams(i).linearSpring);
                xPRB{i}=xx;yPRB{i}=yy;
                nodeX=nodeX+deltaX;
                nodeY=nodeY+deltaY;
            end
            
            [x,y,bendingStress]=beam.getShape(parent.getWorkspace(),node1);
            
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
            distancePRBLine=sqrt((x(end) - xPRB{end}(end))^2+ (y(end) - yPRB{end}(end))^2);
            prbLength=sqrt((xPRB{1}(end) - xPRB{end}(end))^2+ (yPRB{1}(end) - yPRB{end}(end))^2);
            if ~isempty(x)
                if ~isgraphics(beam.line(1,1))
                    %draw PRB
                    for i=1:length(xPRB)
                        beam.line(1,i)=plot(currentAxis,xPRB{i},yPRB{i},'LineWidth',0.5,'LineStyle','-','Color',Colors.Inactive.getColor());
                    end
                    if distancePRBLine < 0.01*prbLength
                        beam.line(1,end+1)=surf('XData',[x x],...
                            'YData',[y y],...
                            'ZData',zeros(length(y),2),...
                            'CData',[bendingStress'/1e6 bendingStress'/1e6],...
                            'FaceColor','none',...
                            'EdgeColor','interp',...
                            'Marker','none','LineWidth',lineWidthBeam,'Parent',currentAxis);

                        colormap(currentAxis,'jet');
                        beam.colorBar=colorbar(currentAxis,'east');
                        beam.colorBar.Label.String = 'Stress (MPa)';
                    end
                else
                    for i=1:length(xPRB)
                        beam.line(1,i).XData=xPRB{i};
                        beam.line(1,i).YData=yPRB{i};
                    end
                    if distancePRBLine < 0.01*prbLength && size(beam.line,2) > length(xPRB) && isgraphics(beam.line(1,end))
                        beam.line(1,end).XData=[x x];
                        beam.line(1,end).YData=[y y];
                        beam.line(1,end).CData=[bendingStress'/1e6 bendingStress'/1e6];
                    elseif distancePRBLine < 0.01*prbLength
                        beam.line(1,end+1)=surf('XData',[x x],...
                            'YData',[y y],...
                            'ZData',zeros(length(y),2),...
                            'CData',[bendingStress'/1e6 bendingStress'/1e6],...
                            'FaceColor','none',...
                            'EdgeColor','interp',...
                            'Marker','none','LineWidth',lineWidthBeam,'Parent',currentAxis);

                        colormap(currentAxis,'jet');
                        beam.colorBar=colorbar(currentAxis,'east');
                        beam.colorBar.Label.String = 'Stress (MPa)';
                    elseif size(beam.line,2) > length(xPRB)
                        delete(beam.line(1,end));
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
            nodeX=node1.x;
            nodeY=node1.y;
            for i=1:length(beam.prbBeams)
                deltaX=beam.prbBeams(i).length*cos(beam.prbBeams(i).theta);
                deltaY=beam.prbBeams(i).length*sin(beam.prbBeams(i).theta);
                [xx,yy]=DrawLink.drawLine([Point(nodeX,nodeY),Point(nodeX+deltaX,nodeY+deltaY)],[Joint.Pin Joint.Pin],limit,beam.prbBeams(i).linearSpring);
                xPRB{i}=xx;yPRB{i}=yy;
                nodeX=nodeX+deltaX;
                nodeY=nodeY+deltaY;
            end
            
            [x,y,bendingStress]=beam.getShape(workspace,node1);
            
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
                    %draw PRB
                    for i=1:length(xPRB)
                        beam.line(1,i)=plot(currentAxis,xPRB{i},yPRB{i},'LineWidth',0.5,'LineStyle','-','Color',Colors.Inactive.getColor());
                    end
                    
                    beam.line(1,end+1)=surf('XData',[x x],...
                        'YData',[y y],...
                        'ZData',zeros(length(y),2),...
                        'CData',[bendingStress'/1e6 bendingStress'/1e6],...
                        'FaceColor','none',...
                        'EdgeColor','interp',...
                        'Marker','none','LineWidth',lineWidthBeam,'Parent',currentAxis);
                    
                    colormap(currentAxis,'jet');
                    beam.colorBar=colorbar(currentAxis,'east');
                    beam.colorBar.Label.String = 'Stress (MPa)';
                else
                    for i=1:length(xPRB)
                        beam.line(1,i).XData=xPRB{i};
                        beam.line(1,i).YData=yPRB{i};
                    end
                    beam.line(1,end).XData=[x x];
                    beam.line(1,end).YData=[y y];
                    beam.line(1,end).CData=[bendingStress'/1e6 bendingStress'/1e6];
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


