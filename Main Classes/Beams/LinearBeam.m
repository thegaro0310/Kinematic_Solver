classdef LinearBeam < Beam
    properties
        angle;
        masterAngle;
        index;
        deltaX;
        deltaY;
        theta;
        %equations
        e;
        g;
        h;
        stress;
        yPos;
        %
        noOfData;
    end
    
    methods(Static)
        function text = getText()
            %get summary text
            text={['Linear Model is the approximate solution of the Euler-Bernoulli beam equation.'...
                'Closed form equations for the beam deformation and strain energy are available.'...
                ]; ['Linear extention is not possible with the linear model and this model is the fastest model while being reasonably accurate.' ...
                ' For more information visit:']; ....
                [ char(9) 'http:\\complaintanalysis.com\Linear']};
        end
        
        function accuracy=getAccuracy()
            %get analysis accuracy
            accuracy=[4 8];
        end
        
        function speed=getSpeed()
            %get analysis speed
            speed=[9 10];
        end
        
        function ext=getExpansion()
            %get allows expansion or not
            ext=false;
        end
    end
    
    methods
        function beam=LinearBeam(id,nodes,joints,angle,lengthBeam,crossSection)
            %constructor for the prb beam class
            beam@Beam(id,0,0,nodes,joints,angle,lengthBeam,crossSection);
            beam.angle=beam.theta0;
            beam.masterAngle=0;
            beam.deltaX=0;
            beam.deltaY=0;
            beam.theta=0;
            beam.index=zeros(1,5);
            beam=beam.loadEquations();
            beam.degreesOfFreedom=5;
        end
        
        function beam=loadEquations(beam)
            %load the BCM equations
            equations=load('LinearEquations.mat');
            beam.e=equations.e;
            beam.g=equations.g;
            beam.h=equations.h;
            beam.stress=equations.stress;
            beam.yPos=equations.yPos;
        end
        
        function beam=updateBeam(beam,inputs,index)
            %update bcm beams from inputs
            inputIndex=beam.id;
            %master angle
            beam.masterAngle=inputs(inputIndex);
            beam.index(1)=index(inputIndex);
            %delta X
            beam.deltaX=inputs(inputIndex+1);
            beam.index(2)=index(inputIndex+1);
            %delta Y
            beam.deltaY=inputs(inputIndex+2);
            beam.index(3)=index(inputIndex+2);
            %theta
            beam.theta=inputs(inputIndex+3);
            beam.index(4)=index(inputIndex+3);
            %real angle
            beam.angle=beam.masterAngle+beam.theta0;
        end
        
       function rotationMatrix=getRotationMatrix(beam,outputMatrix)
            %get the rotation matrix
            currentAngle=outputMatrix(beam.id)+beam.theta0;
            rotationMatrix=[cos(currentAngle) -sin(currentAngle);sin(currentAngle) cos(currentAngle)];
        end
        
        function rotationMatrix=getRotationGradientMatrix(beam,outputMatrix)
            %get the rotation gradient matrix
            currentAngle=outputMatrix(beam.id)+beam.theta0;
            rotationMatrix=[-sin(currentAngle) -cos(currentAngle);cos(currentAngle) -sin(currentAngle)];
        end
        
        function rotationMatrix=getRotationHessianMatrix(beam,outputMatrix)
            %get the rotation hessian matrix
            currentAngle=outputMatrix(beam.id)+beam.theta0;
            rotationMatrix=[-cos(currentAngle) sin(currentAngle);-sin(currentAngle) -cos(currentAngle)];
        end
        
        
        function deltaTheta=getDeltaTheta(beam)
            %get the delta theta from the list of the bcm beams
            deltaTheta=beam.theta;
            if logical(beam.index(1))
                deltaTheta=deltaTheta-beam.masterAngle;
            end
        end
        
        function initialGuess=getInitialGuess(beam)
            %get initial guess for the beam
            initialGuess=[beam.masterAngle;0;beam.deltaY;beam.theta];
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
                currentDeltaX=outputMatrix(beam.id+1);
                currentDeltaY=outputMatrix(beam.id+2);
                coord=beam.getRotationMatrix(outputMatrix)*[currentDeltaX+beam.length0;currentDeltaY];
                x=coord(1);
            end
        end
        
        function y = getY(beam,outputMatrix,distance)
            %x coordinate value
            if distance < 0
                y=-(beam.getY(outputMatrix,100)-beam.getY(outputMatrix,100+distance));
            elseif distance == 0
                y=0;
            else
                currentDeltaX=outputMatrix(beam.id+1);
                currentDeltaY=outputMatrix(beam.id+2);
                coord=beam.getRotationMatrix(outputMatrix)*[currentDeltaX+beam.length0;currentDeltaY];
                y=coord(2);
            end
        end
        
        function gradientX = getGradientX(beam,outputMatrix,index,distance)
            %gradient in x coordinates
            if distance < 0
                gradientX=-(beam.getGradientX(outputMatrix,index,100)-beam.getGradientX(outputMatrix,index,100+distance));
            elseif distance == 0
                gradientX=zeros(max(index),1);
            else
                gradientX=zeros(max(index),1);
                transformation=beam.getRotationMatrix(outputMatrix);
                beamGradient=[transformation(1,1:2)';0];
                transformation=beam.getRotationGradientMatrix(outputMatrix);
                currentDeltaX=outputMatrix(beam.id+1);
                currentDeltaY=outputMatrix(beam.id+2);
                beamTransGradient=transformation(1,:)*[currentDeltaX+beam.length0;currentDeltaY];
                %master angle
                if logical(index(beam.id))
                    gradientX(index(beam.id),1)=beamTransGradient;
                end
                %delta X
                if logical(index(beam.id+1))
                    gradientX(index(beam.id+1),1)=beamGradient(1);
                end
                %delta Y
                if logical(index(beam.id+2))
                    gradientX(index(beam.id+2),1)=beamGradient(2);
                end
                %theta
                if logical(index(beam.id+3))
                    gradientX(abs(index(beam.id+3)),1)=beamGradient(3);
                end
            end
        end
        
        function gradientY = getGradientY(beam,outputMatrix,index,distance)
            %gradient in x coordinates
            if distance < 0
                gradientY=-(beam.getGradientY(outputMatrix,index,100)-beam.getGradientY(outputMatrix,index,100+distance));
            elseif distance == 0
                gradientY=zeros(max(index),1);
            else
                gradientY=zeros(max(index),1);
                transformation=beam.getRotationMatrix(outputMatrix);
                beamGradient=[transformation(2,1:2)';0];
                transformation=beam.getRotationGradientMatrix(outputMatrix);
                currentDeltaX=outputMatrix(beam.id+1);
                currentDeltaY=outputMatrix(beam.id+2);
                beamTransGradient=transformation(2,:)*[currentDeltaX+beam.length0;currentDeltaY];
                %master angle
                if logical(index(beam.id))
                    gradientY(index(beam.id),1)=beamTransGradient;
                end
                %delta X
                if logical(index(beam.id+1))
                    gradientY(index(beam.id+1),1)=beamGradient(1);
                end
                %delta Y
                if logical(index(beam.id+2))
                    gradientY(index(beam.id+2),1)=beamGradient(2);
                end
                %theta
                if logical(index(beam.id+3))
                    gradientY(abs(index(beam.id+3)),1)=beamGradient(3);
                end
            end
        end
        
        function hessianX = getHessianX(beam,outputMatrix,index,distance)
            %gradient in x coordinates
            if distance < 0
                hessianX=-(beam.getHessianX(outputMatrix,index,100)-beam.getHessianX(outputMatrix,index,100+distance));
            elseif distance == 0
                hessianX=sparse(max(index),max(index));
            else
                i=[];j=[];v=[];
                transformationGradient=beam.getRotationGradientMatrix(outputMatrix);
                transformationHessian=beam.getRotationHessianMatrix(outputMatrix);
                %check if there is master
                if logical(index(beam.id))
                    %master theta-master theta
                    i(end+1)=index(beam.id);j(end+1)=index(beam.id);
                    currentDeltaX=outputMatrix(beam.id+1);
                    currentDeltaY=outputMatrix(beam.id+2);
                    v(end+1)=transformationHessian(1,:)*[currentDeltaX+beam.length0;currentDeltaY];
                    %x-master theta
                    if logical(index(beam.id+1))
                        i(end+1)=index(beam.id);j(end+1)=index(beam.id+1);
                        v(end+1)=transformationGradient(1,1);
                        i(end+1)=index(beam.id+1);j(end+1)=index(beam.id);
                        v(end+1)=transformationGradient(1,1);
                    end
                    %y-master theta
                    if logical(index(beam.id+2))
                        i(end+1)=index(beam.id);j(end+1)=index(beam.id+2);
                        v(end+1)=transformationGradient(1,2);
                        i(end+1)=index(beam.id+2);j(end+1)=index(beam.id);
                        v(end+1)=transformationGradient(1,2);
                    end
                end
                hessianX=sparse(i,j,v,max(index),max(index));
            end
        end
        
        function hessianY = getHessianY(beam,outputMatrix,index,distance)
            %gradient in x coordinates
            if distance < 0
                hessianY=-(beam.getHessianY(outputMatrix,index,100)-beam.getHessianY(outputMatrix,index,100+distance));
            elseif distance == 0
                hessianY=sparse(max(index),max(index));
            else
                i=[];j=[];v=[];
                transformationGradient=beam.getRotationGradientMatrix(outputMatrix);
                transformationHessian=beam.getRotationHessianMatrix(outputMatrix);
                %check if there is master
                if logical(index(beam.id))
                    %master theta-master theta
                    i(end+1)=index(beam.id);j(end+1)=index(beam.id);
                    currentDeltaX=outputMatrix(beam.id+1);
                    currentDeltaY=outputMatrix(beam.id+2);
                    v(end+1)=transformationHessian(2,:)*[currentDeltaX+beam.length0;currentDeltaY];
                    %x-master theta
                    if logical(index(beam.id+1))
                        i(end+1)=index(beam.id);j(end+1)=index(beam.id+1);
                        v(end+1)=transformationGradient(2,1);
                        i(end+1)=index(beam.id+1);j(end+1)=index(beam.id);
                        v(end+1)=transformationGradient(2,1);
                    end
                    %y-master theta
                    if logical(index(beam.id+2))
                        i(end+1)=index(beam.id);j(end+1)=index(beam.id+2);
                        v(end+1)=transformationGradient(2,2);
                        i(end+1)=index(beam.id+2);j(end+1)=index(beam.id);
                        v(end+1)=transformationGradient(2,2);
                    end
                end
                hessianY=sparse(i,j,v,max(index),max(index));
            end
        end
        
        function [angle,index]=getAngle(beam,~)
            %theta
            angle=beam.theta;
            index=beam.index(5);
        end
        
        %%
        %Energy Functions
        function energy = getEnergy(beam,workspace)
            %get energy
            deltaTheta=beam.getDeltaTheta(); 
            %calculate energy
            energy=beam.e(beam.deltaX*workspace.lengthFactor(),beam.deltaY*workspace.lengthFactor(),deltaTheta,...
                beam.length0*workspace.lengthFactor(),beam.crossSection.E*workspace.EFactor()*beam.crossSection.getI()*workspace.lengthFactor()^4,beam.crossSection.thickness*workspace.lengthFactor()); 
        end
        
        function gradient=getGradient(beam,index,workspace)
            %get energy gradient
            gradient=zeros(max(index),1);
            deltaTheta=beam.getDeltaTheta();
            beamGradient=beam.g(beam.deltaX*workspace.lengthFactor(),beam.deltaY*workspace.lengthFactor(),deltaTheta,beam.length0*workspace.lengthFactor(),...
                beam.crossSection.E*workspace.EFactor()*beam.crossSection.getI()*workspace.lengthFactor()^4,beam.crossSection.thickness*workspace.lengthFactor()); 
            gradient(index(beam.id+1),1)=beamGradient(1)*workspace.lengthFactor();
            gradient(index(beam.id+2),1)=beamGradient(2)*workspace.lengthFactor();
            gradient(index(beam.id+3),1)=beamGradient(3);
            %update master angle
            if logical(beam.index(1))
                gradient(index(beam.id),1)=-beamGradient(3);
            end
        end
        
        function hessian=getHessian(beam,index,workspace)
            %get energy hessian
            deltaTheta=beam.getDeltaTheta();
            beamHessian=beam.h(beam.deltaX*workspace.lengthFactor(),beam.deltaY*workspace.lengthFactor(),deltaTheta,beam.length0*workspace.lengthFactor(),...
                beam.crossSection.E*workspace.EFactor()*beam.crossSection.getI()*workspace.lengthFactor()^4,beam.crossSection.thickness*workspace.lengthFactor())*workspace.lengthFactor()^2;
            beamHessian(:,3)=beamHessian(:,3)/workspace.lengthFactor();
            beamHessian(3,:)=beamHessian(3,:)/workspace.lengthFactor();
            i=[];j=[];v=[];
            %x-x
            if logical(beam.index(2))
                i(end+1)=index(beam.id+1);j(end+1)=index(beam.id+1);
                v(end+1)=beamHessian(1,1);
            end
            %x-y
            if logical(beam.index(2)) && logical(beam.index(3))
                i(end+1)=index(beam.id+1);j(end+1)=index(beam.id+2);
                v(end+1)=beamHessian(1,2);
                i(end+1)=index(beam.id+2);j(end+1)=index(beam.id+1);
                v(end+1)=beamHessian(1,2);
            end
            %x-theta
            if logical(beam.index(2)) && logical(beam.index(4))
                i(end+1)=index(beam.id+1);j(end+1)=index(beam.id+3);
                v(end+1)=beamHessian(1,3);
                i(end+1)=index(beam.id+3);j(end+1)=index(beam.id+1);
                v(end+1)=beamHessian(1,3);
            end
            %y-y
            if logical(beam.index(3))
                i(end+1)=index(beam.id+2);j(end+1)=index(beam.id+2);
                v(end+1)=beamHessian(2,2);
            end
            %y-theta
            if logical(beam.index(3)) && logical(beam.index(4))
                i(end+1)=index(beam.id+2);j(end+1)=index(beam.id+3);
                v(end+1)=beamHessian(2,3);
                i(end+1)=index(beam.id+3);j(end+1)=index(beam.id+2);
                v(end+1)=beamHessian(2,3);
            end
            %theta-theta
            if logical(beam.index(4))
                i(end+1)=index(beam.id+3);j(end+1)=index(beam.id+3);
                v(end+1)=beamHessian(3,3);
            end
            %other beam theta dependencies
            if logical(beam.index(1))
                %mastertheta-master theta
                i(end+1)=index(beam.id);j(end+1)=index(beam.id);
                v(end+1)=beamHessian(3,3);
                %theta-master theta
                if logical(beam.index(4))
                    i(end+1)=index(beam.id);j(end+1)=index(beam.id+3);
                    v(end+1)=-beamHessian(3,3);
                    i(end+1)=index(beam.id+3);j(end+1)=index(beam.id);
                    v(end+1)=-beamHessian(3,3);
                end
                %x-master theta
                if logical(beam.index(2))
                    i(end+1)=index(beam.id);j(end+1)=index(beam.id+1);
                    v(end+1)=-beamHessian(1,3);
                    i(end+1)=index(beam.id+1);j(end+1)=index(beam.id);
                    v(end+1)=-beamHessian(1,3);
                end
                %y-master theta
                if logical(beam.index(3))
                    i(end+1)=index(beam.id);j(end+1)=index(beam.id+2);
                    v(end+1)=-beamHessian(2,3);
                    i(end+1)=index(beam.id+2);j(end+1)=index(beam.id);
                    v(end+1)=-beamHessian(2,3);
                end
            end
            hessian=sparse(i,j,v,max(index),max(index));
        end
        
        %% miscellaneous function
        function rotationMatrix=getUpdatedRotationMatrix(beam)
            %get the rotation matrix
            currentAngle=beam.angle;
            rotationMatrix=[cos(currentAngle) -sin(currentAngle);sin(currentAngle) cos(currentAngle)];
        end
        
        function [x,y,bendingStress]=getShape(beam,workspace)
            %get beam shape
            ux=linspace(0,beam.length0,beam.noOfData)'+linspace(0,beam.deltaX,beam.noOfData)'; 
            deltaTheta=beam.getDeltaTheta();
            bendingStress=abs(beam.stress(beam.deltaX*workspace.lengthFactor(),beam.deltaY*workspace.lengthFactor(),deltaTheta,beam.crossSection.thickness*workspace.lengthFactor(),...
                beam.crossSection.E*workspace.EFactor(),ux*workspace.lengthFactor(),beam.length0*workspace.lengthFactor())); 
            uy=beam.yPos(beam.deltaX*workspace.lengthFactor(),beam.deltaY*workspace.lengthFactor(),deltaTheta,beam.length0*workspace.lengthFactor(),...
                beam.crossSection.E*workspace.EFactor()*beam.crossSection.getI()*workspace.lengthFactor()^4,ux*workspace.lengthFactor())/workspace.lengthFactor();
            x=zeros(length(ux),1);
            y=zeros(length(uy),1);
            for i=1:length(ux)
                pos=beam.getUpdatedRotationMatrix()*[ux(i);uy(i)];
                x(i)=pos(1);
                y(i)=pos(2);
            end
        end
        
        
    end
    
end

