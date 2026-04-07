classdef BcmBeam < Beam
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
        p;
        fm;
        %
        noOfData;
    end
    
    methods(Static)
        function text = getText()
            %get summary text
            text={['Beam Constraint Model was developed for accurately analyzing beams in the intermediate deflection range.'...
                ' Multiple segment BCM can accurately handle large deflections.'...
                ]; ['BCM have closed form equations for the beam deformation and strain energy. Axial strain is present in the BCM Beam.' ...
                ' For more information visit:']; ....
                [ char(9) 'http:\\complaintanalysis.com\BCM']};
        end
        
        function accuracy=getAccuracy()
            %get analysis accuracy
            accuracy=[8 10];
        end
        
        function speed=getSpeed()
            %get analysis speed
            speed=[5 8];
        end
        
        function ext=getExpansion()
            %get allows expansion or not
            ext=true;
        end
    end
    
    
    methods
        function beam=BcmBeam(id,nodes,joints,angle,lengthBeam,crossSection)
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
            equations=load('BCMEquations.mat');
            beam.e=equations.e;
            beam.g=equations.g;
            beam.h=equations.h;
            beam.p=equations.p;
            beam.fm=equations.fm;
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
        
%         function deltaTheta=getDeltaTheta(beam,outputMatrix,inputMatrix)
%             %get the delta theta from the list of the bcm beams
%             deltaTheta=outputMatrix(beam.id+3);
%             %master angle
%             if logical(inputMatrix(beam.id+1))
%                 deltaTheta=deltaTheta-outputMatrix(beam.id);
%             end
%         end
        
        function deltaTheta=getDeltaTheta(beam)
            %get the delta theta from the list of the bcm beams
            deltaTheta=beam.theta;
            if logical(beam.index(1))
                deltaTheta=deltaTheta-beam.masterAngle;
            end
        end
        
        function initialGuess=getInitialGuess(beam)
            %get initial guess for the beam
            initialGuess=[beam.masterAngle;beam.deltaX;beam.deltaY;beam.theta];
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
        
        %%
%         %update bcm beams from inputs
%             inputIndex=beam.id;
%             %master angle
%             beam.masterAngle=inputs(inputIndex);
%             beam.index(1)=index(inputIndex);
%             %delta X
%             beam.deltaX=inputs(inputIndex+1);
%             beam.index(2)=index(inputIndex+1);
%             %delta Y
%             beam.deltaY=inputs(inputIndex+2);
%             beam.index(3)=index(inputIndex+2);
%             %theta
%             beam.theta=inputs(inputIndex+3);
%             beam.index(4)=index(inputIndex+3);
%             %real angle
%             beam.angle=beam.masterAngle+beam.theta0;
        %Energy Functions
        function energy = getEnergy(beam,outputMatrix,index,workspace)
            %get energy
            deltaTheta=beam.getDeltaTheta(outputMatrix,index);
            currentDeltaX=outputMatrix(beam.id+1);
            currentDeltaY=outputMatrix(beam.id+2);
            %calculate energy
            %energy=beam.e(beam.deltaX,beam.deltaY,deltaTheta,beam.crossSection.thickness....
             %   ,beam.length0)*beam.crossSection.E*workspace.EFactor()*beam.crossSection.getI()/beam.length0*workspace.lengthFactor()^3;
            energy=test(currentDeltaX,currentDeltaY,deltaTheta,beam.crossSection.thickness....
                ,beam.length0)*beam.crossSection.E*workspace.EFactor()*beam.crossSection.getI()/beam.length0*workspace.lengthFactor()^3;
        end
        
        function gradient=getGradient(beam,outputMatrix,index,workspace)
            %get energy gradient
            gradient=zeros(max(index),1);
            deltaTheta=beam.getDeltaTheta(outputMatrix,index);
            currentDeltaX=outputMatrix(beam.id+1);
            currentDeltaY=outputMatrix(beam.id+2);
            %beamGradient=beam.g(beam.deltaX,beam.deltaY,deltaTheta,beam.crossSection.thickness....
            %    ,beam.length0)*beam.crossSection.E*workspace.EFactor()*beam.crossSection.getI()/beam.length0*workspace.lengthFactor()^3;
            beamGradient=test2(currentDeltaX,currentDeltaY,deltaTheta,beam.crossSection.thickness....
                ,beam.length0)*beam.crossSection.E*workspace.EFactor()*beam.crossSection.getI()/beam.length0*workspace.lengthFactor()^3;
            gradient(index(beam.id+1),1)=beamGradient(1);
            gradient(index(beam.id+2),1)=beamGradient(2);
            gradient(index(beam.id+3),1)=beamGradient(3);
            %update master angle
            if logical(index(beam.id))
                gradient(index(beam.id),1)=-beamGradient(3);
            end
        end
        
        function hessian=getHessian(beam,outputMatrix,index,workspace)
            %get energy hessian
            deltaTheta=beam.getDeltaTheta(outputMatrix,index);
            currentDeltaX=outputMatrix(beam.id+1);
            currentDeltaY=outputMatrix(beam.id+2);
            %beamHessian=beam.h(beam.deltaX,beam.deltaY,deltaTheta,beam.crossSection.thickness....
            %    ,beam.length0)*beam.crossSection.E*workspace.EFactor()*beam.crossSection.getI()/beam.length0*workspace.lengthFactor()^3;
            beamHessian=test3(currentDeltaX,currentDeltaY,deltaTheta,beam.crossSection.thickness....
                ,beam.length0)*beam.crossSection.E*workspace.EFactor()*beam.crossSection.getI()/beam.length0*workspace.lengthFactor()^3;
            i=[];j=[];v=[];
            %x-x
            if logical(index(beam.id+1))
                i(end+1)=index(beam.id+1);j(end+1)=index(beam.id+1);
                v(end+1)=beamHessian(1,1);
            end
            %x-y
            if logical(index(beam.id+1)) && logical(index(beam.id+2))
                i(end+1)=index(beam.id+1);j(end+1)=index(beam.id+2);
                v(end+1)=beamHessian(1,2);
                i(end+1)=index(beam.id+2);j(end+1)=index(beam.id+1);
                v(end+1)=beamHessian(1,2);
            end
            %x-theta
            if logical(index(beam.id+1)) && logical(index(beam.id+3))
                i(end+1)=index(beam.id+1);j(end+1)=index(beam.id+3);
                v(end+1)=beamHessian(1,3);
                i(end+1)=index(beam.id+3);j(end+1)=index(beam.id+1);
                v(end+1)=beamHessian(1,3);
            end
            %y-y
            if logical(index(beam.id+2))
                i(end+1)=index(beam.id+2);j(end+1)=index(beam.id+2);
                v(end+1)=beamHessian(2,2);
            end
            %y-theta
            if logical(index(beam.id+2)) && logical(index(beam.id+3))
                i(end+1)=index(beam.id+2);j(end+1)=index(beam.id+3);
                v(end+1)=beamHessian(2,3);
                i(end+1)=index(beam.id+3);j(end+1)=index(beam.id+2);
                v(end+1)=beamHessian(2,3);
            end
            %theta-theta
            if logical(index(beam.id+3))
                i(end+1)=index(beam.id+3);j(end+1)=index(beam.id+3);
                v(end+1)=beamHessian(3,3);
            end
            %other beam theta dependencies
            if logical(index(beam.id))
                %mastertheta-master theta
                i(end+1)=index(beam.id);j(end+1)=index(beam.id);
                v(end+1)=beamHessian(3,3);
                %x-master theta
                if logical(index(beam.id+1))
                    i(end+1)=index(beam.id);j(end+1)=index(beam.id+1);
                    v(end+1)=-beamHessian(1,3);
                    i(end+1)=index(beam.id+1);j(end+1)=index(beam.id);
                    v(end+1)=-beamHessian(1,3);
                end
                %y-master theta
                if logical(index(beam.id+2))
                    i(end+1)=index(beam.id);j(end+1)=index(beam.id+2);
                    v(end+1)=-beamHessian(2,3);
                    i(end+1)=index(beam.id+2);j(end+1)=index(beam.id);
                    v(end+1)=-beamHessian(2,3);
                end
                %theta-master theta
                if logical(index(beam.id+3))
                    i(end+1)=index(beam.id);j(end+1)=index(beam.id+3);
                    v(end+1)=-beamHessian(3,3);
                    i(end+1)=index(beam.id+3);j(end+1)=index(beam.id);
                    v(end+1)=-beamHessian(3,3);
                end
            end
            hessian=sparse(i,j,v,max(index),max(index));
        end
        
        function [angle,index]=getAngle(beam,~)
            %theta
            angle=beam.theta;
            index=beam.index(5);
        end
        
        %% miscellaneous functions
        
        function [ux,uy,theta,thetaPrime]=getDeformation(beam,workspace)
            %get beam shape
            ux=zeros(beam.noOfData,1);
            uy=zeros(beam.noOfData,1);
            theta=zeros(beam.noOfData,1);
            thetaPrime=zeros(beam.noOfData,1);
            xList=linspace(0,1,beam.noOfData);
            deltaTheta=beam.theta;
            %master angle
            if logical(beam.index(1,1))
                deltaTheta=deltaTheta-beam.masterAngle;
            end
            pLoad=beam.p(beam.deltaX,beam.deltaY,deltaTheta,beam.crossSection.thickness,beam.length0);
            fmLoad=beam.fm(beam.deltaY,deltaTheta,pLoad,beam.length0); 
            f=fmLoad(1)  ;
            m=fmLoad(2) ;
            signP=sign(pLoad);
            pMagnitude=abs(pLoad); 
            if signP > 0
                %positive p
                c=@(x)cosh(x);
                s=@(x)sinh(x);
                t=@(x)tanh(x);
            else
                %negative p
                c=@(x)cos(x);
                s=@(x)sin(x);
                t=@(x)tan(x);
            end
            for i=1:length(xList)
                x=xList(i);
                r=sqrt(pMagnitude);
                rx=r*xList(i);
                if r<= 0.5
                    %very small p load
                    k11=(x^2/2 - x^3/6) +1/120*(20*x^2 - 5*x^4 + x^5)*r^2 +...
                        ((336*x^2 - 70*x^4 + 7*x^6 - x^7)*r^4)/5040+((9792*x^2 - 2016*x^4 + 168*x^6 - 9*x^8 + x^9)*r^6)/362880;
                    k12=x^2/2 + signP*1/24*(-6*x^2 + x^4)*r^2 + 1/720*(75*x^2 - 15*x^4 + x^6)*r^4+signP*((-1708*x^2 + 350*x^4 - 28*x^6 + x^8)*r^6)/40320;
                    k21=(x - x^2/2) + signP*1/24*(-8*x + 4*x^3 - x^4)*r^2 +  1/720*(96*x - 40*x^3 + 6*x^5 - x^6)*r^4+signP*((-2176*x + 896*x^3 - 112*x^5 + 8*x^7 - x^8)*r^6)/40320;
                    k22=x + signP*1/6*(-3*x + x^3)*r^2 + 1/120*(25*x - 10*x^3 + x^5)*r^4+((427*x - 175*x^3 + 21*x^5 - x^7)*r^6)/5040;
                    c11=1/120*(20*x^3 - 15*x^4 + 3*x^5) + signP*((-560*x^3 + 210*x^4 + 168*x^5 -105*x^6 + 15*x^7)*r^2)/5040 + ...
                        ((7616*x^3 - 2016*x^4 - 2688*x^5 + 840*x^6 + 384*x^7 - 189*x^8 + 21*x^9)*r^4)/120960+...
                        signP*((-1309440*x^3 + 269280*x^4 + 502656*x^5 - 110880*x^6 - 84480*x^7 +   20790*x^8 + 7040*x^9 - 2805*x^10 + 255*x^11)*r^6)/39916800;
                    c12=(x^3/6 - x^4/16) + signP*((-200*x^3 + 45*x^4 + 48*x^5 -15*x^6)*r^2)/1440 +...
                        ((6832*x^3 - 1050*x^4 - 2240*x^5 + 420*x^6 +256*x^7 - 63*x^8)*r^4)/80640+...
                        ((332400*x^3 - 38430*x^4 - 122976*x^5 + 15750*x^6 + 19200*x^7 - 2835*x^8 - 1280*x^9 + 255*x^10)*r^6)/7257600;
                    c21=c12;
                    c22=x^3/6 + 1/30*signP*(-5*x^3 + x^5)*r^2 + 1/630*(70*x^3 - 21*x^5 + 2*x^7)*r^4+signP*((-357*x^3 + 126*x^5 - 18*x^7 + x^9)*r^6)/5670;
                    k21Prime=(1 - x) + signP*1/24*(-8 + 12*x^2 - 4*x^3)*r^2 +  1/720*(96- 120*x^2 + 30*x^4 - 6*x^5)*r^4;
                    k22Prime=1 + 1/6*signP*(-3 + 3*x^2)*r^2 + 1/120*(25 - 30*x^2 + 5*x^4)*r^4;
                else 
                    %big p load
                    k11=signP*(t(r)/r^3*(c(rx)-1)-s(rx)/r^3+x/r^2);
                    k12=signP*(c(rx)-1)/(r^2*c(r));
                    k21=(signP*1-signP*c(rx)+t(r)*s(rx))/r^2;
                    k22=s(rx)/(r*c(r));
                    c11=(4*rx+2*rx*c(2*r)-4*s(rx-2*r)-4*s(rx)...
                        -s(2*r-2*rx)-3*s(2*r))/(8*r^5*c(r)^2);
                    c12=(4*c(rx)-2*c(rx)^2 ...
                        +signP*t(r)*(s(2*rx)-2*rx)-2)/(8*r^4*c(r));
                    c21=c12;
                    c22=signP*(s(2*rx)-2*rx)/(8*r^3*c(r)^2);
                    k21Prime=(t(r)*c(r*x)-s(r*x))/r;
                    k22Prime=c(r*x)/c(r);
                end
                %tip
                ux(i)=beam.crossSection.thickness^2*x/(12*(beam.length0)^2)*pLoad-[f,m]*[c11,c12;c21 c22]*[f;m];
                uy(i)=[k11,k12]*[f;m;] ;
                theta(i)=[k21,k22]*[f;m;];
                thetaPrime(i)=[k21Prime,k22Prime]*[f;m;];
            end
        end
        
        function rotationMatrix=getUpdatedRotationMatrix(beam)
            %get the rotation matrix
            currentAngle=beam.angle;
            rotationMatrix=[cos(currentAngle) -sin(currentAngle);sin(currentAngle) cos(currentAngle)];
        end
        
        function [x,y,bendingStress]=getShape(beam,workspace)
            %get beam shape
            [ux,uy,~,thetaPrime]=beam.getDeformation(workspace) ;
            bendingStress=abs(thetaPrime)*beam.crossSection.E*workspace.EFactor()*beam.crossSection.thickness/(2*beam.length0);
            ux=linspace(0,beam.length0,length(ux))'+ux.*beam.length0;
            uy=uy.*beam.length0;
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

