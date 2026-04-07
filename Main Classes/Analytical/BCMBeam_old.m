classdef BCMBeam
    properties
        id;
        length;
        transformationAngle;
        masterAngle;
        deltaX;
        deltaY;
        theta;
        %slave-master relations
        masterBCM=0;
        masterPRB=0;
        %cross-section properties
        E;
        thickness;
        width;
        %equations
        e;
        g;
        h;
    end
    properties (Dependent)
        x;
        y;
        angle;
        I;
    end
    methods
        
        function beam=BCMBeam(angle,length,thickness,width,E)
            %constructor function
            beam.length=length;
            beam.transformationAngle=angle;
            beam.masterAngle=0;
            beam.thickness=thickness;
            beam.width=width;
            beam.E=E;
            beam.deltaX=0.0;
            beam.deltaY=0.0;
            beam.theta=0.0;
            beam=beam.loadEquations();
        end
        
        function beam=loadEquations(beam)
            %load the BCM equations
            equations=load('BCMEquations.mat');
            beam.e=equations.e;
            beam.g=equations.g;
            beam.h=equations.h;
        end
        
        function rotationMatrix=getRotationMatrix(beam)
           %get the rotation matrix
           realAngle=beam.transformationAngle+beam.masterAngle;
           rotationMatrix=[cos(realAngle) -sin(realAngle);sin(realAngle) cos(realAngle)];
        end
        
        function rotationMatrix=getRotationGradientMatrix(beam)
           %get the rotation gradient matrix
           realAngle=beam.transformationAngle+beam.masterAngle;
           rotationMatrix=[-sin(realAngle) -cos(realAngle);cos(realAngle) -sin(realAngle)];
        end
        
        function rotationMatrix=getRotationHessianMatrix(beam)
           %get the rotation hessian matrix
           realAngle=beam.transformationAngle+beam.masterAngle;
           rotationMatrix=[-cos(realAngle) sin(realAngle);-sin(realAngle) -cos(realAngle)];
        end
        
        function x = get.x(beam)
            %called when you get x property
            coord=beam.getRotationMatrix()*[beam.deltaX+beam.length;beam.deltaY];
            x=coord(1);
        end
        
        function beam = set.x(beam,~)
            %called when you set x property            
        end
        
        function x = get.y(beam)
            %called when you get y property
            coord=beam.getRotationMatrix()*[beam.deltaX+beam.length;beam.deltaY];
            x=coord(2);
        end
        
        function beam = set.y(beam,~)
            %called when you set y property            
        end
        
        
        function I = get.I(beam)
            %called when you get I property
            I=1/12*beam.thickness^3*beam.width;%assume rectangular cross-section
        end
        
        function beam = set.I(beam,~)
            %called when you set I property            
        end
        
        function angle = get.angle(beam)
            %called when you get angle property
            angle=beam.theta+beam.transformationAngle;
        end
        
        function beam = set.angle(beam,~)
            %called when you set angle property            
        end
        
        function energy = energy(beam,BCMBeams)
            %called when you get energy property
            deltaTheta=beam.getDeltaTheta(BCMBeams);
            energy=beam.e(beam.deltaX,beam.deltaY,deltaTheta,beam.thickness,beam.length)*beam.E*beam.I/beam.length;
        end
                
        function gradient = gradient(beam,inputIndex,BCMBeams)
            %gredient of the energy
            gradient=zeros(max(inputIndex),1);
            deltaTheta=beam.getDeltaTheta(BCMBeams);
            beamGradient=beam.g(beam.deltaX,beam.deltaY,deltaTheta,beam.thickness,beam.length)*beam.E*beam.I/beam.length;
            gradient(inputIndex(beam.id),1)=beamGradient(1);
            gradient(inputIndex(beam.id+1),1)=beamGradient(2);
            gradient(inputIndex(beam.id+2),1)=beamGradient(3);
            %update previous ones
            if logical(beam.masterBCM)
               gradient(inputIndex(BCMBeams{beam.masterBCM}.id+2),1)=-beamGradient(3);
            end
        end
       
        
        function gradientX = gradientConstraintX(beam,inputIndex,BCMBeams,modifier)
            %kinematic gradient in x direction
            gradientX=zeros(max(inputIndex),1);
            transformation=beam.getRotationMatrix();
            beamGradient=[transformation(1,1:2)';0];
            transformation=beam.getRotationGradientMatrix();
            beamTransGradient=transformation(1,:)*[beam.deltaX+beam.length;beam.deltaY];
            %x
            gradientX(inputIndex(beam.id),1)=beamGradient(1);
            %y
            gradientX(inputIndex(beam.id+1),1)=beamGradient(2);
            %theta
            gradientX(inputIndex(beam.id+2),1)=beamGradient(3);
            if logical(inputIndex(beam.id+3))
                %gradient wrt to transition angle
                gradientX(inputIndex(beam.id+3),1)=beamTransGradient;
            end
            %check if there is master
            if logical(beam.masterBCM)
                currentBeam=BCMBeams{beam.masterBCM};
                %theta
                gradientX(inputIndex(currentBeam.id+2),1)=beamTransGradient;
            end
            gradientX=gradientX*modifier;
        end
        
        function gradientY = gradientConstraintY(beam,inputIndex,BCMBeams,modifier)
            %kinematic gradient in y direction
            gradientY=zeros(max(inputIndex),1);
            transformation=beam.getRotationMatrix();
            beamGradient=[transformation(2,1:2)';0];
            transformation=beam.getRotationGradientMatrix();
            beamTransGradient=transformation(2,:)*[beam.deltaX+beam.length;beam.deltaY];
            %x
            gradientY(inputIndex(beam.id),1)=beamGradient(1);
            %y
            gradientY(inputIndex(beam.id+1),1)=beamGradient(2);
            %theta
            gradientY(inputIndex(beam.id+2),1)=beamGradient(3);
            if logical(inputIndex(beam.id+3))
                %gradient wrt to transition angle
                gradientY(inputIndex(beam.id+3),1)=beamTransGradient;
            end
            %check if there is master
            if logical(beam.masterBCM)
                currentBeam=BCMBeams{beam.masterBCM};
                %theta
                gradientY(inputIndex(currentBeam.id+2),1)=beamTransGradient;
            end
            gradientY=gradientY*modifier;
        end
        
        function gradientTheta = gradientConstraintTheta(beam,inputIndex,modifier)
            %kinematic direction of theta
            gradientTheta=zeros(max(inputIndex),1);
            %theta
            gradientTheta(inputIndex(beam.id+2),1)=1;
            if logical(inputIndex(beam.id+3))
                %gradient wrt to transition angle
                gradientTheta(inputIndex(beam.id+3),1)=1;
            end
            gradientTheta=gradientTheta*modifier;
        end
        
        function hessian = hessian(beam,inputIndex,BCMBeams)
            %hessian matrix for the energy
            hessian=zeros(max(inputIndex),max(inputIndex));
            deltaTheta=beam.getDeltaTheta(BCMBeams);
            beamHessian=beam.h(beam.deltaX,beam.deltaY,deltaTheta,beam.thickness,beam.length)*beam.E*beam.I/beam.length;
            %x-x
            hessian(inputIndex(beam.id),inputIndex(beam.id))=beamHessian(1,1);
            %x-y
            hessian(inputIndex(beam.id+1),inputIndex(beam.id))=beamHessian(1,2);
            hessian(inputIndex(beam.id),inputIndex(beam.id+1))=beamHessian(1,2);
            %x-theta
            hessian(inputIndex(beam.id+2),inputIndex(beam.id))=beamHessian(1,3);
            hessian(inputIndex(beam.id),inputIndex(beam.id+2))=beamHessian(1,3);
            %y-y
            hessian(inputIndex(beam.id+1),inputIndex(beam.id+1))=beamHessian(2,2);
            %y-theta
            hessian(inputIndex(beam.id+2),inputIndex(beam.id+1))=beamHessian(2,3);
            hessian(inputIndex(beam.id+1),inputIndex(beam.id+2))=beamHessian(2,3);
            %theta-theta
            hessian(inputIndex(beam.id+2),inputIndex(beam.id+2))=beamHessian(3,3);
            %other beam theta dependencies
            if logical(beam.masterBCM)
                currentBeam=BCMBeams{beam.masterBCM};
                %mastertheta-master theta
                hessian(inputIndex(currentBeam.id+2),inputIndex(currentBeam.id+2))=beamHessian(3,3);
                %theta-master theta
                hessian(inputIndex(beam.id+2),inputIndex(currentBeam.id+2))=-beamHessian(3,3);
                hessian(inputIndex(currentBeam.id+2),inputIndex(beam.id+2))=-beamHessian(3,3);
                %x-master theta
                hessian(inputIndex(currentBeam.id+2),inputIndex(beam.id))=-beamHessian(1,3);
                hessian(inputIndex(beam.id),inputIndex(currentBeam.id+2))=-beamHessian(1,3);
                %y-master theta
                hessian(inputIndex(currentBeam.id+2),inputIndex(beam.id+1))=-beamHessian(2,3);
                hessian(inputIndex(beam.id+1),inputIndex(currentBeam.id+2))=-beamHessian(2,3);
            end
        end
        
        function hessian = hessianX(beam,inputIndex,BCMBeams,modifier)
            %kinematic hessian in x direction
            hessian=zeros(max(inputIndex),max(inputIndex));
            transformationGradient=beam.getRotationGradientMatrix();
            transformationHessian=beam.getRotationHessianMatrix();
            %check if there is master
            if logical(beam.masterBCM)
                masterBeam=BCMBeams{beam.masterBCM};
                %master theta-master theta
                hessian(inputIndex(masterBeam.id+2),inputIndex(masterBeam.id+2))=transformationHessian(1,:)*[beam.deltaX+beam.length;beam.deltaY];
                %x-master theta
                hessian(inputIndex(beam.id),inputIndex(masterBeam.id+2))=transformationGradient(1,1);
                hessian(inputIndex(masterBeam.id+2),inputIndex(beam.id))=transformationGradient(1,1);
                %y-master theta
                hessian(inputIndex(beam.id+1),inputIndex(masterBeam.id+2))=transformationGradient(1,2);
                hessian(inputIndex(masterBeam.id+2),inputIndex(beam.id+1))=transformationGradient(1,2);
            end
            
            %transformation-theta
            if logical(inputIndex(beam.id+3))
                %transformation-transformation
                hessian(inputIndex(beam.id+3),inputIndex(beam.id+3))=transformationHessian(1,:)*[beam.deltaX+beam.length;beam.deltaY];
                %x-transformation
                hessian(inputIndex(beam.id),inputIndex(beam.id+3))=transformationGradient(1,1);
                hessian(inputIndex(beam.id+3),inputIndex(beam.id))=transformationGradient(1,1);
                %y-transformation
                hessian(inputIndex(beam.id+1),inputIndex(beam.id+3))=transformationGradient(1,2);
                hessian(inputIndex(beam.id+3),inputIndex(beam.id+1))=transformationGradient(1,2);
                %check if there is master
                if logical(beam.masterBCM)
                    masterBeam=BCMBeams{beam.masterBCM};
                    %master theta-transformation
                    hessian(inputIndex(masterBeam.id+2),inputIndex(beam.id+3))=transformationHessian(1,:)*[beam.deltaX+beam.length;beam.deltaY];
                    hessian(inputIndex(masterBeam.id+3),inputIndex(beam.id+2))=transformationHessian(1,:)*[beam.deltaX+beam.length;beam.deltaY];
                end
            end
            hessian=hessian*modifier;
        end
        
        function hessian = hessianY(beam,inputIndex,BCMBeams,modifier)
            %kinematic hessian in y direction
            hessian=zeros(max(inputIndex),max(inputIndex));
            transformationGradient=beam.getRotationGradientMatrix();
            transformationHessian=beam.getRotationHessianMatrix();
            %check if there is master
            if logical(beam.masterBCM)
                masterBeam=BCMBeams{beam.masterBCM};
                %master theta-master theta
                hessian(inputIndex(masterBeam.id+2),inputIndex(masterBeam.id+2))=transformationHessian(2,:)*[beam.deltaX+beam.length;beam.deltaY];
                %x-master theta
                hessian(inputIndex(beam.id),inputIndex(masterBeam.id+2))=transformationGradient(2,1);
                hessian(inputIndex(masterBeam.id+2),inputIndex(beam.id))=transformationGradient(2,1);
                %y-master theta
                hessian(inputIndex(beam.id+1),inputIndex(masterBeam.id+2))=transformationGradient(2,2);
                hessian(inputIndex(masterBeam.id+2),inputIndex(beam.id+1))=transformationGradient(2,2);
            end
            
            %transformation-theta
            if logical(inputIndex(beam.id+3))
                %transformation-transformation
                hessian(inputIndex(beam.id+3),inputIndex(beam.id+3))=transformationHessian(2,:)*[beam.deltaX+beam.length;beam.deltaY];
                %x-transformation
                hessian(inputIndex(beam.id),inputIndex(beam.id+3))=transformationGradient(2,1);
                hessian(inputIndex(beam.id+3),inputIndex(beam.id))=transformationGradient(2,1);
                %y-transformation
                hessian(inputIndex(beam.id+1),inputIndex(beam.id+3))=transformationGradient(2,2);
                hessian(inputIndex(beam.id+3),inputIndex(beam.id+1))=transformationGradient(2,2);
                %check if there is master
                if logical(beam.masterBCM)
                    masterBeam=BCMBeams{beam.masterBCM};
                    %master theta-transformation
                    hessian(inputIndex(masterBeam.id+2),inputIndex(beam.id+3))=transformationHessian(2,:)*[beam.deltaX+beam.length;beam.deltaY];
                    hessian(inputIndex(masterBeam.id+3),inputIndex(beam.id+2))=transformationHessian(2,:)*[beam.deltaX+beam.length;beam.deltaY];
                end
            end
            hessian=hessian*modifier;
        end
        
        function hessian = hessianTheta(beam,inputIndex,modifier)
            %kinematic direction of theta
            hessian=zeros(max(inputIndex),max(inputIndex));
            %theta-theta
            hessian(inputIndex(beam.id+2),inputIndex(beam.id+2))=1;
            if logical(inputIndex(beam.id+3))
                %hessian wrt to transition angle
                hessian(inputIndex(beam.id+3),inputIndex(beam.id+3))=0;
                %theta-trans
                hessian(inputIndex(beam.id+2),inputIndex(beam.id+3))=1;
                hessian(inputIndex(beam.id+3),inputIndex(beam.id+2))=1;
            end
            hessian=hessian*modifier;
        end
        
        function deltaTheta=getDeltaTheta(beam,BCMBeams)
            %get the delta theta from the list of the bcm beams
            deltaTheta=beam.theta;
            if logical(beam.masterBCM)
                deltaTheta=deltaTheta-BCMBeams{beam.masterBCM}.theta;
            end
        end
        
    end
end