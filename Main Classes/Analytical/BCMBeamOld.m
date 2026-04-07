classdef BCMBeamOld
    properties
        id;
        length;
        angle;
        deltaX;
        deltaY;
        deltaTheta;
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
        theta;
        I;
        energy;
        gradient;
        hessian;
    end
    methods
        
        function beam=BCMBeamOld(angle,length,thickness,width,E)
            %constructor function
            beam.length=length;
            beam.angle=angle;
            beam.thickness=thickness;
            beam.width=width;
            beam.E=E;
            beam.deltaX=0.0;
            beam.deltaY=0.0;
            beam.deltaTheta=0.0;
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
           rotationMatrix=[cos(beam.angle) -sin(beam.angle);sin(beam.angle) cos(beam.angle)];
        end
        
        function rotationMatrix=getRotationGradientMatrix(beam)
           %get the rotation matrix for the gradient 
           rotationMatrix=[-sin(beam.angle) -cos(beam.angle);cos(beam.angle) -sin(beam.angle)];
        end
        
        function rotationMatrix=getRotationHessianMatrix(beam)
           %get the rotation matrix for the hessian 
           rotationMatrix=[-cos(beam.angle) sin(beam.angle);-sin(beam.angle) -cos(beam.angle)];
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
        
        function theta = get.theta(beam)
            %called when you get theta property
            theta=beam.deltaTheta+beam.angle;
        end
        
        function beam = set.theta(beam,~)
            %called when you set theta property            
        end
        
        function I = get.I(beam)
            %called when you get I property
            I=1/12*beam.thickness^3*beam.width;%assume rectangular cross-section
        end
        
        function beam = set.I(beam,~)
            %called when you set I property            
        end
        
        function energy = get.energy(beam)
            %called when you get energy property
            energy=beam.e(beam.deltaX,beam.deltaY,beam.deltaTheta,beam.thickness,beam.length)*beam.E*beam.I/beam.length;
        end
        
        function beam = set.energy(beam,~)
            %called when you set energy property            
        end
        
        function gradient = get.gradient(beam)
            %called when you get gradient property
            gradient=beam.g(beam.deltaX,beam.deltaY,beam.deltaTheta,beam.thickness,beam.length)*beam.E*beam.I/beam.length;
        end
        
        function beam = set.gradient(beam,~)
            %called when you set energy property            
        end
        
        function hessian = get.hessian(beam)
            %called when you get hessian property
            hessian=beam.h(beam.deltaX,beam.deltaY,beam.deltaTheta,beam.thickness,beam.length)*beam.E*beam.I/beam.length;
        end
        
        function beam = set.hessian(beam,~)
            %called when you set energy property            
        end
        
    end
end