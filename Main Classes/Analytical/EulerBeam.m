classdef EulerBeam
    
    properties
        lengthBeam;%length of the beam
        E;%Youngs modulus
        thickness;%in plane
        width;%out plane
        Fx;%end load x direction
        Fy;%end load y direction
        M;%end moment
        theta;%angle list
        curvature;%curvature list (thetaPrime)
        rotation;
    end
    
    properties (Dependent)
        A;%cross section area
        I;%cross section moment of inertia
        x;%shape of the beam - x coordinates
        y;%shape of the beam - y coordinates
        internalMoment;%internal moment list
        internalStress;%internal stress list
        theta0;%end angle
    end
    
    methods
        function beam=EulerBeam(length,E,thickness,width,Fx,Fy,M)
            %constructor for the EulerBeam
            beam.lengthBeam=length;
            beam.E=E;
            beam.thickness=thickness;
            beam.width=width;
            beam.Fx=Fx;
            beam.Fy=Fy;
            beam.M=M;
        end
        
        function beam=updateLoading(beam,Fx,Fy,M)
            %update the tip loading
            beam.Fx=Fx;
            beam.Fy=Fy;
            beam.M=M;
        end
        
        function A = get.A(beam)
            %called when you get A property
            A=beam.thickness*beam.width;%assume rectangular cross-section
        end
        
        function beam = set.A(beam,~)
            %called when you set A property           
        end
        
        function I = get.I(beam)
            %called when you get I property
            I=1/12*beam.thickness^3*beam.width;%assume rectangular cross-section
        end
        
        function beam = set.I(beam,~)
            %called when you set I property            
        end
        
        function x = get.x(beam)
            %called when you get x property
            nPoints=length(beam.theta);
            x=zeros(1,nPoints);
            ds=beam.lengthBeam/(nPoints-1);
            %integrate with trapozoidal rule
            for i=2:nPoints
                x(i)=x(i-1)+ds*(cos(beam.theta(i-1))+cos(beam.theta(i)))/2;
            end
        end
        
        function beam = set.x(beam,~)
            %called when you set x property            
        end
        
        function y = get.y(beam)
            %called when you get y property
            nPoints=length(beam.theta);
            y=zeros(1,nPoints);
            ds=beam.lengthBeam/(nPoints-1);
            %integrate with trapozoidal rule
            for i=2:nPoints
                y(i)=y(i-1)+ds*(sin(beam.theta(i-1))+sin(beam.theta(i)))/2;
            end
        end
        
        function beam = set.y(beam,~)
            %called when you set y property          
        end
              
        function internalMoment = get.internalMoment(beam)
            %called when you get internalMoment property
            internalMoment=beam.E*beam.I*beam.curvature;
        end
        
        function beam = set.internalMoment(beam,~)
            %called when you set internalMoment property
        end
        
        function internalStress = get.internalStress(beam)
            %called when you get internalStress property
            %assume rectangular cross section
            %maximum bending stress at the cross-section M.c/I
            internalStress=abs(beam.E*beam.thickness/2*beam.curvature);
        end
        
        function beam = set.internalStress(beam,~)
            %called when you set internalStress property
        end
        
        function beam= analytical(beam)
            %solve for a single beam -- Euler-Beurnoulli Equation
            %initial mesh
            solinit = bvpinit(linspace(0,beam.lengthBeam,100),[0 0]);
            %euler function
            func=@(s,y)analyticalFunction(beam,s,y);
            %euler boundary function
            boundary=@(ya,yb)analyticalFunctionBoundary(beam,ya,yb);
            %solve
            nPoints=100;
            if abs(beam.Fx)<1e-5 && abs(beam.Fy)<1e-5 && abs(beam.M)<1e-5
                beam.theta=zeros(1,nPoints);
                beam.curvature=zeros(1,nPoints);  
            else
                options = bvpset('Stats','off', 'FJacobian', @(s,y)beam.jacobian(s,y),'BCJacobian',@(ya,yb)beam.boundaryJacobian(ya,yb),'NMax',1000);
                sol = bvp4c(func,boundary,solinit,options); 
                %solution at finer mesh
                xx = linspace(0,beam.lengthBeam,nPoints);
                yy = deval(sol,xx);
                beam.theta=yy(1,:);
                %K(curvature)=theta'
                beam.curvature=yy(2,:);           
            end
        end
        
        function dydx = analyticalFunction(beam,s,y )
            dydx=zeros(2,1);
            %y(1) = theta, y(2)= theta'
            dydx(1)=y(2);%dtheta=theta'
            dydx(2)=(beam.Fx*sin(y(1))-beam.Fy*cos(y(1)))/(beam.E*beam.I);
        end
        
        function res = analyticalFunctionBoundary(beam, ya,yb )
            res=[ya(1);...%theta(0)=0
                yb(2)-beam.M/(beam.E*beam.I);
                ];
        end
        
        function dfdy = jacobian(beam,s,y)
            dfdy=[0 1;(beam.Fx*cos(y(1))+beam.Fy*sin(y(1)))/(beam.E*beam.I) 0];
        end
        
        function  [dbcdya,dbcdyb] = boundaryJacobian(beam,ya,yb)
            dbcdya=[1 0;0 0];
            dbcdyb=[0 0;0 1];
        end
        
        function strainEnergy = energy(beam)
            s=linspace(0,beam.lengthBeam,length(beam.curvature));
            strainEnergy=0;
            for i=2:length(s)
                strainEnergy=strainEnergy+0.5*abs(s(i)-s(i-1))*(beam.curvature(i)^2+beam.curvature(i-1)^2);
            end
            strainEnergy=strainEnergy/2*beam.E*beam.I;
        end
        
    end
end