classdef testOld
    properties
        r1;
        theta1;
        r2;
        theta2;
        beam;
        bcmBeam;
        beamAngle;
        r3;
        theta3;
        N;
        load;
    end
    methods
        function t=testOld(type)
            t.r1=30e-3;
            t.theta1=90*pi/180;
            t.r2=45e-3;
            t.theta2=20*pi/180;
            t.r3=42.2862e-3;
            t.theta3=0;
            t.load=1;
            switch type
                case 1
                    t.beam=Beam(69e9,45.3909e-3,0.5e-3,0.5e-3,0.0,0.0,0.0);
                    t.beamAngle=pi/2;
                case 2
                    t.bcmBeam=BCMBeamOld(90*pi/180,45.3909e-3,0.5e-3,0.5e-3,69e9);
                case 3
                    t.N=2;
                    for i=1:t.N                       
                        t.bcmBeam{i}=BCMBeamOld(90*pi/180,45.3909e-3/t.N,0.5e-3,0.5e-3,69e9);
                    end
            end
            t=t.solve(type);
        end
        
        function t=solve(t,type)
            switch type
                case 1
                    fMincon=@(x)t.fMinConFunc( x);
                    gMinCon=@(x)t.fMinConCostraint(x);
                    opt=optimoptions(@fmincon,'Display','iter-detailed','Algorithm','interior-point','MaxFunEvals',5e4,'MaxIter',5e4,'TolCon',1e-6,'TolFun',1e-10,'UseParallel',false,...
                        'SpecifyConstraintGradient',false,'SpecifyObjectiveGradient',false,'CheckGradients',false);%
                    initial=zeros(1,4);
                case 2
                    opt=optimoptions(@fmincon,'Display','iter-detailed','Algorithm','interior-point','MaxFunEvals',5e4,'MaxIter',5e4,'TolCon',1e-6,'TolFun',1e-10,'UseParallel',false,...
                        'SpecifyConstraintGradient',true,'SpecifyObjectiveGradient',true,'CheckGradients',false,'FiniteDifferenceType','central','HessianFcn',@(x,lambda)t.hessianfcn(x,lambda));%
                    fMincon=@(x)t.fMinConFunc2( x);
                    gMinCon=@(x)t.fMinConCostraint2(x);
                    initial=zeros(1,4);
                case 3
                    opt=optimoptions(@fmincon,'Display','iter-detailed','Algorithm','interior-point','MaxFunEvals',5e4,'MaxIter',5e4,'TolCon',1e-6,'TolFun',1e-10,'UseParallel',false,...
                        'SpecifyConstraintGradient',true,'SpecifyObjectiveGradient',true,'HessianFcn',@(x,lambda)t.hessianfcn2(x,lambda));%
                    fMincon=@(x)t.fMinConFunc3( x);
                    gMinCon=@(x)t.fMinConCostraint3(x);
                    initial=zeros(1,1+3*t.N);
            end
            initial(1)=t.theta1;
            tic
            [newState,fval,exitflag,~,~,~,hessian] = fmincon(fMincon,initial,[],[],[],[],[],[],gMinCon,opt);
            newState
            toc
        end
        
        function  [f,g]= fMinConFunc(t,x)
            fx=x(2);
            fy=x(3); 
            m=x(4);
            t.beam.euler=t.beam.euler.updateLoading(fx,fy,m);
            t.beam=t.beam.analytical();
            coord=[cos(t.beamAngle) -sin(t.beamAngle);sin(t.beamAngle) cos(t.beamAngle)]*[t.beam.euler.x(end);t.beam.euler.y(end)];
            f=t.beam.energy()-1*coord(1);
            f=f*1000;
            g=[0;(t.beam.euler.x(end)-45.3909e-3)/2;(t.beam.euler.y(end))/2;t.beam.euler.theta(end)/2]*1000;
        end
        
        function [ c,ceq ] = fMinConCostraint(t,x)
            fx=x(2);
            fy=x(3);
            m=x(4);
            t.theta1=x(1);
            t.beam.euler=t.beam.euler.updateLoading(fx,fy,m);
            t.beam=t.beam.analytical();
            coord=[cos(t.beamAngle) -sin(t.beamAngle);sin(t.beamAngle) cos(t.beamAngle)]*[t.beam.euler.x(end);t.beam.euler.y(end)]; 
            endAngle=t.beam.euler.theta(end)+t.beamAngle;
            t.theta2=endAngle-70*pi/180;
            c=[];
            ceq(1)=t.r1*cos(t.theta1)+t.r2*cos(t.theta2)-coord(1)-t.r3; 
            ceq(2)=t.r1*sin(t.theta1)+t.r2*sin(t.theta2)-coord(2); 
        end
        
        function  [f,g]= fMinConFunc2(test,u)
            test.bcmBeam.deltaX=u(2);
            test.bcmBeam.deltaY=u(3);
            test.bcmBeam.deltaTheta=u(4);
            load=1;
            f=test.bcmBeam.energy-load*test.bcmBeam.x;
            %gradient
            g=[0;test.bcmBeam.gradient]-[0;cos(test.bcmBeam.angle)*load;-sin(test.bcmBeam.angle)*load;0];
        end
        
        function [ c,ceq,gradc,gradceq  ] = fMinConCostraint2(t,u)
            t.theta1=u(1);
            t.bcmBeam.deltaX=u(2);
            t.bcmBeam.deltaY=u(3);
            t.bcmBeam.deltaTheta=u(4);
            t.theta2=t.bcmBeam.theta-70*pi/180;
            c=[];
            ceq(1)=t.r1*cos(t.theta1)+t.r2*cos(t.theta2)-t.bcmBeam.x-t.r3; 
            ceq(2)=t.r1*sin(t.theta1)+t.r2*sin(t.theta2)-t.bcmBeam.y; 
            if nargout > 2
                gradc = [];
                gradceq(:,1)=[-t.r1*sin(t.theta1);-cos(t.bcmBeam.angle);+sin(t.bcmBeam.angle);-t.r2*sin(t.theta2)] ;
                gradceq(:,2)=[t.r1*cos(t.theta1);-sin(t.bcmBeam.angle);-cos(t.bcmBeam.angle);t.r2*cos(t.theta2)] ;
            end
        end
        
        function Hout = hessianfcn(test,u,lambda)
            test.theta1=u(1);
            test.bcmBeam.deltaX=u(2);
            test.bcmBeam.deltaY=u(3);
            test.bcmBeam.deltaTheta=u(4);
            test.theta2=test.bcmBeam.theta-70*pi/180;
            Hout=test.bcmBeam.hessian;
            Hout=[zeros(1,3);Hout];
            Hout=[zeros(4,1),Hout];
            Hout=Hout+lambda.eqnonlin(1)*[-test.r1*cos(test.theta1) 0 0 0;0 0 0 0; 0 0 0 0;0 0 0 -test.r2*cos(test.theta2)]...
                +lambda.eqnonlin(2)*[-test.r1*sin(test.theta1) 0 0 0;0 0 0 0; 0 0 0 0;0 0 0 -test.r2*sin(test.theta2)];
        end
        
        function  [f,g]= fMinConFunc3(test,u)
            f=0;
            x=0;
            alpha=test.bcmBeam{1}.angle;
            g=zeros(1+test.N*3,1);
            for i=1:test.N
                test.bcmBeam{i}.deltaX=u(3*(i-1)+2);
                test.bcmBeam{i}.deltaY=u(3*(i-1)+3);
                test.bcmBeam{i}.deltaTheta=u(3*(i-1)+4);
                test.bcmBeam{i}.angle=alpha;
                f=f+test.bcmBeam{i}.energy;
                x=x+test.bcmBeam{i}.x;
                alpha=test.bcmBeam{i}.angle+test.bcmBeam{i}.deltaTheta;
                g(3*(i-1)+2:3*(i-1)+4)=g(3*(i-1)+2:3*(i-1)+4)+test.bcmBeam{i}.gradient;
                rotation=test.bcmBeam{i}.getRotationMatrix();
                g(3*(i-1)+2)=g(3*(i-1)+2)-rotation(1,:)*[1;0;]*test.load;
                g(3*(i-1)+3)=g(3*(i-1)+3)-rotation(1,:)*[0;1;]*test.load;
                if i ~=1
                    rotation=test.bcmBeam{i}.getRotationGradientMatrix();
                    for j=i-1:-1:1
                        g(3*(j-1)+4)=g(3*(j-1)+4)-rotation(1,:)*[test.bcmBeam{i}.deltaX+test.bcmBeam{i}.length;test.bcmBeam{i}.deltaY;]*test.load;
                    end
                end
            end
            f=f-test.load*x;
        end
        
        function [ c,ceq,gradc,gradceq  ] = fMinConCostraint3(t,u)
            t.theta1=u(1);
            x=0;
            y=0;
            alpha=t.bcmBeam{1}.angle;
            gradc = [];
            gradceq=zeros(1+t.N*3,2);
            gradceq(1,1)=-t.r1*sin(t.theta1);
            gradceq(1,2)=t.r1*cos(t.theta1);
            for i=1:t.N
                t.bcmBeam{i}.deltaX=u(3*(i-1)+2);
                t.bcmBeam{i}.deltaY=u(3*(i-1)+3);
                t.bcmBeam{i}.deltaTheta=u(3*(i-1)+4);
                t.bcmBeam{i}.angle=alpha;
                x=x+t.bcmBeam{i}.x;
                y=y+t.bcmBeam{i}.y;
                alpha=t.bcmBeam{i}.angle+t.bcmBeam{i}.deltaTheta;
                rotation=t.bcmBeam{i}.getRotationMatrix();
                gradceq(3*(i-1)+2,1)=gradceq(3*(i-1)+2,1)-rotation(1,:)*[1;0;];
                gradceq(3*(i-1)+2,2)=gradceq(3*(i-1)+2,2)-rotation(2,:)*[1;0;];
                gradceq(3*(i-1)+3,1)=gradceq(3*(i-1)+3,1)-rotation(1,:)*[0;1;];
                gradceq(3*(i-1)+3,2)=gradceq(3*(i-1)+3,2)-rotation(2,:)*[0;1;];
                if i ~=1
                    rotation=t.bcmBeam{i}.getRotationGradientMatrix();
                    for j=i-1:-1:1
                        gradceq(3*(j-1)+4,1)=gradceq(3*(j-1)+4,1)-rotation(1,:)*[t.bcmBeam{i}.deltaX+t.bcmBeam{i}.length;t.bcmBeam{i}.deltaY;];
                        gradceq(3*(j-1)+4,2)=gradceq(3*(j-1)+4,2)-rotation(2,:)*[t.bcmBeam{i}.deltaX+t.bcmBeam{i}.length;t.bcmBeam{i}.deltaY;];
                    end
                end
            end
            t.theta2=t.bcmBeam{end}.theta-70*pi/180;
            for j=1:t.N
                gradceq(3*(j-1)+4,1)=gradceq(3*(j-1)+4,1)-t.r2*sin(t.theta2);
                gradceq(3*(j-1)+4,2)=gradceq(3*(j-1)+4,2)+t.r2*cos(t.theta2);
            end
            c=[];
            ceq(1)=t.r1*cos(t.theta1)+t.r2*cos(t.theta2)-x-t.r3; 
            ceq(2)=t.r1*sin(t.theta1)+t.r2*sin(t.theta2)-y; 
        end
        
        function Hout = hessianfcn2(test,u,lambda)
            test.theta1=u(1);
            Hout=zeros(1+test.N*3,1+test.N*3);
            alpha=test.bcmBeam{1}.angle;
            for i=1:test.N
                test.bcmBeam{i}.deltaX=u(3*(i-1)+2);
                test.bcmBeam{i}.deltaY=u(3*(i-1)+3);
                test.bcmBeam{i}.deltaTheta=u(3*(i-1)+4);
                test.bcmBeam{i}.angle=alpha;
                alpha=test.bcmBeam{i}.angle+test.bcmBeam{i}.deltaTheta;
                hessian=test.bcmBeam{i}.hessian;
                Hout(3*(i-1)+2,3*(i-1)+2)=hessian(1,1);
                Hout(3*(i-1)+3,3*(i-1)+3)=hessian(2,2);
                Hout(3*(i-1)+4,3*(i-1)+4)=hessian(3,3);
                Hout(3*(i-1)+2,3*(i-1)+3)=hessian(1,2);Hout(3*(i-1)+3,3*(i-1)+2)=hessian(1,2);
                Hout(3*(i-1)+2,3*(i-1)+4)=hessian(1,3);Hout(3*(i-1)+4,3*(i-1)+2)=hessian(1,3);
                Hout(3*(i-1)+3,3*(i-1)+4)=hessian(2,3);Hout(3*(i-1)+4,3*(i-1)+3)=hessian(2,3);
                if i ~=1
                    rotation=test.bcmBeam{i}.getRotationHessianMatrix();
                    for j=i-1:-1:1
                        Hout(3*(j-1)+4,3*(j-1)+4)=Hout(3*(j-1)+4,3*(j-1)+4)-rotation(1,:)*[test.bcmBeam{i}.deltaX+test.bcmBeam{i}.length;test.bcmBeam{i}.deltaY;]*test.load;
                        Hout(3*(i-1)+4,3*(j-1)+4)=Hout(3*(i-1)+4,3*(j-1)+4)-rotation(1,:)*[test.bcmBeam{i}.deltaX+test.bcmBeam{i}.length;test.bcmBeam{i}.deltaY;]*test.load;
                        Hout(3*(j-1)+4,3*(i-1)+4)=Hout(3*(j-1)+4,3*(i-1)+4)-rotation(1,:)*[test.bcmBeam{i}.deltaX+test.bcmBeam{i}.length;test.bcmBeam{i}.deltaY;]*test.load;
                    end
                end
            end
            Hout(1,1)=Hout(1,1)+lambda.eqnonlin(1)*-test.r1*cos(test.theta1) +lambda.eqnonlin(2)*-test.r1*sin(test.theta1); 
            Hout(end,end)=Hout(end,end)+lambda.eqnonlin(1)*-test.r2*cos(test.theta2) +lambda.eqnonlin(2)*-test.r2*sin(test.theta2); 
        end
    end
end