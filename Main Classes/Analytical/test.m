classdef test
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
        inputMatrix;
    end
    methods
        function t=test(type)
            t.r1=30e-3;
            t.theta1=90*pi/180;
            t.r2=45e-3;
            t.theta2=20*pi/180;
            t.r3=42.2862e-3;
            t.theta3=0;
            t.load=10;
            switch type
                case 1
                    t.beam=Beam(69e9,45.3909e-3,0.5e-3,0.5e-3,0.0,0.0,0.0);
                    t.beamAngle=90*pi/180;
                case 2
                    t.bcmBeam=BCMBeam(90*pi/180,45.3909e-3,0.5e-3,0.5e-3,69e9);
                    t.bcmBeam.id=2;
                    t.inputMatrix=[1;2;3;4;0];
                case 3
                    t.N=6;
                    t.inputMatrix=zeros(1+4*t.N,1);
                    modifier=0;
                    for i=1:t.N                       
                        t.bcmBeam{i}=BCMBeam(90*pi/180,45.3909e-3/t.N,0.5e-3,0.5e-3,69e9);
                        if i~=1
                            t.bcmBeam{i}.masterBCM=i-1;
                        end
                        t.bcmBeam{i}.id=(i-1)*4+2;
                        t.inputMatrix(t.bcmBeam{i}.id:t.bcmBeam{i}.id+2,1)=linspace(t.bcmBeam{i}.id,t.bcmBeam{i}.id+2,3)-modifier;
                        t.inputMatrix(t.bcmBeam{i}.id+3,1)=0;
                        modifier=modifier+1;
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
                        'SpecifyConstraintGradient',true,'SpecifyObjectiveGradient',true,'CheckGradients',false,'HessianFcn',@(x,lambda)t.hessianfcn(x,lambda));%
                    fMincon=@(x)t.fMinConFunc2( x);
                    gMinCon=@(x)t.fMinConCostraint2(x);
                    initial=zeros(1,4);
                case 3
                    opt=optimoptions(@fmincon,'Display','iter-detailed','Algorithm','interior-point','MaxFunEvals',5e4,'MaxIter',5e4,'TolCon',1e-6,'TolFun',1e-10,'UseParallel',false,...
                        'SpecifyConstraintGradient',true,'SpecifyObjectiveGradient',true,'CheckGradients',false,'HessianFcn',@(x,lambda)t.hessianfcn2(x,lambda));%
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
            test.bcmBeam.theta=u(4);
            f=test.bcmBeam.energy(test.bcmBeam)-test.load*test.bcmBeam.x;
            f=f*1000;
            g=test.bcmBeam.gradient(test.inputMatrix,test.bcmBeam)-test.bcmBeam.gradientConstraintX(test.inputMatrix,test.bcmBeam,test.load);
            g=g*1000;
        end
        
        function [ c,ceq,gradc,gradceq  ] = fMinConCostraint2(test,u)
            test.theta1=u(1);
            test.bcmBeam.deltaX=u(2);
            test.bcmBeam.deltaY=u(3);
            test.bcmBeam.theta=u(4);
            test.theta2=test.bcmBeam.angle-70*pi/180;
            c=[];
            ceq(1)=test.r1*cos(test.theta1)+test.r2*cos(test.theta2)-test.bcmBeam.x-test.r3; 
            ceq(2)=test.r1*sin(test.theta1)+test.r2*sin(test.theta2)-test.bcmBeam.y; 
            if nargout > 2
                gradc = [];
                gradceq(:,1)=[-test.r1*sin(test.theta1);0;0;0]-test.bcmBeam.gradientConstraintX(test.inputMatrix,test.bcmBeam,1.0)+test.bcmBeam.gradientConstraintTheta(test.inputMatrix,-test.r2*sin(test.theta2)) ;
                gradceq(:,2)=[test.r1*cos(test.theta1);0;0;0]-test.bcmBeam.gradientConstraintY(test.inputMatrix,test.bcmBeam,1.0)+test.bcmBeam.gradientConstraintTheta(test.inputMatrix,test.r2*cos(test.theta2)) ;
            end
        end
        
        function Hout = hessianfcn(test,u,lambda)
            test.theta1=u(1);
            test.bcmBeam.deltaX=u(2);
            test.bcmBeam.deltaY=u(3);
            test.bcmBeam.theta=u(4);
            test.theta2=test.bcmBeam.theta-70*pi/180;
            Hout=test.bcmBeam.hessian(test.inputMatrix,test.bcmBeam)*1000+...
                lambda.eqnonlin(1)*[-test.r1*cos(test.theta1) 0 0 0;0 0 0 0; 0 0 0 0;0 0 0 0]...
                +lambda.eqnonlin(2)*[-test.r1*sin(test.theta1) 0 0 0;0 0 0 0; 0 0 0 0;0 0 0 0]...
                +test.bcmBeam.hessianX(test.inputMatrix,test.bcmBeam,-lambda.eqnonlin(1))...
                 +test.bcmBeam.hessianY(test.inputMatrix,test.bcmBeam,-lambda.eqnonlin(2))...
                 +test.bcmBeam.hessianTheta(test.inputMatrix,-test.r2*cos(test.theta2)*lambda.eqnonlin(1))...
                 +test.bcmBeam.hessianTheta(test.inputMatrix,-test.r2*sin(test.theta2)*lambda.eqnonlin(2));
        end
        
        function  [f,g]= fMinConFunc3(test,u)
            f=0;
            x=0;
            g=zeros(1+test.N*3,1);
            for i=1:test.N
                test.bcmBeam{i}.deltaX=u(3*(i-1)+2);
                test.bcmBeam{i}.deltaY=u(3*(i-1)+3);
                test.bcmBeam{i}.theta=u(3*(i-1)+4);
                if i~=1
                    test.bcmBeam{i}.masterAngle=test.bcmBeam{i-1}.theta;
                end
                f=f+test.bcmBeam{i}.energy(test.bcmBeam);
                x=x+test.bcmBeam{i}.x;
                g=g+test.bcmBeam{i}.gradient(test.inputMatrix,test.bcmBeam)-test.bcmBeam{i}.gradientConstraintX(test.inputMatrix,test.bcmBeam,test.load);
            end
            f=f-test.load*x;
            f=f*1000;
            g=g*1000;
        end
        
        function [ c,ceq,gradc,gradceq  ] = fMinConCostraint3(test,u)
            test.theta1=u(1);
            gradceq=zeros(1+test.N*3,2);
            c=[];
            gradc=[];
            ceq(1)=test.r1*cos(test.theta1)-test.r3; 
            ceq(2)=test.r1*sin(test.theta1);
            for i=1:test.N
                test.bcmBeam{i}.deltaX=u(3*(i-1)+2);
                test.bcmBeam{i}.deltaY=u(3*(i-1)+3);
                test.bcmBeam{i}.theta=u(3*(i-1)+4);
                if i~=1
                    test.bcmBeam{i}.masterAngle=test.bcmBeam{i-1}.theta;
                end
                ceq(1)=ceq(1)-test.bcmBeam{i}.x;
                ceq(2)=ceq(2)-test.bcmBeam{i}.y;
                gradceq(:,1)=gradceq(:,1)-test.bcmBeam{i}.gradientConstraintX(test.inputMatrix,test.bcmBeam,1.0);
                gradceq(:,2)=gradceq(:,2)-test.bcmBeam{i}.gradientConstraintY(test.inputMatrix,test.bcmBeam,1.0);
            end
            test.theta2=test.bcmBeam{end}.angle-70*pi/180;
            ceq(1)=ceq(1)+test.r2*cos(test.theta2);
            ceq(2)=ceq(2)+test.r2*sin(test.theta2);
            gradceq(:,1)=gradceq(:,1)+[-test.r1*sin(test.theta1);zeros(3*test.N,1)]...
                +test.bcmBeam{end}.gradientConstraintTheta(test.inputMatrix,-test.r2*sin(test.theta2));
            gradceq(:,2)=gradceq(:,2)+[test.r1*cos(test.theta1);zeros(3*test.N,1)]...
                +test.bcmBeam{end}.gradientConstraintTheta(test.inputMatrix,test.r2*cos(test.theta2));
        end
        
        function Hout = hessianfcn2(test,u,lambda)
            test.theta1=u(1);
            Hout=zeros(1+test.N*3,1+test.N*3);
            for i=1:test.N
                test.bcmBeam{i}.deltaX=u(3*(i-1)+2);
                test.bcmBeam{i}.deltaY=u(3*(i-1)+3);
                test.bcmBeam{i}.theta=u(3*(i-1)+4);
                if i~=1
                    test.bcmBeam{i}.masterAngle=test.bcmBeam{i-1}.theta;
                end
                Hout=Hout+test.bcmBeam{i}.hessian(test.inputMatrix,test.bcmBeam)*1000;
                Hout=Hout+test.bcmBeam{i}.hessianX(test.inputMatrix,test.bcmBeam,-lambda.eqnonlin(1))...
                 +test.bcmBeam{i}.hessianY(test.inputMatrix,test.bcmBeam,-lambda.eqnonlin(2));
            end
            test.theta2=test.bcmBeam{end}.angle-70*pi/180;
            Hout(1,1)=Hout(1,1)+lambda.eqnonlin(1)*-test.r1*cos(test.theta1)+lambda.eqnonlin(2)*-test.r1*sin(test.theta1);
            Hout=Hout+test.bcmBeam{end}.hessianTheta(test.inputMatrix,-test.r2*cos(test.theta2)*lambda.eqnonlin(1))...
                 +test.bcmBeam{end}.hessianTheta(test.inputMatrix,-test.r2*sin(test.theta2)*lambda.eqnonlin(2));
        end
    end
end