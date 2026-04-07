function [newState,exitflag,stable]=solveStaticAnalysis(allBeams,torsionSprings,kinematicEq,forces,moments,input,power,additionalDof,lengthFactor,forceFactor,EFactor)
[~,~,initialGuess] = findInputMatrix(allBeams,zeros(allBeams(end).id+allBeams(end).degreesOfFreedom+3,1),input,additionalDof);
fMincon=@(x)fMinConFunc( x,allBeams,torsionSprings,forces,moments,input,power,additionalDof,lengthFactor,forceFactor,EFactor);
gMinCon=@(x)fMinConCostraint( x,allBeams,input,kinematicEq,additionalDof);
opt=optimoptions(@fmincon,'Display','iter-detailed','Algorithm','interior-point','MaxFunEvals',50000,'MaxIter',1000,'ConstraintTolerance',1e-8,'FunctionTolerance',1e-10,...
    'OptimalityTolerance',1e-10,'SpecifyConstraintGradient',true,'SpecifyObjectiveGradient',true...
    ,'HessianFcn',@(x,lambda)hessianfcn(x,lambda,allBeams,torsionSprings,forces,power,input,kinematicEq,additionalDof,lengthFactor,forceFactor,EFactor)...
    ,'FiniteDifferenceType','central','CheckGradients',false);

if isempty(initialGuess)
    %means statically indeterminate
    newState=[];
    exitflag=31;
    return;
end
%profile on
tic
[newState,fval,exitflag,~,~,~,hessian] = fmincon(fMincon,initialGuess,[],[],[],[],[],[],gMinCon,opt);
toc
%profile viewer

[~,p] = chol(hessian);
if p>0
    stable=0;
elseif p ==0
    stable=1;
end
if exitflag<=0
    exitflag
    fval
end
end

function [f,g] = fMinConFunc(x,allBeams,torsionSprings,forces,moments,input,power,additionalDof,lengthFactor,forceFactor,EFactor)
%FMINCONFUNC this functions calculates potantial energy which will be
%minimized
[outputList,indexList,~] = findInputMatrix(allBeams,x,input,additionalDof);
g=zeros(max(indexList),1);
%

%only update prb beams
for i=1:length(allBeams)
    if strcmp(allBeams(i).class,'PrbBeam')
        allBeams(i)=updateBeam(allBeams(i),outputList,indexList);
    end
end

energy=0;
for i=1:length(allBeams)
    %energy stored in beams
    g=calculateEnergyGradient(g,allBeams(i),outputList,indexList,lengthFactor,forceFactor,EFactor);
    energy=energy+calculateTotalEnergy(allBeams(i),outputList,indexList,lengthFactor,forceFactor,EFactor);
end

%torsion springs
for i=1:length(torsionSprings)
    if logical(torsionSprings(i).active)
        magnitude=torsionSprings(i).stiffness*forceFactor*lengthFactor;
        node=torsionSprings(i).node;
        link1=torsionSprings(i).link1;
        link2=torsionSprings(i).link2;
        %check link 1
        if link1 >0
            if node == allBeams(link1).nodes(1,1)
                [angle1,index1]=getAngle(allBeams(link1),outputList,indexList,0.0);
            elseif node == allBeams(link1).nodes(1,2)
                [angle1,index1]=getAngle(allBeams(link1),outputList,indexList,100.0);
            end
        end
        %check link 2
        if link2 >0
            if node == allBeams(link2).nodes(1,1)
                [angle2,index2]=getAngle(allBeams(link2),outputList,indexList,0.0);
            elseif node == allBeams(link2).nodes(1,2)
                [angle2,index2]=getAngle(allBeams(link2),outputList,indexList,100.0);
            end
        end
        if torsionSprings(i).link1==0 && torsionSprings(i).link2 ==0
            continue;
        elseif torsionSprings(i).link1==0
            energy=energy+1/2*magnitude*(angle2 -torsionSprings(i).theta0(1,2))^2;
            %gradient
            if logical(index2)
                g(index2,1)=g(index2,1)+magnitude*(angle2 -torsionSprings(i).theta0(1,2));
            end
        elseif torsionSprings(i).link2==0
            energy=energy+1/2*magnitude*(  angle1 -torsionSprings(i).theta0(1,1))^2;
            %gradient
            if logical(index1)
                g(index1,1)=g(index1,1)+magnitude*(  angle1 -torsionSprings(i).theta0(1,1));
            end
        else
            energy=energy+1/2*magnitude*( ( angle1 -torsionSprings(i).theta0(1,1))-( angle2-torsionSprings(i).theta0(1,2)))^2;
            %gradient
            if logical(index1)
                g(index1,1)=g(index1,1)+magnitude*( ( angle1 -torsionSprings(i).theta0(1,1))-( angle2-torsionSprings(i).theta0(1,2)));
            end
            if logical(index2)
                g(index2,1)=g(index2,1)-magnitude*( ( angle1 -torsionSprings(i).theta0(1,1))-( angle2-torsionSprings(i).theta0(1,2)));
            end
        end
    end
end

forceEnergy=0;
for i=1:length(forces)
    if logical(forces(i).active)
        [ xValue,yValue,gradient,~] = solveStaticEquations(allBeams,outputList,forces(i).kinematicEquations,indexList,0);
        %not follower
        magnitudeX=forces(i).xValue*forceFactor();
        magnitudeY=forces(i).yValue*forceFactor();
        xDistance=(xValue-forces(i).initialDistanceX)*lengthFactor();
        yDistance=(yValue-forces(i).initialDistanceY)*lengthFactor();
        forceEnergy=forceEnergy+xDistance*magnitudeX*power/100+yDistance*magnitudeY*power/100;
        g=g-gradient(:,1)*lengthFactor()*magnitudeX*power/100-gradient(:,2)*lengthFactor()*magnitudeY*power/100;
    end
end

for i=1:length(moments)
    if logical(moments(i).active)
        [angle,index]=getAngle(allBeams(moments(i).link),outputList,indexList,moments(i).distance);
        magnitude=moments(i).magnitude*forceFactor*lengthFactor;
        forceEnergy=forceEnergy+magnitude*(angle-moments(i).angle0)*power/100;
        g(index,1)=g(index,1)-magnitude*power/100;
    end
end

f=(energy-forceEnergy)/(lengthFactor()*forceFactor());
g=g/(lengthFactor()*forceFactor());
end




function [ c,ceq,gradc,gradceq ] = fMinConCostraint(x,allBeams,input,kinematicEq,additionalDof)
[outputList,indexList,~] = findInputMatrix(allBeams,x,input,additionalDof);
%only update prb beams
for i=1:length(allBeams)
    if strcmp(allBeams(i).class,'PrbBeam')
        allBeams(i)=updateBeam(allBeams(i),outputList,indexList);
    end
end
[ xValue,yValue,gradient,~] = solveStaticEquations(allBeams,outputList,kinematicEq,indexList,0);
c=[];
ceq=zeros(length(kinematicEq)*2,1);
for i=1:length(kinematicEq)
    ceq(2*i-1,1)=xValue(i);
    ceq(2*i,1)=yValue(i);
end
if nargout > 2
    gradc = [];
    gradceq = gradient;
end
end

function Hout = hessianfcn(x,lambda,allBeams,torsionSprings,forces,power,input,kinematicEq,additionalDof,lengthFactor,forceFactor,EFactor)
Hout=sparse(length(x),length(x));
[outputList,indexList,~] = findInputMatrix(allBeams,x,input,additionalDof);
%only update prb beams
for i=1:length(allBeams)
    if strcmp(allBeams(i).class,'PrbBeam')
        allBeams(i)=updateBeam(allBeams(i),outputList,indexList);
    end
end
for i=1:length(allBeams)
    %energy stored in beams
    Hout=Hout+calculateEnergyHessian(allBeams(i),outputList,indexList,lengthFactor,forceFactor,EFactor);
end

%torsion springs
for i=1:length(torsionSprings)
    if logical(torsionSprings(i).active)
        magnitude=torsionSprings(i).stiffness*forceFactor*lengthFactor;
        node=torsionSprings(i).node;
        link1=torsionSprings(i).link1;
        link2=torsionSprings(i).link2;
        %check link 1
        if link1 >0
            if node == allBeams(link1).nodes(1,1)
                [angle1,index1]=getAngle(allBeams(link1),outputList,indexList,0.0);
            elseif node == allBeams(link1).nodes(1,2)
                [angle1,index1]=getAngle(allBeams(link1),outputList,indexList,100.0);
            end
        end
        %check link 2
        if link2 >0
            if node == allBeams(link2).nodes(1,1)
                [angle2,index2]=getAngle(allBeams(link2),outputList,indexList,0.0);
            elseif node == allBeams(link2).nodes(1,2)
                [angle2,index2]=getAngle(allBeams(link2),outputList,indexList,100.0);
            end
        end
        if torsionSprings(i).link1==0 && torsionSprings(i).link2 ==0
            continue;
        elseif torsionSprings(i).link1==0
            %hessian
            if logical(index2)
                Hout(index2,index2)=Hout(index2,index2)+magnitude;
            end
        elseif torsionSprings(i).link2==0
            %hessian
            if logical(index1)
                Hout(index1,index1)=Hout(index1,index1)+magnitude;
            end
        else
            %hessian
            if logical(index2)
                Hout(index2,index2)=Hout(index2,index2)+magnitude;
            end
            if logical(index1)
                Hout(index1,index1)=Hout(index1,index1)+magnitude;
            end
            if logical(index1) && logical(index2)
                Hout(index1,index2)=Hout(index1,index2)-magnitude;
                Hout(index2,index1)=Hout(index2,index1)-magnitude;
            end
        end
    end
end



for i=1:length(forces)
    if logical(forces(i).active)
        [ ~,~,~,hessian] = solveStaticEquations(allBeams,outputList,forces(i).kinematicEquations,indexList,1);
        magnitudeX=forces(i).xValue*forceFactor;
        magnitudeY=forces(i).yValue*forceFactor;
        Hout=Hout-hessian{1}*lengthFactor*magnitudeX*power/100-hessian{2}*lengthFactor*magnitudeY*power/100;
    end
end
Hout=Hout/(lengthFactor*forceFactor);
[ ~,~,~,hessian ] = solveStaticEquations(allBeams,outputList,kinematicEq,indexList,1);
for i=1:length(lambda.eqnonlin)
    Hout=Hout+lambda.eqnonlin(i)*hessian{i};
end
end


% kinematic equations
function x=getX(beam,outputList,distance)
switch beam.class
    case {'CbcmBeam','MlinearBeam'}
        if distance < 0
            x=-(getX(beam,outputList,100)-getX(beam,outputList,100+distance));
        elseif distance == 0
            x=0;
        else
            currentDistance=0;
            x=0;
            for i=1:length(beam.childLengths)
                singleDistance=beam.childLengths(i)/beam.length0*100;
                currentDeltaX=outputList(beam.childIds(i)+1);
                currentDeltaY=outputList(beam.childIds(i)+2);
                coord=getRotationMatrix(beam.childIds(i),beam.theta0,outputList)*[currentDeltaX+beam.childLengths(i);currentDeltaY];
                xSingle=coord(1);
                if currentDistance + singleDistance >= distance
                    modifier=(distance-currentDistance)/singleDistance;
                    x=x+xSingle*modifier;
                    return;
                else
                    x=x+xSingle;
                    currentDistance=currentDistance+singleDistance;
                end
            end
        end
    case 'KinematicsBeam'
        value=getValues(beam,outputList);
        x=beam.equations.xKnown+beam.equations.xPre*KinematicsBeam.preDefinedFunctions(beam.equations.xFunc,value,0);
        x=distance/100*x;
    case 'PrbBeam'
        if distance < 0
            x=-(getX(beam,outputList,100)-getX(beam,outputList,100+distance));
        elseif distance == 0
            x=0;
        else
            currentDistance=0;
            x=0;
            for i=1:length(beam.prbBeams)
                prbDistance=beam.prbModel(i,1)*100;
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
end

function y=getY(beam,outputList,distance)
switch beam.class
    case {'CbcmBeam','MlinearBeam'}
        if distance < 0
            y=-(getY(beam,outputList,100)-getY(beam,outputList,100+distance));
        elseif distance == 0
            y=0;
        else
            currentDistance=0;
            y=0;
            for i=1:length(beam.childLengths)
                singleDistance=beam.childLengths(i)/beam.length0*100;
                %get current
                currentDeltaX=outputList(beam.childIds(i)+1);
                currentDeltaY=outputList(beam.childIds(i)+2);
                coord=getRotationMatrix(beam.childIds(i),beam.theta0,outputList)*[currentDeltaX+beam.childLengths(i);currentDeltaY];
                ySingle=coord(2);
                if currentDistance + singleDistance >= distance
                    modifier=(distance-currentDistance)/singleDistance;
                    y=y+ySingle*modifier;
                    return;
                else
                    y=y+ySingle;
                    currentDistance=currentDistance+singleDistance;
                end
            end
        end
    case 'KinematicsBeam'
        value=getValues(beam,outputList);
        y=beam.equations.yKnown+beam.equations.yPre*KinematicsBeam.preDefinedFunctions(beam.equations.yFunc,value,0);
        y=distance/100*y;
    case 'PrbBeam'
        if distance < 0
            y=-(getY(beam,outputList,100)-getY(beam,outputList,100+distance));
        elseif distance == 0
            y=0;
        else
            currentDistance=0;
            y=0;
            for i=1:length(beam.prbBeams)
                prbDistance=beam.prbModel(i,1)*100;
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
end


function [angle,index]=getAngle(beam,outputList,indexList,distance)
switch beam.class
    case 'KinematicsBeam'
        angle=outputList(beam.id);
        index=indexList(beam.id);
    case {'CbcmBeam','MlinearBeam'}
        currentDistance=0;
        for i=1:length(beam.childIds)
            singleDistance=beam.childLengths(i)/beam.length0*100;
            if currentDistance + singleDistance >= distance
                angle=outputList(beam.childIds(i)+3);
                index=indexList(beam.childIds(i)+3);
                return;
            end
            currentDistance=currentDistance+singleDistance;
        end
        %numerical error check
        angle=outputList(beam.childIds(end)+3);
        index=indexList(beam.childIds(end)+3);
    case 'PrbBeam'
        currentDistance=0;
        angle=[];
        for i=1:length(beam.prbBeams)
            prbDistance=beam.prbModel(i,1)*100;
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
end

function gradientX=getGradientX(gradientX,beam,outputList,indexList,distance)
switch beam.class
    case {'CbcmBeam','MlinearBeam'}
        if distance < 0
            gradientX=-(getGradientX(gradientX,beam,outputList,indexList,100)-getGradientX(gradientX,beam,outputList,indexList,100+distance))+gradientX;
        elseif distance == 0
        else
            currentDistance=0;
            for i=1:length(beam.childLengths)
                singleDistance=beam.childLengths(i)/beam.length0*100;
                %get current gradient
                transformation=getRotationMatrix(beam.childIds(i),beam.theta0,outputList);
                beamGradient=[transformation(1,1:2)';0];
                transformation=getRotationGradientMatrix(beam.childIds(i),beam.theta0,outputList);
                currentDeltaX=outputList(beam.childIds(i)+1);
                currentDeltaY=outputList(beam.childIds(i)+2);
                beamTransGradient=transformation(1,:)*[currentDeltaX+beam.childLengths(i);currentDeltaY];
                if currentDistance + singleDistance >= distance
                    modifier=(distance-currentDistance)/singleDistance;
                    returnNow=1;
                else
                    modifier=1;
                    currentDistance=currentDistance+singleDistance;
                    returnNow=0;
                end
                %master angle
                if logical(indexList(beam.childIds(i)))
                    gradientX(indexList(beam.childIds(i)),1)=gradientX(indexList(beam.childIds(i)),1)+beamTransGradient*modifier;
                end
                %delta X
                if logical(indexList(beam.childIds(i)+1))
                    gradientX(indexList(beam.childIds(i)+1),1)=gradientX(indexList(beam.childIds(i)+1),1)+beamGradient(1)*modifier;
                end
                %delta Y
                if logical(indexList(beam.childIds(i)+2))
                    gradientX(indexList(beam.childIds(i)+2),1)=gradientX(indexList(beam.childIds(i)+2),1)+beamGradient(2)*modifier;
                end
                %theta
                if logical(indexList(beam.childIds(i)+3))
                    gradientX(abs(indexList(beam.childIds(i)+3)),1)=gradientX(abs(indexList(beam.childIds(i)+3)),1)+beamGradient(3)*modifier;
                end
                if logical(returnNow)
                    return;
                end
            end
        end
    case 'KinematicsBeam'
        value=getValues(beam,outputList);
        xDerivative=beam.equations.xPre*KinematicsBeam.preDefinedFunctions(beam.equations.xFunc,value,1)*distance/100;
        if strcmp(beam.type,'slider')
            if logical(indexList(beam.id))
                gradientX(indexList(beam.id),1)=gradientX(indexList(beam.id),1)+xDerivative(1);
            end
            if logical(indexList(beam.frontMasterID))
                gradientX(indexList(beam.frontMasterID),1)=gradientX(indexList(beam.frontMasterID),1)+xDerivative(2);
            end
        elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider') || logical(beam.linearSpring)
            if logical(indexList(beam.id))
                gradientX(indexList(beam.id),1)=gradientX(indexList(beam.id),1)+xDerivative(1);
            end
            if logical(indexList(beam.id+1))
                gradientX(indexList(beam.id+1),1)=gradientX(indexList(beam.id+1),1)+xDerivative(2);
            end
        elseif strcmp(beam.type,'allFixedSlider')
            if logical(indexList(beam.id))
                gradientX(indexList(beam.id),1)=gradientX(indexList(beam.id),1)+xDerivative(1);
            end
        else
            if logical(indexList(beam.id))
                gradientX(indexList(beam.id),1)=gradientX(indexList(beam.id),1)+xDerivative(1);
            end
            if logical(indexList(beam.id+1))
                gradientX(indexList(beam.id+1),1)=gradientX(indexList(beam.id+1),1)+xDerivative(2);
            end
        end
    case 'PrbBeam'
        if distance < 0
            gradientX=-(getGradientX(gradientX,beam,outputList,indexList,100)-getGradientX(gradientX,beam,outputList,indexList,100+distance))+gradientX;
        elseif distance == 0
        else
            currentDistance=0;
            for i=1:length(beam.prbBeams)
                prbDistance=beam.prbModel(i,1)*100;
                if currentDistance + prbDistance >= distance || i == length(beam.prbBeams)
                    modifier=(distance-currentDistance)/prbDistance;
                    if logical(beam.prbBeams(i).index(1))
                        gradientX(beam.prbBeams(i).index(1),1)=gradientX(beam.prbBeams(i).index(1),1)-modifier*beam.prbBeams(i).length*sin(beam.prbBeams(i).theta);
                    end
                    if logical(beam.prbBeams(i).index(2))
                        gradientX(beam.prbBeams(i).index(2),1)=gradientX(beam.prbBeams(i).index(2),1)+modifier*cos(beam.prbBeams(i).theta);
                    end
                    return;
                else
                    if logical(beam.prbBeams(i).index(1))
                        gradientX(beam.prbBeams(i).index(1),1)=gradientX(beam.prbBeams(i).index(1),1)-beam.prbBeams(i).length*sin(beam.prbBeams(i).theta);
                    end
                    if logical(beam.prbBeams(i).index(2))
                        gradientX(beam.prbBeams(i).index(2),1)=gradientX(beam.prbBeams(i).index(2),1)+cos(beam.prbBeams(i).theta);
                    end
                    currentDistance=currentDistance+prbDistance;
                end
            end
        end
end
end

function gradientY=getGradientY(gradientY,beam,outputList,indexList,distance)
switch beam.class
    case {'CbcmBeam','MlinearBeam'}
        if distance < 0
            gradientY=-(getGradientY(gradientY,beam,outputList,indexList,100)-getGradientY(gradientY,beam,outputList,indexList,100+distance))+gradientY;
        elseif distance == 0
        else
            currentDistance=0;
            for i=1:length(beam.childLengths)
                singleDistance=beam.childLengths(i)/beam.length0*100;
                %get current gradient
                transformation=getRotationMatrix(beam.childIds(i),beam.theta0,outputList);
                beamGradient=[transformation(2,1:2)';0];
                transformation=getRotationGradientMatrix(beam.childIds(i),beam.theta0,outputList);
                currentDeltaX=outputList(beam.childIds(i)+1);
                currentDeltaY=outputList(beam.childIds(i)+2);
                beamTransGradient=transformation(2,:)*[currentDeltaX+beam.childLengths(i);currentDeltaY];
                if currentDistance + singleDistance >= distance
                    modifier=(distance-currentDistance)/singleDistance;
                    returnNow=1;
                else
                    modifier=1;
                    currentDistance=currentDistance+singleDistance;
                    returnNow=0;
                end
                %master angle
                if logical(indexList(beam.childIds(i)))
                    gradientY(indexList(beam.childIds(i)),1)=gradientY(indexList(beam.childIds(i)),1)+beamTransGradient*modifier;
                end
                %delta X
                if logical(indexList(beam.childIds(i)+1))
                    gradientY(indexList(beam.childIds(i)+1),1)=gradientY(indexList(beam.childIds(i)+1),1)+beamGradient(1)*modifier;
                end
                %delta Y
                if logical(indexList(beam.childIds(i)+2))
                    gradientY(indexList(beam.childIds(i)+2),1)=gradientY(indexList(beam.childIds(i)+2),1)+beamGradient(2)*modifier;
                end
                %theta
                if logical(indexList(beam.childIds(i)+3))
                    gradientY(abs(indexList(beam.childIds(i)+3)),1)=gradientY(abs(indexList(beam.childIds(i)+3)),1)+beamGradient(3)*modifier;
                end
                if logical(returnNow)
                    return;
                end
            end
        end
    case 'KinematicsBeam'
        value=getValues(beam,outputList);
        yDerivative=beam.equations.yPre*KinematicsBeam.preDefinedFunctions(beam.equations.yFunc,value,1)*distance/100;
        if strcmp(beam.type,'slider')
            if logical(indexList(beam.id))
                gradientY(indexList(beam.id),1)=gradientY(indexList(beam.id),1)+yDerivative(1);
            end
            if logical(indexList(beam.frontMasterID))
                gradientY(indexList(beam.frontMasterID),1)=gradientY(indexList(beam.frontMasterID),1)+yDerivative(2);
            end
        elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider') || logical(beam.linearSpring)
            if logical(indexList(beam.id))
                gradientY(indexList(beam.id),1)=gradientY(indexList(beam.id),1)+yDerivative(1);
            end
            if logical(indexList(beam.id+1))
                gradientY(indexList(beam.id+1),1)=gradientY(indexList(beam.id+1),1)+yDerivative(2);
            end
        elseif strcmp(beam.type,'allFixedSlider')
            if logical(indexList(beam.id))
                gradientY(indexList(beam.id),1)=gradientY(indexList(beam.id),1)+yDerivative(1);
            end
        else
            if logical(indexList(beam.id))
                gradientY(indexList(beam.id),1)=gradientY(indexList(beam.id),1)+yDerivative(1);
            end
            if logical(indexList(beam.id+1))
                gradientY(indexList(beam.id+1),1)=gradientY(indexList(beam.id+1),1)+yDerivative(2);
            end
        end
    case 'PrbBeam'
        if distance < 0
            gradientY=-(getGradientY(gradientY,beam,outputList,indexList,100)-getGradientY(gradientY,beam,outputList,indexList,100+distance))+gradientY;
        elseif distance == 0
        else
            currentDistance=0;
            for i=1:length(beam.prbBeams)
                prbDistance=beam.prbModel(i,1)*100;
                if currentDistance + prbDistance >= distance || i == length(beam.prbBeams)
                    modifier=(distance-currentDistance)/prbDistance;
                    if logical(beam.prbBeams(i).index(1))
                        gradientY(beam.prbBeams(i).index(1),1)=gradientY(beam.prbBeams(i).index(1),1)+modifier*beam.prbBeams(i).length*cos(beam.prbBeams(i).theta);
                    end
                    if logical(beam.prbBeams(i).index(2))
                        gradientY(beam.prbBeams(i).index(2),1)=gradientY(beam.prbBeams(i).index(2),1)+modifier*sin(beam.prbBeams(i).theta);
                    end
                    return;
                else
                    if logical(beam.prbBeams(i).index(1))
                        gradientY(beam.prbBeams(i).index(1),1)=gradientY(beam.prbBeams(i).index(1),1)+beam.prbBeams(i).length*cos(beam.prbBeams(i).theta);
                    end
                    if logical(beam.prbBeams(i).index(2))
                        gradientY(beam.prbBeams(i).index(2),1)=gradientY(beam.prbBeams(i).index(2),1)+sin(beam.prbBeams(i).theta);
                    end
                    currentDistance=currentDistance+prbDistance;
                end
            end
        end
end
end

function hessianX=getHessianX(beam,outputList,indexList,distance)
switch beam.class
    case {'CbcmBeam','MlinearBeam'}
        if distance < 0
            hessianX=-(getHessianX(beam,outputList,indexList,100)-getHessianX(beam,outputList,indexList,100+distance));
        elseif distance == 0
            hessianX=sparse(max(indexList),max(indexList));
        else
            currentDistance=0;
            hessianX=sparse(max(indexList),max(indexList));
            for index=1:length(beam.childLengths)
                singleDistance=beam.childLengths(index)/beam.length0*100;
                %get the hessian
                i=[];j=[];v=[];
                transformationGradient=getRotationGradientMatrix(beam.childIds(index),beam.theta0,outputList);
                transformationHessian=getRotationHessianMatrix(beam.childIds(index),beam.theta0,outputList);
                %check if there is master
                if logical(indexList(beam.childIds(index)))
                    %master theta-master theta
                    i(end+1)=indexList(beam.childIds(index));j(end+1)=indexList(beam.childIds(index));
                    currentDeltaX=outputList(beam.childIds(index)+1);
                    currentDeltaY=outputList(beam.childIds(index)+2);
                    v(end+1)=transformationHessian(1,:)*[currentDeltaX+beam.childLengths(index);currentDeltaY];
                    %x-master theta
                    if logical(indexList(beam.childIds(index)+1))
                        i(end+1)=indexList(beam.childIds(index));j(end+1)=indexList(beam.childIds(index)+1);
                        v(end+1)=transformationGradient(1,1);
                        i(end+1)=indexList(beam.childIds(index)+1);j(end+1)=indexList(beam.childIds(index));
                        v(end+1)=transformationGradient(1,1);
                    end
                    %y-master theta
                    if logical(indexList(beam.childIds(index)+2))
                        i(end+1)=indexList(beam.childIds(index));j(end+1)=indexList(beam.childIds(index)+2);
                        v(end+1)=transformationGradient(1,2);
                        i(end+1)=indexList(beam.childIds(index)+2);j(end+1)=indexList(beam.childIds(index));
                        v(end+1)=transformationGradient(1,2);
                    end
                end
                hessianXSingle=sparse(i,j,v,max(indexList),max(indexList));
                if currentDistance + singleDistance >= distance
                    modifier=(distance-currentDistance)/singleDistance;
                    hessianX=hessianX+modifier*hessianXSingle;
                    return;
                else
                    hessianX=hessianX+hessianXSingle;
                    currentDistance=currentDistance+singleDistance;
                end
            end
        end
    case 'KinematicsBeam'
        value=getValues(beam,outputList);
        xHessian=beam.equations.xPre*KinematicsBeam.preDefinedFunctions(beam.equations.xFunc,value,2);
        hessianX=sparse(max(indexList),max(indexList));
        if strcmp(beam.type,'slider')
            if logical(indexList(beam.id))
                hessianX(indexList(beam.id),indexList(beam.id))=xHessian(1);
            end
            if logical(indexList(beam.frontMasterID))
                hessianX(indexList(beam.frontMasterID),indexList(beam.frontMasterID))=xHessian(2);
            end
            if logical(indexList(beam.id)) && logical(indexList(beam.frontMasterID))
                hessianX(indexList(beam.id),indexList(beam.frontMasterID))=xHessian(3);
                hessianX(indexList(beam.frontMasterID),indexList(beam.id))=xHessian(3);
            end
        elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider') || logical(beam.linearSpring)
            if logical(indexList(beam.id))
                hessianX(indexList(beam.id),indexList(beam.id))=xHessian(1);
            end
            if logical(indexList(beam.id+1))
                hessianX(indexList(beam.id+1),indexList(beam.id+1))=xHessian(2);
            end
            if logical(indexList(beam.id)) && logical(indexList(beam.id+1))
                hessianX(indexList(beam.id),indexList(beam.id+1))=xHessian(3);
                hessianX(indexList(beam.id+1),indexList(beam.id))=xHessian(3);
            end
        elseif strcmp(beam.type,'allFixedSlider')
            if logical(indexList(beam.id))
                hessianX(indexList(beam.id),indexList(beam.id))=xHessian(1);
            end
        else
            if logical(indexList(beam.id))
                hessianX(indexList(beam.id),indexList(beam.id))=xHessian(1);
            end
        end
        hessianX=distance/100*hessianX;
    case 'PrbBeam'
        if distance < 0
            hessianX=-(getHessianX(beam,outputList,indexList,100)-getHessianX(beam,outputList,indexList,100+distance));
        elseif distance == 0
            hessianX=sparse(max(indexList),max(indexList));
        else
            ii=[];j=[];v=[];
            currentDistance=0;
            for i=1:length(beam.prbBeams)
                prbDistance=beam.prbModel(i,1)*100;
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
                    hessianX=sparse(ii,j,v,max(indexList),max(indexList));
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
end

function hessianY=getHessianY(beam,outputList,indexList,distance)
switch beam.class
    case {'CbcmBeam','MlinearBeam'}
        if distance < 0
            hessianY=-(getHessianY(beam,outputList,indexList,100)-getHessianY(beam,outputList,indexList,100+distance));
        elseif distance == 0
            hessianY=sparse(max(indexList),max(indexList));
        else
            currentDistance=0;
            hessianY=sparse(max(indexList),max(indexList));
            for index=1:length(beam.childLengths)
                singleDistance=beam.childLengths(index)/beam.length0*100;
                %get the hessian
                i=[];j=[];v=[];
                transformationGradient=getRotationGradientMatrix(beam.childIds(index),beam.theta0,outputList);
                transformationHessian=getRotationHessianMatrix(beam.childIds(index),beam.theta0,outputList);
                %check if there is master
                if logical(indexList(beam.childIds(index)))
                    %master theta-master theta
                    i(end+1)=indexList(beam.childIds(index));j(end+1)=indexList(beam.childIds(index));
                    currentDeltaX=outputList(beam.childIds(index)+1);
                    currentDeltaY=outputList(beam.childIds(index)+2);
                    v(end+1)=transformationHessian(2,:)*[currentDeltaX+beam.childLengths(index);currentDeltaY];
                    %x-master theta
                    if logical(indexList(beam.childIds(index)+1))
                        i(end+1)=indexList(beam.childIds(index));j(end+1)=indexList(beam.childIds(index)+1);
                        v(end+1)=transformationGradient(2,1);
                        i(end+1)=indexList(beam.childIds(index)+1);j(end+1)=indexList(beam.childIds(index));
                        v(end+1)=transformationGradient(2,1);
                    end
                    %y-master theta
                    if logical(indexList(beam.childIds(index)+2))
                        i(end+1)=indexList(beam.childIds(index));j(end+1)=indexList(beam.childIds(index)+2);
                        v(end+1)=transformationGradient(2,2);
                        i(end+1)=indexList(beam.childIds(index)+2);j(end+1)=indexList(beam.childIds(index));
                        v(end+1)=transformationGradient(2,2);
                    end
                end
                hessianYSingle=sparse(i,j,v,max(indexList),max(indexList));
                if currentDistance + singleDistance >= distance
                    modifier=(distance-currentDistance)/singleDistance;
                    hessianY=hessianY+modifier*hessianYSingle;
                    return;
                else
                    hessianY=hessianY+hessianYSingle;
                    currentDistance=currentDistance+singleDistance;
                end
            end
        end
    case 'KinematicsBeam'
        value=getValues(beam,outputList);
        yHessian=beam.equations.yPre*KinematicsBeam.preDefinedFunctions(beam.equations.yFunc,value,2);
        hessianY=sparse(max(indexList),max(indexList));
        if strcmp(beam.type,'slider')
            if logical(indexList(beam.id))
                hessianY(indexList(beam.id),indexList(beam.id))=yHessian(1);
            end
            if logical(indexList(beam.frontMasterID))
                hessianY(indexList(beam.frontMasterID),indexList(beam.frontMasterID))=yHessian(2);
            end
            if logical(indexList(beam.id)) && logical(indexList(beam.frontMasterID))
                hessianY(indexList(beam.id),indexList(beam.frontMasterID))=yHessian(3);
                hessianY(indexList(beam.frontMasterID),indexList(beam.id))=yHessian(3);
            end
        elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider') || logical(beam.linearSpring)
            if logical(indexList(beam.id))
                hessianY(indexList(beam.id),indexList(beam.id))=yHessian(1);
            end
            if logical(indexList(beam.id+1))
                hessianY(indexList(beam.id+1),indexList(beam.id+1))=yHessian(2);
            end
            if logical(indexList(beam.id)) && logical(indexList(beam.id+1))
                hessianY(indexList(beam.id),indexList(beam.id+1))=yHessian(3);
                hessianY(indexList(beam.id+1),indexList(beam.id))=yHessian(3);
            end
        elseif strcmp(beam.type,'allFixedSlider')
            if logical(indexList(beam.id))
                hessianY(indexList(beam.id),indexList(beam.id))=yHessian(1);
            end
        else
            if logical(indexList(beam.id))
                hessianY(indexList(beam.id),indexList(beam.id))=yHessian(1);
            end
        end
        hessianY=distance/100*hessianY;
    case 'PrbBeam'
        %hessian in y coordinates
        if distance < 0
            hessianY=-(getHessianY(beam,outputList,indexList,100)-getHessianY(beam,outputList,indexList,100+distance));
        elseif distance == 0
            hessianY=sparse(max(indexList),max(indexList));
        else
            ii=[];j=[];v=[];
            currentDistance=0;
            for i=1:length(beam.prbBeams)
                prbDistance=beam.prbModel(i,1)*100;
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
                    hessianY=sparse(ii,j,v,max(indexList),max(indexList));
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
end

%


%bcm energy  functions
function energy=bcmEnergy(x,y,theta,t,L)
energy=(1.0./L.^2.*(t.^2.*y.^2.*2.1e3+y.^4.*7.92e2+L.^4.*theta.^4.*2.4e1+L.^2.*x.^2.*2.1e3+L.^3.*theta.^2.*x.*2.8e2-L.^3.*theta.^3.*y.*8.4e1+L.^2.*t.^2.*theta.^2.*7.0e2+L.^2.*theta.^2.*y.^2.*2.81e2-L.*theta.*y.^3.*3.24e2+L.*x.*y.^2.*2.52e3-L.*t.^2.*theta.*y.*2.1e3-L.^2.*theta.*x.*y.*4.2e2).*(3.0./2.0))./(t.^2.*5.25e2+y.^2.*9.0+L.^2.*theta.^2.*1.1e1-L.*theta.*y.*9.0);
end

function gradient= bcmGradient(x,y,theta,t,L)
gradient=[(L.*(x.*6.3e3-theta.*y.*6.3e2)+y.^2.*3.78e3+L.^2.*theta.^2.*4.2e2)./(L.*(t.^2.*5.25e2+y.^2.*9.0+L.^2.*theta.^2.*1.1e1-L.*theta.*y.*9.0));-1.0./L.^2.*1.0./(t.^2.*5.25e2+y.^2.*9.0+L.^2.*theta.^2.*1.1e1-L.*theta.*y.*9.0).^2.*(L.*(t.^4.*theta.*1.65375e6+theta.*y.^4.*3.645e4-t.^2.*x.*y.*3.969e6+t.^2.*theta.*y.^2.*7.6545e5)-t.^2.*y.^3.*2.4948e6+L.^3.*(t.^2.*theta.^3.*9.135e4+theta.^3.*y.^2.*1.86975e4-theta.*x.^2.*2.835e4-theta.^2.*x.*y.*7.56e4)-t.^4.*y.*3.3075e6+L.^4.*(theta.^3.*x.*3.15e3-theta.^4.*y.*8.625e3)-y.^5.*2.1384e4+L.^5.*theta.^5.*1.062e3+L.^2.*(theta.^2.*y.^3.*-6.102e4+x.^2.*y.*5.67e4+t.^2.*theta.*x.*3.3075e5+theta.*x.*y.^2.*2.835e4-t.^2.*theta.^2.*y.*4.92975e5));(1.0./(t.^2.*5.25e2+y.^2.*9.0+L.^2.*theta.^2.*1.1e1-L.*theta.*y.*9.0).^2.*(t.^2.*y.^3.*-5.67e4-t.^4.*y.*3.675e5+y.^5.*1.404e3+L.^5.*theta.^5.*1.76e2-L.^3.*theta.*x.^2.*1.54e4-L.^4.*theta.^4.*y.*5.24e2+L.^2.*x.^2.*y.*6.3e3+L.^3.*t.^2.*theta.^3.*1.68e4-L.^2.*theta.^2.*y.^3.*4.11e2+L.^3.*theta.^3.*y.^2.*7.92e2+L.*t.^4.*theta.*2.45e5-L.*theta.*y.^4.*4.122e3+L.*x.*y.^3.*6.3e3-L.^2.*t.^2.*theta.^2.*y.*3.85e4-L.*t.^2.*x.*y.*7.35e4+L.^2.*t.^2.*theta.*x.*9.8e4+L.*t.^2.*theta.*y.^2.*8.715e4-L.^2.*theta.*x.*y.^2.*1.68e4+L.^3.*theta.^2.*x.*y.*7.0e2).*(9.0./2.0))./L];
end

function hessian= bcmHessian(x,y,theta,t,L)
hessian=reshape([6.3e3./(t.^2.*5.25e2+y.^2.*9.0+L.^2.*theta.^2.*1.1e1-L.*theta.*y.*9.0),(1.0./(t.^2.*5.25e2+y.^2.*9.0+L.^2.*theta.^2.*1.1e1-L.*theta.*y.*9.0).^2.*(L.^2.*(theta.*x.*5.67e4+theta.^2.*y.*7.56e4)+t.^2.*y.*3.969e6-L.*(x.*y.*1.134e5+t.^2.*theta.*3.3075e5+theta.*y.^2.*2.835e4)-L.^3.*theta.^3.*3.15e3))./L,-1.0./(t.^2.*5.25e2+y.^2.*9.0+L.^2.*theta.^2.*1.1e1-L.*theta.*y.*9.0).^2.*(L.^2.*(theta.*x.*1.386e5-theta.^2.*y.*3.15e3)+t.^2.*y.*3.3075e5-L.*(x.*y.*5.67e4+t.^2.*theta.*4.41e5-theta.*y.^2.*7.56e4)-y.^3.*2.835e4),(1.0./(t.^2.*5.25e2+y.^2.*9.0+L.^2.*theta.^2.*1.1e1-L.*theta.*y.*9.0).^2.*(L.^2.*(theta.*x.*5.67e4+theta.^2.*y.*7.56e4)+t.^2.*y.*3.969e6-L.*(x.*y.*1.134e5+t.^2.*theta.*3.3075e5+theta.*y.^2.*2.835e4)-L.^3.*theta.^3.*3.15e3))./L,1.0./L.^2.*1.0./(t.^2.*5.25e2+y.^2.*9.0+L.^2.*theta.^2.*1.1e1-L.*theta.*y.*9.0).^3.*(t.^2.*y.^4.*1.12266e7+t.^4.*y.^2.*1.2800025e9+t.^6.*5.788125e8+y.^6.*6.4152e4+L.^6.*theta.^6.*2.5253e4+L.^5.*theta.^4.*x.*2.583e5-L.^5.*theta.^5.*y.*9.8496e4+L.^2.*t.^4.*theta.^2.*8.8475625e7+L.^4.*t.^2.*theta.^4.*2.76885e6-L.^2.*t.^2.*x.^2.*9.9225e6-L.^4.*theta.^2.*x.^2.*3.78e4+L.^2.*theta.^2.*y.^4.*4.2768e5-L.^3.*theta.^3.*y.^3.*6.05475e5+L.^4.*theta.^4.*y.^2.*5.93595e5+L.^2.*x.^2.*y.^2.*5.103e5+L.*t.^4.*x.*6.94575e8-L.*theta.*y.^5.*1.92456e5+L.^3.*t.^2.*theta.^2.*x.*2.57985e7-L.^3.*t.^2.*theta.^3.*y.*9.5823e6-L.^3.*theta.^2.*x.*y.^2.*6.804e5-L.*t.^4.*theta.*y.*2.3814e8+L.^2.*t.^2.*theta.^2.*y.^2.*5.5041525e7-L.*t.^2.*theta.*y.^3.*2.84067e7-L.*t.^2.*x.*y.^2.*3.5721e7+L.^2.*theta.*x.*y.^3.*1.701e5-L.^3.*theta.*x.^2.*y.*5.103e5+L.^4.*theta.^3.*x.*y.*5.67e4+L.^2.*t.^2.*theta.*x.*y.*5.9535e6).*3.0,(1.0./(t.^2.*5.25e2+y.^2.*9.0+L.^2.*theta.^2.*1.1e1-L.*theta.*y.*9.0).^3.*(t.^2.*y.^4.*-4.1958e6+t.^4.*y.^2.*7.938e7+t.^6.*1.929375e8-y.^6.*1.2636e4+L.^6.*theta.^6.*2.596e3-L.^5.*theta.^4.*x.*7.7e3-L.^5.*theta.^5.*y.*6.372e3+L.^2.*t.^4.*theta.^2.*1.9845e7+L.^4.*t.^2.*theta.^4.*3.962e5-L.^2.*t.^2.*x.^2.*3.3075e6+L.^4.*theta.^2.*x.^2.*2.079e5-L.^2.*theta.^2.*y.^4.*1.55115e5+L.^3.*theta.^3.*y.^3.*1.91925e5-L.^4.*theta.^4.*y.^2.*5.85e2+L.^2.*x.^2.*y.^2.*1.701e5+L.*t.^4.*x.*3.85875e7+L.*theta.*y.^5.*3.7908e4+L.*x.*y.^4.*5.67e4-L.^3.*t.^2.*theta.^2.*x.*1.323e6-L.^3.*t.^2.*theta.^3.*y.*1.7976e6-L.^3.*theta.^2.*x.*y.^2.*1.89e5-L.*t.^4.*theta.*y.*7.938e7+L.^2.*t.^2.*theta.^2.*y.^2.*1.478925e6+L.*t.^2.*theta.*y.^3.*9.7146e6-L.*t.^2.*x.*y.^2.*1.1907e7-L.^2.*theta.*x.*y.^3.*2.457e5-L.^3.*theta.*x.^2.*y.*6.111e5+L.^4.*theta.^3.*x.*y.*3.633e5+L.^2.*t.^2.*theta.*x.*y.*2.18295e7).*(-9.0./2.0))./L,-1.0./(t.^2.*5.25e2+y.^2.*9.0+L.^2.*theta.^2.*1.1e1-L.*theta.*y.*9.0).^2.*(L.^2.*(theta.*x.*1.386e5-theta.^2.*y.*3.15e3)+t.^2.*y.*3.3075e5-L.*(x.*y.*5.67e4+t.^2.*theta.*4.41e5-theta.*y.^2.*7.56e4)-y.^3.*2.835e4),(1.0./(t.^2.*5.25e2+y.^2.*9.0+L.^2.*theta.^2.*1.1e1-L.*theta.*y.*9.0).^3.*(t.^2.*y.^4.*-4.1958e6+t.^4.*y.^2.*7.938e7+t.^6.*1.929375e8-y.^6.*1.2636e4+L.^6.*theta.^6.*2.596e3-L.^5.*theta.^4.*x.*7.7e3-L.^5.*theta.^5.*y.*6.372e3+L.^2.*t.^4.*theta.^2.*1.9845e7+L.^4.*t.^2.*theta.^4.*3.962e5-L.^2.*t.^2.*x.^2.*3.3075e6+L.^4.*theta.^2.*x.^2.*2.079e5-L.^2.*theta.^2.*y.^4.*1.55115e5+L.^3.*theta.^3.*y.^3.*1.91925e5-L.^4.*theta.^4.*y.^2.*5.85e2+L.^2.*x.^2.*y.^2.*1.701e5+L.*t.^4.*x.*3.85875e7+L.*theta.*y.^5.*3.7908e4+L.*x.*y.^4.*5.67e4-L.^3.*t.^2.*theta.^2.*x.*1.323e6-L.^3.*t.^2.*theta.^3.*y.*1.7976e6-L.^3.*theta.^2.*x.*y.^2.*1.89e5-L.*t.^4.*theta.*y.*7.938e7+L.^2.*t.^2.*theta.^2.*y.^2.*1.478925e6+L.*t.^2.*theta.*y.^3.*9.7146e6-L.*t.^2.*x.*y.^2.*1.1907e7-L.^2.*theta.*x.*y.^3.*2.457e5-L.^3.*theta.*x.^2.*y.*6.111e5+L.^4.*theta.^3.*x.*y.*3.633e5+L.^2.*t.^2.*theta.*x.*y.*2.18295e7).*(-9.0./2.0))./L,1.0./(t.^2.*5.25e2+y.^2.*9.0+L.^2.*theta.^2.*1.1e1-L.*theta.*y.*9.0).^3.*(t.^2.*y.^4.*1.20015e6-t.^4.*y.^2.*2.0671875e7-t.^6.*6.43125e7+y.^6.*5.913e3-L.^6.*theta.^6.*9.68e2+L.^5.*theta.^5.*y.*2.376e3-L.^2.*t.^4.*theta.^2.*9.1875e6-L.^4.*t.^2.*theta.^4.*1.386e5+L.^2.*t.^2.*x.^2.*4.0425e6-L.^4.*theta.^2.*x.^2.*2.541e5-L.^2.*theta.^2.*y.^4.*7.8705e4+L.^3.*theta.^3.*y.^3.*8.475e3-L.^4.*theta.^4.*y.^2.*4.32e3+L.^2.*x.^2.*y.^2.*1.26e4-L.*t.^4.*x.*2.5725e7+L.*theta.*y.^5.*5.3136e4+L.*x.*y.^4.*1.89e4+L.^3.*t.^2.*theta.^2.*x.*1.617e6+L.^3.*t.^2.*theta.^3.*y.*2.023e5-L.^3.*theta.^2.*x.*y.^2.*2.772e5+L.*t.^4.*theta.*y.*1.1025e7+L.^2.*t.^2.*theta.^2.*y.^2.*5.87475e5-L.*t.^2.*theta.*y.^3.*1.0773e6+L.*t.^2.*x.*y.^2.*4.6305e6+L.^2.*theta.*x.*y.^3.*2.079e5+L.^3.*theta.*x.^2.*y.*2.079e5+L.^4.*theta.^3.*x.*y.*7.7e3-L.^2.*t.^2.*theta.*x.*y.*2.4255e6).*-9.0],[3,3]);
end

%linear energy functions
function energy=linearEnergy(x,y,theta,l,EI,t)
energy=EI.*1.0./l.^3.*(y.^2.*3.0+l.^2.*theta.^2-l.*theta.*y.*3.0).*2.0+(EI.*1.0./t.^2.*x.^2.*6.0)./l;
end

function gradient= linearGradient(x,y,theta,l,EI,t)
gradient=[(EI.*1.0./t.^2.*x.*1.2e1)./l;EI.*1.0./l.^3.*(y.*6.0-l.*theta.*3.0).*2.0;EI.*1.0./l.^2.*(y.*3.0-l.*theta.*2.0).*-2.0];
end

function hessian= linearHessian(~,~,~,l,EI,t)
hessian=reshape([(EI.*1.0./t.^2.*1.2e1)./l,0.0,0.0,0.0,EI.*1.0./l.^3.*1.2e1,EI.*1.0./l.^2.*-6.0,0.0,EI.*1.0./l.^2.*-6.0,(EI.*4.0)./l],[3,3]);
end


function rotationMatrix=getRotationMatrix(id,theta0,outputList)
%get the rotation matrix
currentAngle=outputList(id)+theta0;
rotationMatrix=[cos(currentAngle) -sin(currentAngle);sin(currentAngle) cos(currentAngle)];
end

function rotationMatrix=getRotationGradientMatrix(id,theta0,outputList)
%get the rotation gradient matrix
currentAngle=outputList(id)+theta0;
rotationMatrix=[-sin(currentAngle) -cos(currentAngle);cos(currentAngle) -sin(currentAngle)];
end

function rotationMatrix=getRotationHessianMatrix(id,theta0,outputList)
%get the rotation hessian matrix
currentAngle=outputList(id)+theta0;
rotationMatrix=[-cos(currentAngle) sin(currentAngle);-sin(currentAngle) -cos(currentAngle)];
end

%get bcm delta theta
function deltaTheta=getDeltaTheta(id,outputList,indexList)
%get the delta theta from the list of the bcm beams
deltaTheta=outputList(id+3);
%master angle
if logical(indexList(id))
    deltaTheta=deltaTheta-outputList(id);
end
end

%prb functions
function beam=updateBeam(beam,outputList,indexList)
%updateBeam prb beams from inputs
inputIndex=beam.id;
%first beam angle
if beam.joints(1,1) == Joint.GroundWelded
    beam.prbBeams(1).theta=beam.prbBeams(1).theta0;
    beam.prbBeams(1).index(1)=0;
else
    beam.prbBeams(1).theta=outputList(inputIndex);
    beam.prbBeams(1).index(1)=indexList(inputIndex);
    inputIndex=inputIndex+1;
end
%first beam length
if ~isinf(beam.prbModel(1,3))
    beam.prbBeams(1).length=outputList(inputIndex);
    beam.prbBeams(1).index(2)=indexList(inputIndex);
    inputIndex=inputIndex+1;
else
    beam.prbBeams(1).length=beam.prbBeams(1).length0;
    beam.prbBeams(1).index(2)=0;
end
%remaining beams
for i=2:size(beam.prbModel,1)
    %angle
    beam.prbBeams(i).theta=outputList(inputIndex);
    beam.prbBeams(i).index(1)=indexList(inputIndex);
    inputIndex=inputIndex+1;
    %linear spring
    if ~isinf(beam.prbModel(i,3))
        beam.prbBeams(i).length=outputList(inputIndex);
        beam.prbBeams(i).index(2)=indexList(inputIndex);
        inputIndex=inputIndex+1;
    else
        beam.prbBeams(i).length=beam.prbBeams(i).length0;
        beam.prbBeams(i).index(2)=0;
    end
end
end

function energy=prbEnergy(beam,outputList,lengthFactor,torsionSpringFactor,linearSpringFactor)
%get energy
%torsion springs
energy=1/2*beam.torsionSprings(1).magnitude*(beam.prbBeams(1).theta-beam.prbBeams(1).theta0)^2;
for i=2:length(beam.torsionSprings)
    energy=energy+1/2*beam.torsionSprings(i).magnitude*torsionSpringFactor*((beam.prbBeams(i).theta-beam.prbBeams(i).theta0)-(beam.prbBeams(i-1).theta-beam.prbBeams(i-1).theta0))^2;
end
%linear springs
for i=1:length(beam.prbBeams)
    if logical(beam.prbBeams(i).linearSpring)
        energy=energy+1/2*beam.prbBeams(i).linearSpring*linearSpringFactor*((beam.prbBeams(i).length-beam.prbBeams(i).length0)*lengthFactor)^2;
    end
end
end

function gradient= prbGradient(beam,gradient,outputList,indexList,lengthFactor,torsionSpringFactor,linearSpringFactor)
%get energy gradient
%torsion springs
if logical(beam.prbBeams(1).index(1))
    gradient(beam.prbBeams(1).index(1))=gradient(beam.prbBeams(1).index(1))+beam.torsionSprings(1).magnitude*torsionSpringFactor*(beam.prbBeams(1).theta-beam.prbBeams(1).theta0);
end
for i=2:length(beam.torsionSprings)
    if logical(beam.prbBeams(i).index(1))
        gradient(beam.prbBeams(i).index(1))= gradient(beam.prbBeams(i).index(1))+beam.torsionSprings(i).magnitude*torsionSpringFactor*((beam.prbBeams(i).theta-beam.prbBeams(i).theta0)-(beam.prbBeams(i-1).theta-beam.prbBeams(i-1).theta0));
    end
    if logical(beam.prbBeams(i-1).index(1))
        gradient(beam.prbBeams(i-1).index(1))= gradient(beam.prbBeams(i-1).index(1)) - beam.torsionSprings(i).magnitude*torsionSpringFactor*((beam.prbBeams(i).theta-beam.prbBeams(i).theta0)-(beam.prbBeams(i-1).theta-beam.prbBeams(i-1).theta0));
    end
end
%linear springs
for i=1:length(beam.prbBeams)
    if logical(beam.prbBeams(i).linearSpring)
        if logical(beam.prbBeams(i).index(2))
            gradient(beam.prbBeams(i).index(2))= gradient(beam.prbBeams(i).index(2))+beam.prbBeams(i).linearSpring*linearSpringFactor*(beam.prbBeams(i).length-beam.prbBeams(i).length0)*lengthFactor()^2;
        end
    end
end
end

function hessian= prbHessian(beam,outputList,indexList,lengthFactor,torsionSpringFactor,linearSpringFactor)
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
        v(end+1)=beam.torsionSprings(i).magnitude*torsionSpringFactor;
    end
    if logical(beam.prbBeams(i-1).index(1))
        ii(end+1)=beam.prbBeams(i-1).index(1);j(end+1)=beam.prbBeams(i-1).index(1);
        v(end+1)=beam.torsionSprings(i).magnitude*torsionSpringFactor;
    end
    if logical(beam.prbBeams(i).index(1)) && logical(beam.prbBeams(i-1).index(1))
        ii(end+1)=beam.prbBeams(i-1).index(1);j(end+1)=beam.prbBeams(i).index(1);
        v(end+1)=-beam.torsionSprings(i).magnitude*torsionSpringFactor;
        ii(end+1)=beam.prbBeams(i).index(1);j(end+1)=beam.prbBeams(i-1).index(1);
        v(end+1)=-beam.torsionSprings(i).magnitude*torsionSpringFactor;
    end
end
%linear springs
for i=1:length(beam.prbBeams)
    if logical(beam.prbBeams(i).linearSpring)
        if logical(beam.prbBeams(i).index(2))
            ii(end+1)=beam.prbBeams(i).index(2);j(end+1)=beam.prbBeams(i).index(2);
            v(end+1)=beam.prbBeams(i).linearSpring*linearSpringFactor*lengthFactor^2;
        end
    end
end
hessian=sparse(ii,j,v,max(indexList),max(indexList));
end

%%%
%kinematic beam functions
function value=getValues(beam,outputList)
%get values from the input matrix
if strcmp(beam.type,'slider')
    value(1)=outputList(beam.id);
    value(2)=beam.theta;
    value(3)=beam.theta0;
elseif strcmp(beam.type,'fixedPinSlider') || strcmp(beam.type,'pinSlider')
    value(1)=outputList(beam.id);
    value(2)=outputList(beam.id+1);
elseif strcmp(beam.type,'allFixedSlider')
    value=outputList(beam.id);
elseif  ~logical(beam.linearSpring)
    value=outputList(beam.id);
else
    value(1)=outputList(beam.id);
    value(2)=outputList(beam.id+1);
end
end
%


function energy=calculateTotalEnergy(beam,outputList,indexList,lengthFactor,forceFactor,EFactor)
energy=0.0;
switch beam.class
    case {'CbcmBeam','MlinearBeam'}
        for i=1:length(beam.childLengths)
            deltaTheta=getDeltaTheta(beam.childIds(i),outputList,indexList);
            deltaX=outputList(beam.childIds(i)+1);
            deltaY=outputList(beam.childIds(i)+2);
            I=1/12*beam.inPlaneThickness^3*beam.outPlaneThickness;
            switch beam.class
                case 'CbcmBeam'
                    energy=energy+bcmEnergy(deltaX,deltaY,deltaTheta,beam.inPlaneThickness,beam.childLengths(i))...
                        *beam.E*EFactor()*I/beam.childLengths(i)*lengthFactor^3;
                case 'MlinearBeam'
                    energy=energy+linearEnergy(deltaX*lengthFactor,deltaY*lengthFactor,deltaTheta,...
                        beam.childLengths(i)*lengthFactor,beam.E*EFactor*I*lengthFactor^4,beam.inPlaneThickness*lengthFactor);
            end
        end
    case 'KinematicsBeam'
        if logical(beam.linearSpring)
            energy=1/2*(beam.linearSpring*forceFactor/lengthFactor)*((outputList(beam.id+1)-beam.length0)*lengthFactor)^2;
        end
    case 'PrbBeam'
        energy=prbEnergy(beam,outputList,lengthFactor,lengthFactor*forceFactor,forceFactor/lengthFactor);
end
end

function gradient=calculateEnergyGradient(gradient,beam,outputList,indexList,lengthFactor,forceFactor,EFactor)
switch beam.class
    case {'CbcmBeam','MlinearBeam'}
        for i=1:length(beam.childLengths)
            deltaTheta=getDeltaTheta(beam.childIds(i),outputList,indexList);
            deltaX=outputList(beam.childIds(i)+1);
            deltaY=outputList(beam.childIds(i)+2);
            I=1/12*beam.inPlaneThickness^3*beam.outPlaneThickness;
            switch beam.class
                case 'CbcmBeam'
                    beamGradient=bcmGradient(deltaX,deltaY,deltaTheta,beam.inPlaneThickness,beam.childLengths(i))...
                        *beam.E*EFactor()*I/beam.childLengths(i)*lengthFactor^3;
                case 'MlinearBeam'
                    beamGradient=linearGradient(deltaX*lengthFactor,deltaY*lengthFactor,deltaTheta,...
                        beam.childLengths(i)*lengthFactor,beam.E*EFactor*I*lengthFactor^4,beam.inPlaneThickness*lengthFactor);
                    beamGradient(1:2)=beamGradient(1:2)*lengthFactor;
            end
            gradient(indexList(beam.childIds(i)+1),1)=gradient(indexList(beam.childIds(i)+1),1)+beamGradient(1);
            gradient(indexList(beam.childIds(i)+2),1)=gradient(indexList(beam.childIds(i)+2),1)+beamGradient(2);
            gradient(indexList(beam.childIds(i)+3),1)=gradient(indexList(beam.childIds(i)+3),1)+beamGradient(3);
            %update master angle
            if logical(indexList(beam.childIds(i)))
                gradient(indexList(beam.childIds(i)),1)=gradient(indexList(beam.childIds(i)),1)-beamGradient(3);
            end
        end
    case 'KinematicsBeam'
        if logical(beam.linearSpring) && logical(indexList(beam.id+1))
            gradient(indexList(beam.id+1),1)=gradient(indexList(beam.id+1),1)+(beam.linearSpring*forceFactor/lengthFactor)*((outputList(beam.id+1)-beam.length0)*lengthFactor^2);
        end
    case 'PrbBeam'
        gradient=prbGradient(beam,gradient,outputList,indexList,lengthFactor,lengthFactor*forceFactor,forceFactor/lengthFactor);
end
end

function hessian=calculateEnergyHessian(beam,outputList,indexList,lengthFactor,forceFactor,EFactor)
switch beam.class
    case {'CbcmBeam','MlinearBeam'}
        i=[];j=[];v=[];
        for index=1:length(beam.childLengths)
            deltaTheta=getDeltaTheta(beam.childIds(index),outputList,indexList);
            deltaX=outputList(beam.childIds(index)+1);
            deltaY=outputList(beam.childIds(index)+2);
            I=1/12*beam.inPlaneThickness^3*beam.outPlaneThickness;
            switch beam.class
                case 'CbcmBeam'
                    beamHessian=bcmHessian(deltaX,deltaY,deltaTheta,beam.inPlaneThickness,beam.childLengths(index))...
                        *beam.E*EFactor()*I/beam.childLengths(index)*lengthFactor^3;
                case 'MlinearBeam'
                    beamHessian=linearHessian(deltaX*lengthFactor,deltaY*lengthFactor,deltaTheta,...
                        beam.childLengths(index)*lengthFactor,beam.E*EFactor*I*lengthFactor^4,beam.inPlaneThickness*lengthFactor)*lengthFactor^2;
                    beamHessian(:,3)=beamHessian(:,3)/lengthFactor;
                    beamHessian(3,:)=beamHessian(3,:)/lengthFactor;
            end
            %x-x
            if logical(indexList(beam.childIds(index)+1))
                i(end+1)=indexList(beam.childIds(index)+1);j(end+1)=indexList(beam.childIds(index)+1);
                v(end+1)=beamHessian(1,1);
            end
            %x-y
            if logical(indexList(beam.childIds(index)+1)) && logical(indexList(beam.childIds(index)+2))
                i(end+1)=indexList(beam.childIds(index)+1);j(end+1)=indexList(beam.childIds(index)+2);
                v(end+1)=beamHessian(1,2);
                i(end+1)=indexList(beam.childIds(index)+2);j(end+1)=indexList(beam.childIds(index)+1);
                v(end+1)=beamHessian(1,2);
            end
            %x-theta
            if logical(indexList(beam.childIds(index)+1)) && logical(indexList(beam.childIds(index)+3))
                i(end+1)=indexList(beam.childIds(index)+1);j(end+1)=indexList(beam.childIds(index)+3);
                v(end+1)=beamHessian(1,3);
                i(end+1)=indexList(beam.childIds(index)+3);j(end+1)=indexList(beam.childIds(index)+1);
                v(end+1)=beamHessian(1,3);
            end
            %y-y
            if logical(indexList(beam.childIds(index)+2))
                i(end+1)=indexList(beam.childIds(index)+2);j(end+1)=indexList(beam.childIds(index)+2);
                v(end+1)=beamHessian(2,2);
            end
            %y-theta
            if logical(indexList(beam.childIds(index)+2)) && logical(indexList(beam.childIds(index)+3))
                i(end+1)=indexList(beam.childIds(index)+2);j(end+1)=indexList(beam.childIds(index)+3);
                v(end+1)=beamHessian(2,3);
                i(end+1)=indexList(beam.childIds(index)+3);j(end+1)=indexList(beam.childIds(index)+2);
                v(end+1)=beamHessian(2,3);
            end
            %theta-theta
            if logical(indexList(beam.childIds(index)+3))
                i(end+1)=indexList(beam.childIds(index)+3);j(end+1)=indexList(beam.childIds(index)+3);
                v(end+1)=beamHessian(3,3);
            end
            %other beam theta dependencies
            if logical(indexList(beam.childIds(index)))
                %mastertheta-master theta
                i(end+1)=indexList(beam.childIds(index));j(end+1)=indexList(beam.childIds(index));
                v(end+1)=beamHessian(3,3);
                %x-master theta
                if logical(indexList(beam.childIds(index)+1))
                    i(end+1)=indexList(beam.childIds(index));j(end+1)=indexList(beam.childIds(index)+1);
                    v(end+1)=-beamHessian(1,3);
                    i(end+1)=indexList(beam.childIds(index)+1);j(end+1)=indexList(beam.childIds(index));
                    v(end+1)=-beamHessian(1,3);
                end
                %y-master theta
                if logical(indexList(beam.childIds(index)+2))
                    i(end+1)=indexList(beam.childIds(index));j(end+1)=indexList(beam.childIds(index)+2);
                    v(end+1)=-beamHessian(2,3);
                    i(end+1)=indexList(beam.childIds(index)+2);j(end+1)=indexList(beam.childIds(index));
                    v(end+1)=-beamHessian(2,3);
                end
                %theta-master theta
                if logical(indexList(beam.childIds(index)+3))
                    i(end+1)=indexList(beam.childIds(index));j(end+1)=indexList(beam.childIds(index)+3);
                    v(end+1)=-beamHessian(3,3);
                    i(end+1)=indexList(beam.childIds(index)+3);j(end+1)=indexList(beam.childIds(index));
                    v(end+1)=-beamHessian(3,3);
                end
            end
        end
        hessian=sparse(i,j,v,max(indexList),max(indexList));
    case 'KinematicsBeam'
        hessian=sparse([],[],[],max(indexList),max(indexList));
        if logical(beam.linearSpring) && logical(indexList(beam.id+1))
            hessian(indexList(beam.id+1),indexList(beam.id+1))=(beam.linearSpring*forceFactor/lengthFactor*lengthFactor^2);
        end
    case 'PrbBeam'
        hessian=prbHessian(beam,outputList,indexList,lengthFactor,lengthFactor*forceFactor,forceFactor/lengthFactor);
end
end

function [ xValue,yValue,gradient,hessian]  = solveStaticEquations(allBeams,outputList,equations,indexList,type)
if isempty(indexList)
    indexList=zeros(length(outputList),1);
end

j=1;
if ~logical(type)
    xValue=zeros(1, length(equations));
    yValue=zeros(1, length(equations));
else
    xValue=[];
    yValue=[];
end
%derivatives
if ~logical(type)
    gradient=zeros(max(indexList),length(equations)*2);
else
    gradient=[];
end
if logical(type)
    hessian=cell(1,length(equations)*2);
else
    hessian=[];
end
for i=1:length(equations)
    if logical(type)
        hessianX=sparse(max(indexList),max(indexList));
        hessianY=sparse(max(indexList),max(indexList));
    else
        gradientX=zeros(max(indexList),1);
        gradientY=zeros(max(indexList),1);
        xValue(i)=0;yValue(i)=0;
    end
    for k=1:length(equations(i).beamList)
        if logical(equations(i).beamList(k))
            if ~logical(type)
                xValue(i)=xValue(i)+getX(allBeams(k),outputList,equations(i).beamList(k)*100);
                yValue(i)=yValue(i)+getY(allBeams(k),outputList,equations(i).beamList(k)*100);
                gradientX=getGradientX(gradientX,allBeams(k),outputList,indexList,equations(i).beamList(k)*100);
                gradientY=getGradientY(gradientY,allBeams(k),outputList,indexList,equations(i).beamList(k)*100);
            else
                hessianX=hessianX+getHessianX(allBeams(k),outputList,indexList,equations(i).beamList(k)*100);
                hessianY=hessianY+getHessianY(allBeams(k),outputList,indexList,equations(i).beamList(k)*100);
            end
        end
    end
    if ~logical(type)
        gradient(:,2*i-1)=gradientX;
        gradient(:,2*i)=gradientY;
    else
        hessian{1,2*i-1}=hessianX;
        hessian{1,2*i}=hessianY;
    end
    j=j+2;
end
end

%get the initial guess
function [outputList,indexList,initialGuess] = findInputMatrix(allBeams,inputMatrix,input,additionalDof)
%prepares the input matrix for solvers
switch allBeams(end).class
    case 'KinematicsBeam'
        degreesOfFreedom=allBeams(end).id+1;
    otherwise
        degreesOfFreedom=allBeams(end).id+allBeams(end).degreesOfFreedom-1;
end
initialGuess=[];
outputList=zeros(1,degreesOfFreedom);
indexList=zeros(1,degreesOfFreedom);
%assign the inputs
%input(length(inputs))=struct('index',[],'dof',[],'value',[]);
for i=1:length(input)
    if input(i).index > 0
        switch allBeams(input(i).index).class
            case 'KinematicsBeam'
                if ~logical(input(i).value)
                    input(i).value=1e-10;
                end
                if logical(allBeams(input(i).index).frontMasterID)
                    outputList(allBeams(input(i).index).frontMasterID)=input(i).value-allBeams(input(i).index).theta0;
                elseif logical(allBeams(input(i).index).endMasterID)
                    inputValue=input(i).value-allBeams(input(i).index).theta0;
                    if ~logical(inputValue)
                        inputValue=1e-6;
                    end
                    outputList(allBeams(input(i).index).endMasterID)=inputValue;
                else
                    outputList(allBeams(input(i).index).id+input(i).dof-1)=input(i).value;
                end
        end
    end
end
inputPosition=1;
%add the additional inputs
for i=1:length(additionalDof)
    if ~logical(additionalDof(i).known) && ~logical(outputList(i))
        if isempty(additionalDof(i).value)
            initialGuess(end+1)=0;
        else
            initialGuess(end+1)=additionalDof(i).value;
        end
        additionalDof(i).value=inputMatrix(inputPosition);
        outputList(i)=inputMatrix(inputPosition);
        indexList(i)=inputPosition;
        inputPosition=inputPosition+1;
    end
end
for i=1:length(allBeams)
    switch allBeams(i).class
        case 'KinematicsBeam'
            if strcmp(allBeams(i).type,'slider')
                if ~logical(outputList(allBeams(i).id))
                    %length
                    outputList(allBeams(i).id)=inputMatrix(inputPosition);
                    indexList(allBeams(i).id)=inputPosition;
                    inputPosition=inputPosition+1;
                    initialGuess(end+1)=allBeams(i).length;
                end
            elseif strcmp(allBeams(i).type,'allFixedSlider')
                if ~logical(outputList(allBeams(i).id))
                    %length
                    outputList(allBeams(i).id)=inputMatrix(inputPosition);
                    indexList(allBeams(i).id)=inputPosition;
                    inputPosition=inputPosition+1;
                    initialGuess(end+1)=allBeams(i).initialGuess;
                end
            elseif allBeams(i).fixed == 1 || (allBeams(i).degreesOfFreedom == 1 &&  logical(allBeams(i).linearSpring)) || allBeams(i).degreesOfFreedom == 0
                if logical(allBeams(i).frontMasterID) || logical(allBeams(i).endMasterID)
                    if logical(allBeams(i).frontMasterID)
                        master=allBeams(i).frontMasterID;
                    else
                        master=allBeams(i).endMasterID;
                    end
                    outputList(allBeams(i).id)=allBeams(i).theta0+outputList(master);
                    indexList(allBeams(i).id)=indexList(master);
                else
                    outputList(allBeams(i).id)=allBeams(i).theta;
                end
                %check if linear spring
                if logical(allBeams(i).linearSpring)
                    if ~logical(outputList(allBeams(i).id+1))
                        outputList(allBeams(i).id+1)=inputMatrix(inputPosition);
                        indexList(allBeams(i).id+1)=inputPosition;
                        inputPosition=inputPosition+1;
                        initialGuess(end+1)=allBeams(i).length;
                    end
                end
            elseif allBeams(i).degreesOfFreedom >1
                if ~logical(outputList(allBeams(i).id))
                    if logical(allBeams(i).theta)
                        initialGuess(end+1)=allBeams(i).theta;
                    else
                        initialGuess(end+1)=1e-6;
                    end
                    outputList(allBeams(i).id)=inputMatrix(inputPosition);
                    indexList(allBeams(i).id)=inputPosition;
                    inputPosition=inputPosition+1;
                end
                if ~logical(outputList(allBeams(i).id+1))
                    initialGuess(end+1)=allBeams(i).length;
                    outputList(allBeams(i).id+1)=inputMatrix(inputPosition);
                    indexList(allBeams(i).id+1)=inputPosition;
                    inputPosition=inputPosition+1;
                end
            else
                if ~logical(outputList(allBeams(i).id))
                    if logical(allBeams(i).frontMasterID) || logical(allBeams(i).endMasterID)
                        if logical(allBeams(i).frontMasterID)
                            master=allBeams(i).frontMasterID;
                        else
                            master=allBeams(i).endMasterID;
                        end
                        outputList(allBeams(i).id)=allBeams(i).theta0+outputList(master);
                        indexList(allBeams(i).id)=indexList(master);
                    else
                        outputList(allBeams(i).id)=inputMatrix(inputPosition);
                        indexList(allBeams(i).id)=inputPosition;
                        inputPosition=inputPosition+1;
                        if logical(allBeams(i).theta)
                            initialGuess(end+1)=allBeams(i).theta;
                        else
                            initialGuess(end+1)=1e-6;
                        end
                    end
                end
            end
        case 'PrbBeam'
            initialGuessBeam=allBeams(i).initialGuess;
            if logical(allBeams(i).frontMasterID) && logical(allBeams(i).endMasterID) && ~isinf(allBeams(i).prbModel(end,end))
                %two end masters and linear spring at the end
                %first prb angle
                outputList(allBeams(i).id)=allBeams(i).prbBeams(1).theta0+outputList(allBeams(i).frontMasterID);
                indexList(allBeams(i).id)=indexList(allBeams(i).frontMasterID);
                %other dof
                for j=1:allBeams(i).degreesOfFreedom-3
                    outputList(allBeams(i).id+j)=inputMatrix(inputPosition);
                    indexList(allBeams(i).id+j)=inputPosition;
                    inputPosition=inputPosition+1;
                    initialGuess(end+1)=initialGuessBeam(j+1);
                end
                %final prb angle
                outputList(allBeams(i).id+allBeams(i).degreesOfFreedom-2)=allBeams(i).prbBeams(end).theta0+outputList(allBeams(i).endMasterID);
                indexList(allBeams(i).id+allBeams(i).degreesOfFreedom-2)=indexList(allBeams(i).endMasterID);
                %final length
                outputList(allBeams(i).id+allBeams(i).degreesOfFreedom-1)=inputMatrix(inputPosition);
                indexList(allBeams(i).id+allBeams(i).degreesOfFreedom-1)=inputPosition;
                inputPosition=inputPosition+1;
                initialGuess(end+1)=initialGuessBeam(end);
            elseif logical(allBeams(i).frontMasterID) && logical(allBeams(i).endMasterID)
                %two end masters
                outputList(allBeams(i).id)=allBeams(i).prbBeams(1).theta0+outputList(allBeams(i).frontMasterID);
                indexList(allBeams(i).id)=indexList(allBeams(i).frontMasterID);
                %other dof
                for j=1:allBeams(i).degreesOfFreedom-2
                    outputList(allBeams(i).id+j)=inputMatrix(inputPosition);
                    indexList(allBeams(i).id+j)=inputPosition;
                    inputPosition=inputPosition+1;
                    initialGuess(end+1)=initialGuessBeam(j+1);
                end
                %final prb angle
                outputList(allBeams(i).id+allBeams(i).degreesOfFreedom-1)=allBeams(i).prbBeams(end).theta0+outputList(allBeams(i).endMasterID);
                indexList(allBeams(i).id+allBeams(i).degreesOfFreedom-1)=indexList(allBeams(i).endMasterID);
            elseif logical(allBeams(i).frontMasterID)
                %just front master
                %first prb angle
                outputList(allBeams(i).id)=allBeams(i).prbBeams(1).theta0+outputList(allBeams(i).frontMasterID);
                indexList(allBeams(i).id)=indexList(allBeams(i).frontMasterID);
                %other dof
                for j=1:allBeams(i).degreesOfFreedom-1
                    outputList(allBeams(i).id+j)=inputMatrix(inputPosition);
                    indexList(allBeams(i).id+j)=inputPosition;
                    inputPosition=inputPosition+1;
                    initialGuess(end+1)=initialGuessBeam(j+1);
                end
            elseif logical(allBeams(i).endMasterID) && ~isinf(allBeams(i).prbModel(end,end))
                %just end master and linear spring at the end
                %other dof
                for j=0:allBeams(i).degreesOfFreedom-3
                    outputList(allBeams(i).id+j)=inputMatrix(inputPosition);
                    indexList(allBeams(i).id+j)=inputPosition;
                    inputPosition=inputPosition+1;
                    initialGuess(end+1)=initialGuessBeam(j+1);
                end
                %final prb angle
                outputList(allBeams(i).id+allBeams(i).degreesOfFreedom-2)=allBeams(i).prbBeams(end).theta0+outputList(allBeams(i).endMasterID);
                indexList(allBeams(i).id+allBeams(i).degreesOfFreedom-2)=indexList(allBeams(i).endMasterID);
                %final length
                outputList(allBeams(i).id+allBeams(i).degreesOfFreedom-1)=inputMatrix(inputPosition);
                indexList(allBeams(i).id+allBeams(i).degreesOfFreedom-1)=inputPosition;
                inputPosition=inputPosition+1;
                initialGuess(end+1)=initialGuessBeam(end);
            elseif logical(allBeams(i).endMasterID)
                %just end master
                %other dof
                for j=0:allBeams(i).degreesOfFreedom-2
                    outputList(allBeams(i).id+j)=inputMatrix(inputPosition);
                    indexList(allBeams(i).id+j)=inputPosition;
                    inputPosition=inputPosition+1;
                    initialGuess(end+1)=initialGuessBeam(j+1);
                end
                %final prb angle
                outputList(allBeams(i).id+allBeams(i).degreesOfFreedom-1)=allBeams(i).prbBeams(end).theta0+outputList(allBeams(i).endMasterID);
                indexList(allBeams(i).id+allBeams(i).degreesOfFreedom-1)=indexList(allBeams(i).endMasterID);
            else
                %no masters
                for j=0:allBeams(i).degreesOfFreedom-1
                    outputList(allBeams(i).id+j)=inputMatrix(inputPosition);
                    indexList(allBeams(i).id+j)=inputPosition;
                    inputPosition=inputPosition+1;
                    initialGuess(end+1)=initialGuessBeam(j+1);
                end
            end
        case {'CbcmBeam','MlinearBeam'}
            id=allBeams(i).id;
            if logical(allBeams(i).frontMasterID) && logical(allBeams(i).endMasterID)
                %tranformation angle
                for j=1:length(allBeams(i).childLengths)
                    initialGuessBeam=allBeams(i).initialGuess(j,:);
                    %master angle
                    if j == 1
                        outputList(id+(j-1)*4)=outputList(allBeams(i).frontMasterID);
                        indexList(id+(j-1)*4)=indexList(allBeams(i).frontMasterID);
                    else
                        outputList(id+(j-1)*4)=outputList(id+(j-2)*4+3);
                        indexList(id+(j-1)*4)=indexList(id+(j-2)*4+3);
                    end
                    %delta x,delta y
                    for k=1:2
                        outputList(id+(j-1)*4+k)=inputMatrix(inputPosition);
                        indexList(id+(j-1)*4+k)=inputPosition;
                        inputPosition=inputPosition+1;
                        initialGuess(end+1)=initialGuessBeam(k+1);
                    end
                    %theta
                    if j == length(allBeams(i).childLengths)
                        outputList(id+(j-1)*4+3)=outputList(allBeams(i).endMasterID);
                        indexList(id+(j-1)*4+3)=indexList(allBeams(i).endMasterID);
                    else
                        outputList(id+(j-1)*4+3)=inputMatrix(inputPosition);
                        indexList(id+(j-1)*4+3)=inputPosition;
                        inputPosition=inputPosition+1;
                        initialGuess(end+1)=initialGuessBeam(end);
                    end
                end
            elseif logical(allBeams(i).frontMasterID)
                %tranformation angle
                for j=1:length(allBeams(i).childLengths)
                    initialGuessBeam=allBeams(i).initialGuess(j,:);
                    %master angle
                    if j == 1
                        outputList(id+(j-1)*4)=outputList(allBeams(i).frontMasterID);
                        indexList(id+(j-1)*4)=indexList(allBeams(i).frontMasterID);
                    else
                        outputList(id+(j-1)*4)=outputList(id+(j-2)*4+3);
                        indexList(id+(j-1)*4)=indexList(id+(j-2)*4+3);
                    end
                    %delta x,delta y,theta
                    for k=1:3
                        outputList(id+(j-1)*4+k)=inputMatrix(inputPosition);
                        indexList(id+(j-1)*4+k)=inputPosition;
                        inputPosition=inputPosition+1;
                        initialGuess(end+1)=initialGuessBeam(k+1);
                    end
                end
            elseif logical(allBeams(i).endMasterID)
                for j=1:length(allBeams(i).childLengths)
                    initialGuessBeam=allBeams(i).initialGuess(j,:);
                    %master angle
                    if j == 1
                        outputList(id+(j-1)*4)=0;
                        indexList(id+(j-1)*4)=0;
                    else
                        outputList(id+(j-1)*4)=outputList(id+(j-2)*4+3);
                        indexList(id+(j-1)*4)=indexList(id+(j-2)*4+3);
                    end
                    %delta x,delta y
                    for k=1:2
                        outputList(id+(j-1)*4+k)=inputMatrix(inputPosition);
                        indexList(id+(j-1)*4+k)=inputPosition;
                        inputPosition=inputPosition+1;
                        initialGuess(end+1)=initialGuessBeam(k+1);
                    end
                    %theta
                    if j == length(allBeams(i).childLengths)
                        outputList(id+(j-1)*4+3)=outputList(allBeams(i).endMasterID);
                        indexList(id+(j-1)*4+3)=indexList(allBeams(i).endMasterID);
                    else
                        outputList(id+(j-1)*4+3)=inputMatrix(inputPosition);
                        indexList(id+(j-1)*4+3)=inputPosition;
                        inputPosition=inputPosition+1;
                        initialGuess(end+1)=initialGuessBeam(end);
                    end
                end
            else
                for j=1:length(allBeams(i).childLengths)
                    initialGuessBeam=allBeams(i).initialGuess(j,:);
                    %master angle
                    if j == 1
                        outputList(id+(j-1)*4)=0;
                        indexList(id+(j-1)*4)=0;
                    else
                        outputList(id+(j-1)*4)=outputList(id+(j-2)*4+3);
                        indexList(id+(j-1)*4)=indexList(id+(j-2)*4+3);
                    end
                    %delta x,delta y
                    for k=1:3
                        outputList(id+(j-1)*4+k)=inputMatrix(inputPosition);
                        indexList(id+(j-1)*4+k)=inputPosition;
                        inputPosition=inputPosition+1;
                        initialGuess(end+1)=initialGuessBeam(k+1);
                    end
                end
            end
    end
end
end


