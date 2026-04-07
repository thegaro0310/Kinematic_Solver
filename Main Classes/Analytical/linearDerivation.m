clear
clc
syms x y theta EI l yy p m t E xx
[p,m] = solve([y==p*l^3/(3*EI)+m*l^2/(2*EI),theta==p*l^2/(2*EI)+m*l/(EI)],[p,m]);
yy=p*xx^2/(6*EI)*(3*l-xx)+m*xx^2/(2*EI);
e=simplify(int( diff(diff(yy,xx),xx)^2,0,l)*EI/2,10);
g=simplify(gradient(e,[x y theta]),10);
h=simplify(hessian(e,[x y theta]),10);
stress=simplify(diff(diff(yy,xx),xx)*t/2*E,10);
yPos=simplify(p*xx^2/(6*EI)*(3*l-xx)+m*xx^2/(2*EI),10);
equation.e=matlabFunction(e,'Vars',[x y theta l EI]);
equation.g=matlabFunction(g,'Vars',[x y theta l EI]);
equation.h=matlabFunction(h,'Vars',[x y theta l EI]);
equation.stress=matlabFunction(stress,'Vars',[x y theta t E xx l]);
equation.yPos=matlabFunction(yPos,'Vars',[x y theta l EI xx]);
save('LinearEquations.mat','-struct','equation');