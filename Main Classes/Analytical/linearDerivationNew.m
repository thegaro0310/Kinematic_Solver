clear
clc
syms x y theta EI l yy f m t E xx p
[f,m] = solve([y==f*l^3/(3*EI)+m*l^2/(2*EI),theta==f*l^2/(2*EI)+m*l/(EI)],[f,m]);
yy=f*xx^2/(6*EI)*(3*l-xx)+m*xx^2/(2*EI);
p=12*EI/(l*t^2)*x;
e=simplify(int( diff(diff(yy,xx),xx)^2,0,l)*EI/2+1/2*EI*t^2*p^2/(12*l^3)/EI^2*l^4,10);
g=simplify(gradient(e,[x y theta]),10);
h=simplify(hessian(e,[x y theta]),10);
stress=simplify(diff(diff(yy,xx),xx)*t/2*E,10);
yPos=simplify(f*xx^2/(6*EI)*(3*l-xx)+m*xx^2/(2*EI),10);
equation.e=matlabFunction(e,'Vars',[x y theta l EI t]);
equation.g=matlabFunction(g,'Vars',[x y theta l EI t]);
equation.h=matlabFunction(h,'Vars',[x y theta l EI t]);
equation.stress=matlabFunction(stress,'Vars',[x y theta t E xx l]);
equation.yPos=matlabFunction(yPos,'Vars',[x y theta l EI xx]);
save('LinearEquations.mat','-struct','equation');