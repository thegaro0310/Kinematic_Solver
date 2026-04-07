syms x y theta t L p pp
fm=[12 -6;-6 4]*[(y/L);theta]+pp*[6/5 -1/10;-1/10 2/15]*[(y/L);theta]+pp^2*[-1/700 1/1400;1/1400 -11/6300]*[(y/L);theta];
eq=-(x/L)+t^2*p/(12*L^2)-1/2*[(y/L) theta]*[6/5 -1/10;-1/10 2/15]*[(y/L);theta]-p*[(y/L) theta]*[-1/700 1/1400;1/1400 -11/6300]*[(y/L);theta];
p=simplify(solve(eq==0,p),10);
e=simplify(1/2*t^2*p^2/(12*L^2)+1/2*[(y/L) theta]*[12 -6;-6 4]*[(y/L);theta]-1/2*p^2*[(y/L) theta]*[-1/700 1/1400;1/1400 -11/6300]*[(y/L);theta],10);
g=simplify(gradient(e,[x,y,theta]),10);
h=simplify(hessian(e,[x,y,theta]),10);
equation.e=matlabFunction(e,'Vars',[x y theta t L]);
equation.g=matlabFunction(g,'Vars',[x y theta t L]);
equation.h=matlabFunction(h,'Vars',[x y theta t L]);
equation.p=matlabFunction(p,'Vars',[x y theta t L]);
equation.fm=matlabFunction(fm,'Vars',[y theta pp L]);
save('BCMEquations.mat','-struct','equation');