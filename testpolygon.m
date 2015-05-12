
NPol = 10;
Rvo = 1;

Rvo2 = Rvo*(1+(tan(pi/NPol)^2))^0.5;

TInner =0:2*pi/360:2*pi;
XInner = Rvo*cos(TInner);
YInner = Rvo*sin(TInner);

TOuter =0:2*pi/NPol:2*pi;
XOuter = Rvo2*cos(TOuter);
YOuter = Rvo2*sin(TOuter);

plot(XInner,YInner); hold on; grid on;
plot(XOuter,YOuter,'-*r')