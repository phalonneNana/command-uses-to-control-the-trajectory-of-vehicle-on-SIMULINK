function [sys,x0,str,ts] = Dynamique(t,x,u,flag)
    

switch flag


  case 0            
    [sys,x0,str,ts]=mdlInitializeSizes;
    
  case 1
    sys=mdlDerivatives(t,x,u);
    
  case 3
    sys=mdlOutputs(t,x,u);


  case { 2, 4, 9 }
    sys = [];


  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts]=mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 2;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  =[0 0]';
str = [];
ts  = [0 0];


function sys=mdlDerivatives(~,x,u)
Vx=90;
m=1719;
Iz=3300;
Cf=170550;
Cr=137844;
lf=1.195;
lr=1.513;

A=[-(Cf+Cr)/(m*Vx)  (((-Cf*lf + lr*Cr)/(m*Vx))-Vx);-((-Cf*lf + lr*Cr)/(Iz*Vx)) -(((Cf^2)*lf + (lr^2)*Cr)/(Iz*Vx))];
B=[Cf/m 0;0 lf*Cf/Iz];

ddx=A*x + B*u;

sys(1) = ddx(1);
sys(2) = ddx(2);

function sys=mdlOutputs(~,x,~)

sys(1) = x(1); %vitesse latérale
sys(2) = x(2); % vitesse angulaire de lacet

