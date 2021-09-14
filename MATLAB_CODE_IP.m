m=0.2;
M=0.5;
g=9.8;
b=0.1;
l=0.3;
I=0.006;
b=I*(M+m)+m*M*l^2;
A=[0,1,0,0 ;g*(M+m)*m*l/b,0,0,-m*b*l/b ;0,0, 0,1;m^2*l^2*g/b,0,0,-b*(I+m*l^2)/b  ];
B=[0;m*l/b;0;(I+m*l^2)/b];

C=[1 0 0 0;0 0 1 0];
eig(A)
Controllabilty=ctrb(A,B);
observbility=obsv(A,C);
rank(Controllabilty);
rank(observbility);
sys=ss(A,B,C,[0 ;0]);
sys_open_loop=tf(sys)
figure(1);
subplot(1,2,1);
impulse(sys_open_loop)
legend();
subplot(1,2,2);
step(sys_open_loop);
legend()

%pole placement Method
Ts=1;
zeta=0.1;
wn=4/(zeta*Ts);
desired_poles=[[roots([1 2*zeta*wn wn*wn])]', -5, -6];
K=place(A,B,desired_poles);     % Gain matrix 
new_A=A-B*K;
roots([1 2*zeta*wn wn*wn])
pole_placement_eigen=eig(new_A) ;
SYS1=ss(new_A,B,C,0) ;

Ts=1;
zeta=0.3;
wn=4/(zeta*Ts);
desired_poles=[[roots([1 2*zeta*wn wn*wn])]', -5, -6];
roots([1 2*zeta*wn wn*wn])
K=place(A,B,desired_poles);     % Gain matrix 
new_A=A-B*K;
pole_placement_eigen=eig(new_A) ;
SYS2=ss(new_A,B,C,0) ;

Ts=1;
zeta=0.6;
wn=4/(zeta*Ts);
desired_poles=[[roots([1 2*zeta*wn wn*wn])]', -5, -6];
roots([1 2*zeta*wn wn*wn])
K=place(A,B,desired_poles);     % Gain matrix 
new_A=A-B*K;
pole_placement_eigen=eig(new_A) ;
SYS3=ss(new_A,B,C,0) ;

Ts=1;
zeta=0.8;
wn=4/(zeta*Ts);
desired_poles=[[roots([1 2*zeta*wn wn*wn])]', -5, -6];
roots([1 2*zeta*wn wn*wn])
K=place(A,B,desired_poles);     % Gain matrix 
new_A=A-B*K;
pole_placement_eigen=eig(new_A) ;
SYS4=ss(new_A,B,C,0) ;

figure(2),
subplot(1,2,1);
impulseplot(SYS1,SYS2,SYS3,SYS4)
legend()
subplot(1,2,2);
stepplot(SYS1,SYS2,SYS3,SYS4);
legend()



%Linear Quadratic regulator Metod
Q= C'*C;
R = 1; 
K = lqr(A,B,Q,R) ;
Anew = [(A-B*K)]; 
Bnew = [B]; 
Cnew = [C]; 
Dnew = [0]; 
sys_lqr = ss(Anew,Bnew,Cnew,Dnew) ;
figure(3);
subplot(1,2,1);
impulse(sys_lqr) ;
legend();
subplot(1,2,2)
step(sys_lqr) ;
legend()
eig(Anew)

%pid control method
%To get the values of Kp,Kd,Ki use matlab pid tuner app if mathematical model is avaliable. Otherwise the are many method to tune pid in literature one can go 
% through it (i.e., Trial and error method,Zeigler-Nichols Method).



