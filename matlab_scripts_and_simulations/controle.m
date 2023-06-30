
s = tf('s');
%todos os dados estão em metros
a1 = 0.005;%area de secção tanque 1
a2 = 0.005;%area de secção tanque 2
g =  9.8;%gravidade
A1 = 0.4273; %Area tanque 1
A2 = 0.5184; %Area tanque 2
h1 = 0.09; %altura tanque 1 regime permanente
h2 = 0.06; %altura tanque 2 regime permanente
num = ((a1*sqrt(2*g))/(A1*A2*sqrt(h1)))+(s/A2); 
dem1 = s+(a1*sqrt(2*g))/(A1*sqrt(h1));
dem2 = s+(a2*sqrt(2*g))/A2*sqrt(h2);

G = num/(dem1*dem2);
%%
%metodo de ziegler
num = [1, 0.1727];
den = [0.5184, 0.09494, 0.0009363];
poles = roots(den);

T = mean(abs(poles));
KP = 0.60;
KI = 2.0 / T;
KD = 0.5 * T;
pid_ziegle = pid(KP, KI, KD);
%%
%metodo alocação de polos rltool
C = (14.635*(s+0.9008)*(s+0.7585))/s;
[nummerador, denominador] = tfdata(C, 'v');
b2 = nummerador(1);
b1 = nummerador(2);
b0 = nummerador(3);

a2 = denominador(1);
a1 = denominador(2);
a0 = denominador(3);

%Compute gains
kp_rltool = (a1*b1 - b0)/a1^2;
ki_rltool = b0/a1;
kd_rltool = (b0 - a1*b1 + a1^2*b2)/(a1^3);
a = a1;
pid_rltool = pid(kp_rltool, ki_rltool, kd_rltool);
%% 
%step(G)
%step(feedback(pid_rltool*G,1));
step(feedback(pid_ziegle*G,1));
%%
%teste florindo, não funciona mais...
[B,A] = tfdata(minreal(G),'v');
s = tf('s');
%%
%continuação teste do florindo...
M = B()*eye(2,2);
ts = 120;
Ed = 0.7;
Wnd = 4/(ts*Ed);
f1 = 1.2*Wnd;
Pd = conv([1 2*Ed*Wnd Wnd^2],[1 f1]);
P = [Pd(2)-A(2) Pd(2)-A(2) Pd(3)];
X = inv(M)*P'
C = X(2) + X(3)/s + X(1)*s

Lf = C*G
T = feedback(Lf,1)
damp(T),
figure, step(T)