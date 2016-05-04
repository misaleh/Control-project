% Name: Mostafa Ahmed Ibrahim Mohamed Saleh
% Section: 6    
% Seat Number: 228
clc;clear all;close all;
%% Draft 
%
%System eqns:
%p(k+1) = p(k) + T*v(k)
%v(k+1) = (1-alpha)*v(k) +(T/m)f(k)
%T=0.1
%alpha = 0.1
%m=1

%state space eqns:
%x(k+1) = A*x(k) + Bu(k)
%y(k)  = C*x(k) +D*u(k)
%u(k) = [fx ; fy]
%x(k) = [px ; py; vx ; vy]
%y(k) = [px ; py]

%For this system
%A B C D can be found by inspection 
%as A maps prevoius x to next x 
%x = [px ; py ; vx ; vy]
%from equation p(k+1) = p(k) + T*v(k) the next position is function of the
%current postition and velocity scaled by 0.1 so the first to raw which maps
%to px and py are [1 0 0.1 0;0 1 0 0.1]
%similary from v(k+1) = (1-alpha)*v(k) +(T/m)f(k) , we can see that the
%velocity depends only on pervious velocity scaled by 0.9 
%so the last two raws will be [0 0 0.9 0;0 0 0 0.9]

%similary for B as B relates the input force to x which contains positon and velocity 
 %from v(k+1) = (1-alpha)*v(k) +(T/m)f(k)
 %next velocity depends on the force by scale T/m = 0.1 




%% System charactristecs 
%explained in draft
A = [1 0 0.1 0 ; 0 1 0 0.1; 0 0 0.9 0;0 0 0 0.9] %4*4
B = [0 0; 0 0; 0.1 0; 0 0.1 ] %4*2
C = [1 0 0 0; 0 1 0 0 ];%2*4
D = [0 0 ; 0 0]; %2*2
%% Constarins 
X10 = [2 ;2]; %first destination at k = 10
X30 = [-2;3 ];  %second destination at k = 30
X40 = [4;-3 ];  %third destination at k = 40
X80 = [-4;-2];  %fourth destination at k = 80
Xdes = [X80 ; X40 ; X30 ; X10];
%% calculate controllaiblty matricies 
for k = [10 30 40 80]    %time at each constrain
    C_100 = B; %intiliaze controllaiblty matrix
    for m = 1:k-1
           C_100 = [C_100 , (A^m)*B];
    end
    if(k == 10)
        C_10 = C*C_100; 
    elseif(k == 30)
         C_30 = C*C_100;
    elseif(k == 40)
        C_40 = C*C_100;
    elseif(k == 80)
        C_80 = C*C_100;
    end
end
diff_80_40 = length(C_80) - length(C_40);  %differnce in length to put zeros 
diff_80_30 = length(C_80) - length( C_30);   %differnce in length to put zeros 
diff_80_10 = length(C_80) - length(C_10);   %differnce in length to put zeros 
C_a_tot = [  zeros(2,40) C_80 ; zeros(2,diff_80_40 +40) C_40; zeros(2,diff_80_30 +40)  C_30 ;zeros(2,diff_80_10 +40) C_10 ];
%concatenate  the marices putting zeros at the start of shorter ones
%Ck = [0000.. 00000 ;C80 ;  0000...00 C40 ; 000. 00 C30;000. 00000 C10] 

%% calculate Umin 
%Xdes = [ x80 ; x40 ; x30 ; x10]
%C = [C80 ; 0000..0C40 ...]
%Required to find U

Umin = C_a_tot'*(inv(C_a_tot*C_a_tot'))*Xdes;
Emin_a =Umin.'*Umin  %minimum energy
%% finding postition of x and y
positionx =0;
positiony=0;
x = [0;0 ;0 ;0]; %intial postition
for i = 0:2:196   
    positionx(i/2+1) = x(1); %postitionx(0) , postitionx(1) ,postitionx(2) ...  
    positiony(i/2+1) = x(2); %similarly
    U = [ Umin(end -i -1) ; Umin(end-i)];%[fx(i); fy(i)]
    x = A*x + B*U;     %compute state vector
end
%% Plotting
time = 1:99;
fx = Umin(end-1:-2:1); % at  1 , 3 ,5 ... since Umin [fx(80) fy(80) ...fx(0) fy(0) ]
fy = Umin(end:-2:1);  % at 2 ,4 ,6 ...
figure('name','force(a)')
subplot(2,1,1)
plot(fx)
xlabel('time')
ylabel('fx')
subplot(2,1,2)
plot(fy)
xlabel('time')
ylabel('fy')
figure('name','scatter(a)')
scatter(positionx,positiony,time)

%% part b

%Umin = Ck'*inv*(CkCk')*(Xdes-A^k*x(0))
X_init = [0;0 ;-4 ;-2];
Ak = A^k;
X_0 = Ak*X_init;
Uminb = C_a_tot'*(inv(C_a_tot*C_a_tot'))*(Xdes - [ X_0([1 2]);X_0([1 2]);X_0([1 2]);X_0([1 2])]); 
Emin_b = Uminb.'*Uminb %calculate minmumum energy
%% finding postition of x and y(partb)
positionx =0;
positiony=0;
x = [-4;-2 ;0 ;0];
for i = 0:2:196  
    positionx(i/2+1) = x(1) ;
    positiony(i/2+1) = x(2);
    U = [ Uminb(end -i -1) ; Uminb(end-i)];%[fx(i); fy(i)]
    x = A*x + B*U;
end
%% plotting partb
figure('name','force(b)')
fx = Uminb(end-1:-2:1); % at  1 , 3 ,5 ... since Umin [fx(80) fy(80) ...fx(0) fy(0) ]
fy = Uminb(end:-2:1);  % at 2 ,4 ,6 ...
subplot(2,1,1)
plot(fx)
xlabel('time')
ylabel('fx')
subplot(2,1,2)
plot(fy)
xlabel('time')
ylabel('fy')
figure('name','scatter(b)')
scatter(positionx,positiony,time)