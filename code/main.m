%%
clc
clear
warning('off','Control:analysis:LsimStartTime')
%% formation parameters
Adj = ;
L = ;
for i = 
    L(i,i) = sum(abs(Adj(i,:)));
end
Sigma = diag([ones(),-ones()]);
% test = -Sigma;
M = diag([]);
L_wavy = Sigma*L*Sigma;
%% agent parameters
M0 = diag();
A = [zeros(), eye()];
temp = [zeros(), -inv(M0)*PSI];
A = 
B = 
C = 
D =  
S = 
E = inv(B'*B)*B'*D;


%% method parametres
temp = eye();
setlmis([])
Gamma = lmivar();
lmiterm([1 1 1 Gamma],1,-L_wavy,'s')
lmis1 = getlmis;
[tmin, xfeas] = feasp(lmis1);
Gamma = dec2mat(lmis1, xfeas, Gamma);

%% method parametres
L_arc = Gamma*L_wavy + L_wavy'*Gamma;
eig_Larc = eig(L_arc)



%% method parametres
hurwitch = 
temp_block = 
temp = [eye(3);eye(3);temp_block];
F = 

%% method parametres
delta =;
temp = -B*B';
setlmis([])
P_inv = lmivar();
lmiterm([1 1 1 P_inv], A, 1, 's')
lmiterm([1 1 1 0], temp)
lmiterm([1 1 1 P_inv], delta,1)
lmiterm([2 1 1 P_inv], -1, 1)

lmis1 = getlmis;

% solve
[tmin, xfeas] = feasp(lmis1);
P_inv = dec2mat(lmis1, xfeas, P_inv)
eig_P_inv = eig(P_inv)
P = inv(P_inv)
P = P*1e-1
eig_P = eig(P)
term = P*A + A'*P - P*B*B'*P + delta*eye(6)
eig_term = eig(P*A + A'*P - P*B*B'*P + delta*eye(6))
K = -B'*P

%% method parametres
m1 = ;
omega_hat = ;
B_wavy = pinv(B);
temp =[];
B_hat = [temp,zeros()];   

%% define system dynamics
dyna_leader = ss(A,zeros(),C,zeros());
expdyna_follower = ss(A_,B_,C_,zeros());
expdyna_eso = ss(A_,eye(),C_,zeros());
dyna_disturbance = ss(S,zeros(),eye(),zeros(=));
%% initialization
x00 = [1;2;3;1;1;-1];
x0_mtx = x00;
x10 = x00 + h(1,0) + randn(6,1)*0.01; x20 = x00 + h(2,0) + randn(6,1)*0.01;
x30 = x00 + h(3,0) + randn(6,1)*0.01; x40 = x00 + h(4,0) + randn(6,1)*0.01;
x50 = -x00 + h(5,0) + randn(6,1)*0.01; x60 = -x00 + h(6,0) + randn(6,1)*0.01;
x70 = -x00 + h(7,0) + randn(6,1)*0.01; x80 = -x00 + h(8,0) + randn(6,1)*0.01;

d10 = [-0.1;-0.2]; d20 = [0.5;0.5]; d30 = [0.1;0.1]; d40 = [0.1;0.2]; 
d50 = [0.3;0.4]; d60 = [0.2;0.1]; d70 = [0.05;0.05]; d80 = [-0.1;0.2]; 

eta10 = [x10;d10]; eta20 = [x20;d20]; eta30 = [x30;d30]; eta40 = [x40;d40]; 
eta50 = [x50;d50]; eta60 = [x60;d60]; eta70 = [x70;d70]; eta80 = [x80;d80]; 

eta1_mtx = eta10; eta2_mtx = eta20; eta3_mtx = eta30; eta4_mtx = eta40; 
eta5_mtx = eta50; eta6_mtx = eta60; eta7_mtx = eta70; eta8_mtx = eta80; 

% estimated agent states init
x10_eso = [1.1;2.2;3.3;1;1;-1]; x20_eso = [4.4;5.5;6.6;1;1;-1]; x30_eso = [7.7;8.8;9.9;1;1;-1]; x40_eso = [-0.9;-1;-2;1;1;-1]; 
x50_eso = [-4.4;-5.5;-5.6;1;1;-1]; x60_eso = [-7.7;-8.8;-9.9;1;1;-1]; x70_eso = [-1.2;1.2;2.1;1;1;-1]; x80_eso = [1.2;-2.1;3.2;1;1;-1]; 

d10_eso = [-0.3;-0.4]; d20_eso = [0.55;0.55]; d30_eso = [0.12;0.3]; d40_eso = [0.5;0.5]; 
d50_eso = [0.39;0.49]; d60_eso = [0.82;0.81]; d70_eso = [0.55;0.55]; d80_eso = [-0.31;0.22]; 

eta10_eso = [x10_eso;d10_eso];  eta20_eso = [x20_eso;d20_eso];  eta30_eso = [x30_eso;d30_eso]; eta40_eso = [x40_eso;d40_eso];  
eta50_eso = [x50_eso;d50_eso];  eta60_eso = [x60_eso;d60_eso];  eta70_eso = [x70_eso;d70_eso];  eta80_eso = [x80_eso;d80_eso];  

eta1_eso_mtx = eta10_eso; eta2_eso_mtx = eta20_eso; eta3_eso_mtx = eta30_eso; eta4_eso_mtx = eta40_eso; 
eta5_eso_mtx = eta50_eso; eta6_eso_mtx = eta60_eso; eta7_eso_mtx = eta70_eso; eta8_eso_mtx = eta80_eso; 


tk1_collection = 0; tk2_collection = 0; tk3_collection = 0; tk4_collection = 0; 
tk5_collection = 0; tk6_collection = 0; tk7_collection = 0; tk8_collection = 0; 


x1_eso_tk = x10_eso; x2_eso_tk = x20_eso; x3_eso_tk = x30_eso; x4_eso_tk = x40_eso; 
x5_eso_tk = x50_eso; x6_eso_tk = x60_eso; x7_eso_tk = x70_eso; x8_eso_tk = x80_eso; 


% hi fcn
h1_mtx = [];
h2_mtx = [];
h3_mtx = [];
h4_mtx = [];
h5_mtx = [];
h6_mtx = [];
h7_mtx = [];
h8_mtx = [];


%%
time = 80;
dt = 0.01;
for t = 0:dt:time
%{当前时刻为t，t时刻所有状态（agent状态和eso状态）已知，先计算控制量u(t)，再更新t+ΔT时刻状态，最后判断事件触发条件是否满足，如果满足，更新触发时刻为t时刻，更新x_eso(tk)为x_eso(t)
	%leader状态更新
	x0_current = x0_mtx(:,end);
	x0_update = expm(A*(t+dt))*x00;
	x0_mtx = [x0_mtx,x0_update];

	%各follower更新当前时刻状态递推变量
	tk1 = tk1_collection(end);
	x1_wavy = expm(A*(t-tk1))*x1_eso_tk;

	tk2 = tk2_collection(end);
	x2_wavy = expm(A*(t-tk2))*x2_eso_tk;

	tk3 = tk3_collection(end);
	x3_wavy = expm(A*(t-tk3))*x3_eso_tk;

	tk4 = tk4_collection(end);
	x4_wavy = expm(A*(t-tk4))*x4_eso_tk;

	tk5 = tk5_collection(end);
	x5_wavy = expm(A*(t-tk5))*x5_eso_tk;

	tk6 = tk6_collection(end);
	x6_wavy = expm(A*(t-tk6))*x6_eso_tk;

	tk7 = tk7_collection(end);
	x7_wavy = expm(A*(t-tk7))*x7_eso_tk;

	tk8 = tk8_collection(end);
	x8_wavy = expm(A*(t-tk8))*x8_eso_tk;

    xi_wavy_collection = [x1_wavy,x2_wavy,x3_wavy,x4_wavy,x5_wavy,x6_wavy,x7_wavy,x8_wavy];

	%①计算follower 1控制量u(t)
	i = 1;
	ksi_eso = ;
	ai0 = M(i,i);
	xi_wavy = xi_wavy_collection(:,i);
	sigma_i = Sigma(i,i);
	for j = 
		aij = Adj(i,j);
		if aij == 0
			continue
		end
		xj_wavy = xi_wavy_collection(:,j);
		ksi_temp = abs(aij)*(xi_wavy-h(i,t)-sign(aij)*(xj_wavy - h(j,t))) + abs(ai0)*(xi_wavy - h(i,t) - sigma_i*x0_current);
		ksi_eso = ksi_eso + ksi_temp;
	end
	Hi = B_wavy*(dh(i,t)-A*h(i,t));
	etai_eso = eta1_eso_mtx(:,end);
	di_eso = etai_eso();
	ui = c*K*ksi_eso - E*di_eso + Hi;
	
	%②计算下一采样时刻t+dt follower1的扩展状态
	eta1_current = eta1_mtx(:,end);
	y1_current = C_*eta1_current;
	t_lsim = t:dt:(t+dt);
	u_lsim = [ui';zeros()];
	[~,tOut,eta_lsim] = lsim(expdyna_follower,u_lsim,t_lsim,eta1_current);
	eta1_update = eta_lsim(end,:)';
	eta1_mtx = [eta1_mtx,eta1_update];

	%③计算下一采样时刻t+dt follower1的扩展状态估计值
	eta1_eso_current = eta1_eso_mtx(:,end);
	ui_eso = B_*ui + F*(y1_current - C_*eta1_eso_current); 
	u_lsim = [ui_eso';zeros()];
	[~,~,eta_eso_lsim] = lsim(expdyna_eso,u_lsim,t_lsim,eta1_eso_current);
	eta1_eso_update = eta_eso_lsim(end,:)';
	eta1_eso_mtx = [eta1_eso_mtx,eta1_eso_update];

	%④判断follower1是否事件触发
	xi_eso = etai_eso();
	ei = xi_wavy - xi_eso;
	ei_wavy = sigma_i*ei;
	ksi_wavy = sigma_i*ksi_eso;
	fi_et = f(omega_hat,m1,m2,m3,ei_wavy,ksi_wavy);
	if fi_et >= 0
		%事件触发条件满足，更新触发时刻和触发时刻状态估计值
		tk1_collection = 
		x1_eso_tk = 
    end
    
    %①计算follower 2控制量u(t)
	i = 2;
	ksi_eso = zeros();
	ai0 = M(i,i);
	xi_wavy = xi_wavy_collection(:,i);
	sigma_i = Sigma(i,i);
	for j = 1:8 
		aij = Adj(i,j);
		if aij == 0
			continue
		end
		xj_wavy = xi_wavy_collection(:,j);
		ksi_temp = abs(aij)*(xi_wavy-h(i,t)-sign(aij)*(xj_wavy - h(j,t))) + abs(ai0)*(xi_wavy - h(i,t) - sigma_i*x0_current);
		ksi_eso = ksi_eso + ksi_temp;
	end
	Hi = B_wavy*(dh(i,t)-A*h(i,t));
	etai_eso = eta2_eso_mtx(:,end);
	di_eso = etai_eso(); 
	ui = c*K*ksi_eso - E*di_eso + Hi;
    
    %②计算下一采样时刻t+dt follower2的扩展状态
	eta2_current = eta2_mtx(:,end);
	y2_current = C_*eta2_current;
	t_lsim = t:dt:(t+dt);
	u_lsim = [ui';zeros()]; 
	[~,tOut,eta_lsim] = sim(expdyna_follower,u_lsim,t_lsim,eta2_current);
	eta2_update = eta_lsim(end,:)';
	eta2_mtx = [eta2_mtx,eta2_update];
    
    %③计算下一采样时刻t+dt follower2的扩展状态估计值
	eta2_eso_current = eta2_eso_mtx(:,end);
	ui_eso = B_*ui + F*(y2_current - C_*eta2_eso_current); 
	u_lsim = [ui_eso';zeros()];
	[~,~,eta_eso_lsim] = lsim(expdyna_eso,u_lsim,t_lsim,eta2_eso_current);
	eta2_eso_update = eta_eso_lsim(end,:)';
	eta2_eso_mtx = [eta2_eso_mtx,eta2_eso_update];
    
    %④判断follower2是否事件触发
	xi_eso = etai_eso();
	ei = xi_wavy - xi_eso;
	ei_wavy = sigma_i*ei;
	ksi_wavy = sigma_i*ksi_eso;
	fi_et = f(omega_hat,m1,m2,m3,ei_wavy,ksi_wavy);
    if fi_et >= 0
        %事件触发条件满足，更新触发时刻和触发时刻状态估计值
        tk2_collection = [tk2_collection,t];
        x2_eso_tk = xi_eso;
    end
    
    
    %①计算follower 3控制量u(t)
	i = 3;
	ksi_eso = zeros();
	ai0 = M(i,i);
	xi_wavy = xi_wavy_collection(:,i);
	sigma_i = Sigma(i,i);
	for j = 
		aij = Adj(i,j);
		if aij == 0
			continue
		end
		xj_wavy = xi_wavy_collection(:,j);
		ksi_temp = abs(aij)*(xi_wavy-h(i,t)-sign(aij)*(xj_wavy - h(j,t))) + abs(ai0)*(xi_wavy - h(i,t) - sigma_i*x0_current);
		ksi_eso = ksi_eso + ksi_temp;
	end
	Hi = B_wavy*(dh(i,t)-A*h(i,t));
	etai_eso = eta3_eso_mtx(:,end);
	di_eso = etai_eso();
	ui = c*K*ksi_eso - E*di_eso + Hi;
	
	%②计算下一采样时刻t+dt follower3的扩展状态
	eta3_current = eta3_mtx(:,end);
	y3_current = C_*eta3_current;
	t_lsim = t:dt:(t+dt);
	u_lsim = [ui';zeros()];
	[~,tOut,eta_lsim] = lsim(expdyna_follower,u_lsim,t_lsim,eta3_current);
	eta3_update = eta_lsim(end,:)';
	eta3_mtx = [eta3_mtx,eta3_update];

	%③计算下一采样时刻t+dt follower3的扩展状态估计值
	eta3_eso_current = eta3_eso_mtx(:,end);
	ui_eso = B_*ui + F*(y3_current - C_*eta3_eso_current); 
	u_lsim = [ui_eso';zeros()];
	[~,~,eta_eso_lsim] = lsim(expdyna_eso,u_lsim,t_lsim,eta3_eso_current);
	eta3_eso_update = eta_eso_lsim(end,:)';
	eta3_eso_mtx = [eta3_eso_mtx,eta3_eso_update];

	%④判断follower3是否事件触发
	xi_eso = etai_eso();
	ei = xi_wavy - xi_eso;
	ei_wavy = sigma_i*ei;
	ksi_wavy = sigma_i*ksi_eso;
	fi_et = f(omega_hat,m1,m2,m3,ei_wavy,ksi_wavy);
	if fi_et >= 0
		%事件触发条件满足，更新触发时刻和触发时刻状态估计值
		tk3_collection = [tk3_collection,t];
		x3_eso_tk = xi_eso;
    end
    
    
    
    %①计算follower 4控制量u(t)
	i = 4;
	ksi_eso = zeros(i);
	ai0 = M(i,i);
	xi_wavy = xi_wavy_collection(:,i);
	sigma_i = Sigma(i,i);
	for j = 
		aij = Adj(i,j);
		if aij == 0
			continue
		end
		xj_wavy = xi_wavy_collection(:,j);
		ksi_temp = abs(aij)*(xi_wavy-h(i,t)-sign(aij)*(xj_wavy - h(j,t))) + abs(ai0)*(xi_wavy - h(i,t) - sigma_i*x0_current);
		ksi_eso = ksi_eso + ksi_temp;
	end
	Hi = B_wavy*(dh(i,t)-A*h(i,t));
	etai_eso = eta4_eso_mtx(:,end);
	di_eso = etai_eso();
	ui = c*K*ksi_eso - E*di_eso + Hi;
	
	%②计算下一采样时刻t+dt follower4的扩展状态
	eta4_current = eta4_mtx(:,end);
	y4_current = C_*eta4_current;
	t_lsim = t:dt:(t+dt);
	u_lsim = [ui';zeros()];
	[~,tOut,eta_lsim] = lsim(expdyna_follower,u_lsim,t_lsim,eta4_current);
	eta4_update = eta_lsim(end,:)';
	eta4_mtx = [eta4_mtx,eta4_update];

	%③计算下一采样时刻t+dt follower4的扩展状态估计值
	eta4_eso_current = eta4_eso_mtx(:,end);
	ui_eso = B_*ui + F*(y4_current - C_*eta4_eso_current); 
	u_lsim = [ui_eso';zeros()];
	[~,~,eta_eso_lsim] = lsim(expdyna_eso,u_lsim,t_lsim,eta4_eso_current);
	eta4_eso_update = eta_eso_lsim(end,:)';
	eta4_eso_mtx = [eta4_eso_mtx,eta4_eso_update];

	%④判断follower4是否事件触发
	xi_eso = etai_eso();
	ei = xi_wavy - xi_eso;
	ei_wavy = sigma_i*ei;
	ksi_wavy = sigma_i*ksi_eso;
	fi_et = f(omega_hat,m1,m2,m3,ei_wavy,ksi_wavy);
    if fi_et >= 0
        %事件触发条件满足，更新触发时刻和触发时刻状态估计值
        tk4_collection = [tk4_collection,t];
        x4_eso_tk = xi_eso;
    end
    
    
    %①计算follower 5控制量u(t)
	i = 5;
	ksi_eso = zeros();
	ai0 = M(i,i);
	xi_wavy = xi_wavy_collection(:,i);
	sigma_i = Sigma(i,i);
	for j =
		aij = Adj(i,j);
		if aij == 0
			continue
		end
		xj_wavy = xi_wavy_collection(:,j);
		ksi_temp = abs(aij)*(xi_wavy-h(i,t)-sign(aij)*(xj_wavy - h(j,t))) + abs(ai0)*(xi_wavy - h(i,t) - sigma_i*x0_current);
		ksi_eso = ksi_eso + ksi_temp;
	end
	Hi = B_wavy*(dh(i,t)-A*h(i,t));
	etai_eso = eta5_eso_mtx(:,end);
	di_eso = etai_eso);
	ui = c*K*ksi_eso - E*di_eso + Hi;
	
	%②计算下一采样时刻t+dt follower5的扩展状态
	eta5_current = eta5_mtx(:,end);
	y5_current = C_*eta5_current;
	t_lsim = t:dt:(t+dt);
	u_lsim = [ui';zeros()];
	[~,tOut,eta_lsim] = lsim(expdyna_follower,u_lsim,t_lsim,eta5_current);
	eta5_update = eta_lsim(end,:)';
	eta5_mtx = [eta5_mtx,eta5_update];

	%③计算下一采样时刻t+dt follower5的扩展状态估计值
	eta5_eso_current = eta5_eso_mtx(:,end);
	ui_eso = B_*ui + F*(y5_current - C_*eta5_eso_current); 
	u_lsim = [ui_eso';zeros()];
	[~,~,eta_eso_lsim] = lsim(expdyna_eso,u_lsim,t_lsim,eta5_eso_current);
	eta5_eso_update = eta_eso_lsim(end,:)';
	eta5_eso_mtx = [eta5_eso_mtx,eta5_eso_update];

	%④判断follower5是否事件触发
	xi_eso = etai_eso();
	ei = xi_wavy - xi_eso;
	ei_wavy = sigma_i*ei;
	ksi_wavy = sigma_i*ksi_eso;
	fi_et = f(omega_hat,m1,m2,m3,ei_wavy,ksi_wavy);
    if fi_et >= 0
        %事件触发条件满足，更新触发时刻和触发时刻状态估计值
        tk5_collection = [tk5_collection,t];
        x5_eso_tk = xi_eso;
    end
    
    %①计算follower 6控制量u(t)
	i = 6;
	ksi_eso = zeros();
	ai0 = M(i,i);
	xi_wavy = xi_wavy_collection(:,i);
	sigma_i = Sigma(i,i);
	for j = 
		aij = Adj(i,j);
		if aij == 0
			continue
		end
		xj_wavy = xi_wavy_collection(:,j);
		ksi_temp = abs(aij)*(xi_wavy-h(i,t)-sign(aij)*(xj_wavy - h(j,t))) + abs(ai0)*(xi_wavy - h(i,t) - sigma_i*x0_current);
		ksi_eso = ksi_eso + ksi_temp;
	end
	Hi = B_wavy*(dh(i,t)-A*h(i,t));
	etai_eso = eta6_eso_mtx(:,end)
	di_eso = etai_eso();
	ui = c*K*ksi_eso - E*di_eso + Hi;
	
	%②计算下一采样时刻t+dt follower6的扩展状态
	eta6_current = eta6_mtx(:,end);
	y6_current = C_*eta6_current;
	t_lsim = t:dt:(t+dt);
	u_lsim = [ui';zeros()];
	[~,tOut,eta_lsim] = lsim(expdyna_follower,u_lsim,t_lsim,eta6_current);
	eta6_update = eta_lsim(end,:)';
	eta6_mtx = [eta6_mtx,eta6_update];

	%③计算下一采样时刻t+dt follower6的扩展状态估计值
	eta6_eso_current = eta6_eso_mtx(:,end);
	ui_eso = B_*ui + F*(y6_current - C_*eta6_eso_current); 
	u_lsim = [ui_eso';zeros()]; 
	[~,~,eta_eso_lsim] = lsim(expdyna_eso,u_lsim,t_lsim,eta6_eso_current);
	eta6_eso_update = eta_eso_lsim(end,:)';
	eta6_eso_mtx = [eta6_eso_mtx,eta6_eso_update];

	%④判断follower6是否事件触发
	xi_eso = etai_eso(); 
	ei = xi_wavy - xi_eso;
	ei_wavy = sigma_i*ei;
	ksi_wavy = sigma_i*ksi_eso;
	fi_et = f(omega_hat,m1,m2,m3,ei_wavy,ksi_wavy);
	if fi_et >= 0
		%事件触发条件满足，更新触发时刻和触发时刻状态估计值
		tk6_collection = [tk6_collection,t];
		x6_eso_tk = xi_eso;	
    end
    
    
    %①计算follower 7控制量u(t)
	i = 7;
	ksi_eso = zeros(); 
	ai0 = M(i,i);
	xi_wavy = xi_wavy_collection(:,i);
	sigma_i = Sigma(i,i);
	for j = 1:8 
		aij = Adj(i,j);
		if aij == 0
			continue
		end
		xj_wavy = xi_wavy_collection(:,j);
		ksi_temp = abs(aij)*(xi_wavy-h(i,t)-sign(aij)*(xj_wavy - h(j,t))) + abs(ai0)*(xi_wavy - h(i,t) - sigma_i*x0_current);
		ksi_eso = ksi_eso + ksi_temp;
	end
	Hi = B_wavy*(dh(i,t)-A*h(i,t));
	etai_eso = eta7_eso_mtx(:,end);
	di_eso = etai_eso(7:8,1);
	ui = c*K*ksi_eso - E*di_eso + Hi;
	
	%②计算下一采样时刻t+dt follower1的扩展状态
	eta7_current = eta7_mtx(:,end);
	y7_current = C_*eta7_current;
	t_lsim = t:dt:(t+dt);
	u_lsim = [ui';zeros()]; 
	[~,tOut,eta_lsim] = lsim(expdyna_follower,u_lsim,t_lsim,eta7_current);
	eta7_update = eta_lsim(end,:)';
	eta7_mtx = [eta7_mtx,eta7_update];

	%③计算下一采样时刻t+dt follower7的扩展状态估计值
	eta7_eso_current = eta7_eso_mtx(:,end);
	ui_eso = B_*ui + F*(y7_current - C_*eta7_eso_current); 
	u_lsim = [ui_eso';zeros()]; 
	[~,~,eta_eso_lsim] = lsim(expdyna_eso,u_lsim,t_lsim,eta7_eso_current);
	eta7_eso_update = eta_eso_lsim(end,:)';
	eta7_eso_mtx = [eta7_eso_mtx,eta7_eso_update];

	%④判断follower7是否事件触发
	xi_eso = etai_eso();
	ei = xi_wavy - xi_eso;
	ei_wavy = sigma_i*ei;
	ksi_wavy = sigma_i*ksi_eso;
	fi_et = f(omega_hat,m1,m2,m3,ei_wavy,ksi_wavy);
	if fi_et >= 0
		%事件触发条件满足，更新触发时刻和触发时刻状态估计值
		tk7_collection = [tk7_collection,t];
		x7_eso_tk = xi_eso;	
    end
    
    
    %①计算follower 8控制量u(t)
	i = 8;
	ksi_eso = zeros();
	ai0 = M(i,i);
	xi_wavy = xi_wavy_collection(:,i);
	sigma_i = Sigma(i,i);
	for j =
		aij = Adj(i,j);
		if aij == 0
			continue
		end
		xj_wavy = xi_wavy_collection(:,j);
		ksi_temp = abs(aij)*(xi_wavy-h(i,t)-sign(aij)*(xj_wavy - h(j,t))) + abs(ai0)*(xi_wavy - h(i,t) - sigma_i*x0_current);
		ksi_eso = ksi_eso + ksi_temp;
	end
	Hi = B_wavy*(dh(i,t)-A*h(i,t));
	etai_eso = eta8_eso_mtx(:,end);
	di_eso = etai_eso(); 
	ui = c*K*ksi_eso - E*di_eso + Hi;
	
	%②计算下一采样时刻t+dt follower8的扩展状态
	eta8_current = eta8_mtx(:,end);
	y8_current = C_*eta8_current;
	t_lsim = t:dt:(t+dt);
	u_lsim = [ui';zeros()];
	[~,tOut,eta_lsim] = lsim(expdyna_follower,u_lsim,t_lsim,eta8_current);
	eta8_update = eta_lsim(end,:)';
	eta8_mtx = [eta8_mtx,eta8_update];

	%③计算下一采样时刻t+dt follower8的扩展状态估计值
	eta8_eso_current = eta8_eso_mtx(:,end);
	ui_eso = B_*ui + F*(y8_current - C_*eta8_eso_current); 
	u_lsim = [ui_eso';zeros()]; 
	[~,~,eta_eso_lsim] = lsim(expdyna_eso,u_lsim,t_lsim,eta8_eso_current);
	eta8_eso_update = eta_eso_lsim(end,:)';
	eta8_eso_mtx = [eta8_eso_mtx,eta8_eso_update];

	%④判断follower8是否事件触发
	xi_eso = etai_eso();
	ei = xi_wavy - xi_eso;
	ei_wavy = sigma_i*ei;
	ksi_wavy = sigma_i*ksi_eso;
	fi_et = f(omega_hat,m1,m2,m3,ei_wavy,ksi_wavy);
	if fi_et >= 0
		%事件触发条件满足，更新触发时刻和触发时刻状态估计值
		tk8_collection = [tk8_collection,t];
		x8_eso_tk = xi_eso;
    end
    
    
    %将hi(t)存入hi_mtx中
    h1_mtx = [h1_mtx,h(1,t)];
    h2_mtx = [h2_mtx,h(2,t)];
    h3_mtx = [h3_mtx,h(3,t)];
    h4_mtx = [h4_mtx,h(4,t)];
    h5_mtx = [h5_mtx,h(5,t)];
    h6_mtx = [h6_mtx,h(6,t)];
    h7_mtx = [h7_mtx,h(7,t)];
    h8_mtx = [h8_mtx,h(8,t)];
end

%% Process simulation results and plots







