%--- 사전에 정의된 변수들 ---
% HTM        : 4×4 현재 end-effector 변환 행렬
% Jaco       : 6×7 Jacobian 행렬 (HTM에 대응)
% q_0          : 7×1 현재 관절각 (q_0_k)
% q_0_min, q_0_max     : 7×1 관절각 최소·최대
% q_0d_min, q_0d_max   : 7×1 관절속도 최소·최대
% dt                : 제어 주기 (예: 0.01 [s])
% target_HTM        : 4×4 목표 end-effector 변환 행렬

%% i) Basic Coefficient Setting (KUKA - Version)

dt = 0.01; % 제어주기, unit: sec

% Kinematics parameter
global d1 d3 d5 d7;

d1 = 340; % Unit: mm
d3 = 400; % Unit: mm
d5 = 400; % Unit: mm
d7 = 126; % Unit: mm

% QP - 가중치 파라미터
omega_p = 1.0;    % joint-change 최소화 가중치
alpha   = 0.1;    % singularity 회피(DLS) 가중치
lambda  = 0.01;   % DLS 댐핑 계수
Kp_pos  = 1.0;    % 위치 제어 이득
Kp_rot  = 1.0;    % 자세 제어 이득

% Constraint 파라미터
q_0_min = [-2*pi, -2*pi, -2*pi, -2*pi, -2*pi, -2*pi, -2*pi];
q_0_max = [ 2*pi,  2*pi,  2*pi,  2*pi,  2*pi,  2*pi,  2*pi];

q_0d_min = [-314, -314, -314, -314, -314, -314, -314];
q_0d_max = [ 314,  314,  314,  314,  314,  314,  314];

%% ii) Input variables

% Current joint angles (joint-change 최소화를 위해 현재값 사용)
q_0 = [0, -pi/2, 0, pi/2, 0, 0, 0];

% Target HTM 
target_HTM = KUKA_FK07(q_0(1),q_0(2),q_0(3),q_0(4),q_0(5),q_0(6),q_0(7));
target_HTM(1,4) = target_HTM(1,4) + 10;

%% 1) EE 위치·자세 오차 계산 (선형화용 목표 변위)
HTM = KUKA_FK07(q_0(1),q_0(2),q_0(3),q_0(4),q_0(5),q_0(6),q_0(7)); % Through Forward Kinematics

e_pos = target_HTM(1:3,4) - HTM(1:3,4);
R_err = target_HTM(1:3,1:3) * HTM(1:3,1:3)';  
e_rot = 0.5*[
  R_err(3,2)-R_err(2,3);
  R_err(1,3)-R_err(3,1);
  R_err(2,1)-R_err(1,2)
];
Delta_x_des = [Kp_pos*e_pos; Kp_rot*e_rot];   % 6×1

%% 2) Hessian H와 선형항 f 구성
%  (a) joint-change 최소화 항: (q−q_0)'·ω_p·I·(q−q_0)
H1 = 2*omega_p * eye(7);
f1 = -2*omega_p * q_0';

%  (b) singularity 회피 (Damped-LS) 항: q_0'·[α(J'J + λI)]·q_0
Jaco = KUKA_jacobian(q_0(1),q_0(2),q_0(3),q_0(4),q_0(5),q_0(6),q_0(7));
H2 = 2*alpha * (Jaco.'*Jaco + lambda*eye(7));
f2 = zeros(7,1);

% 최종 q_0P 비용함수:  1/2 q_0'Hq_0 + f' q_0
H = H1 + H2;
f = f1 + f2;

%% 3) Eq_0uality 제약:  J*(q_0 − q_0_k) = Delta_x_des  →  J*q_0 = Delta_x_des + J*q_0_k
Aeq_0 = Jaco;  
beq_0 = Delta_x_des + Jaco*q_0';

%% 4) Ineq_0uality 제약: joint limits & velocity limits을 bound로
% (a) 관절각 한계
lb_angle = q_0_min;
ub_angle = q_0_max;

% (b) 속도 한계:  (q_0 − q_0_k)/dt ∈ [q_0d_min, q_0d_max]
%  → q_0 ∈ [q_0_k + dt*q_0d_min,  q_0_k + dt*q_0d_max]
lb_vel = q_0 + dt*q_0d_min;
ub_vel = q_0 + dt*q_0d_max;

% element-wise로 결합
lb = max(lb_angle, lb_vel);
ub = min(ub_angle, ub_vel);

%% 5) q_0P 풀기 (Optimization Toolbox의 q_0uadprog)
opts = optimoptions('quadprog','Display','none');
[q_0_sol,~,exitflag] = quadprog( ...
    H, f, ...           % Hessian & linear term
    [], [], ...         % A, b (inequalities 없음)
    Aeq_0, beq_0, ...   % Aeq, beq (equalities)
    lb, ub, ...         % lower & upper bounds
    [], ...             % x0 (초기값, 빈칸 허용)
    opts );

if exitflag ~= 1
  warning('q_0P 풀이가 수렴하지 않았습니다 (exitflag = %d)', exitflag);
end

%% 6) 결과
% q_0_sol: 최적 관절각 (위치-레벨 q_0P 해)
% 관절속도로 변환하고 싶다면: q_0d = (q_0_sol - q_0) / dt;
disp('=== 위치-레벨 q_0P 결과 ===');
disp('q_0_new ='); disp(q_0_sol.');

disp('=== 초기 포인트 ===');
fprintf("Px: %.1f, Py: %.1f, Pz: %.1f \n",...
    HTM(1,4),HTM(2,4),HTM(3,4));

disp('=== 목표 포인트 ===');
fprintf("Px: %.1f, Py: %.1f, Pz: %.1f \n",...
    target_HTM(1,4),target_HTM(2,4),target_HTM(3,4));

disp('=== 이동된 포인트 ===');
HTM = KUKA_FK07(q_0_sol(1),q_0_sol(2),q_0_sol(3),q_0_sol(4),...
    q_0_sol(5),q_0_sol(6),q_0_sol(7));

fprintf("Px: %.1f, Py: %.1f, Pz: %.1f \n",...
    HTM(1,4),HTM(2,4),HTM(3,4));