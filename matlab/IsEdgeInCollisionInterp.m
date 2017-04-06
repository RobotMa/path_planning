function collision = IsEdgeInCollisionES_Interp(P1, P2, dt)

global  ra_s rb_s ang_s ra_o rb_o ang_o tx_o ty_o ra_f rb_f ra_e1 rb_e1 ra_e2 rb_e2 N_s N_o


collision = 0;
% dt = 1;
N_step  = floor(dt*sqrt((P1(1)-P2(1))*(P1(1)-P2(1))+(P1(2)-P2(2))*(P1(2)-P2(2))));

if N_step == 0
    return;
end

tx_step = linspace(P1(1), P2(1), N_step);
ty_step = linspace(P1(2), P2(2), N_step);
ang_f_step  = linspace(P1(3), P2(3), N_step);
ang_e1_step = linspace(P1(4), P2(4), N_step);
ang_e2_step = linspace(P1(5), P2(5), N_step);

% check if the line hits the obstacle
for i = 1:N_o
    X_o = [tx_o(i); ty_o(i)];
    A_o = diag([1/ra_o(i)^2, 1/rb_o(i)^2]);
    R_o = [cos(ang_o(i)) -sin(ang_o(i));
        sin(ang_o(i)) cos(ang_o(i))];
    for j = 1:N_step
        if  ([tx_step(j); ty_step(j)]-X_o)'*R_o*A_o*R_o'*([tx_step(j); ty_step(j)]-X_o)<=1
            collision = 1;
            return
        end
    end
end

% check if the rabbit hits the obstacle or arena, step by step
for i = 1:N_step
    [tx_e1, ty_e1, ang_e1_new, tx_e2, ty_e2, ang_e2_new] = Parameter_ears(ra_f, rb_f, ang_f_step(i), ang_e1_step(i), ang_e2_step(i));
    
    for j = 1:N_s
        if characteristicContainmentChecking2D(ra_s(j),rb_s(j),ang_s(j),0,0,ra_f,rb_f,ang_f_step(i),tx_step(i),ty_step(i)) ||...
                characteristicContainmentChecking2D(ra_s(j),rb_s(j),ang_s(j),0 ,0, ra_e1,rb_e1,ang_e1_new,tx_e1+tx_step(i),ty_e1+ty_step(i)) || ...
                characteristicContainmentChecking2D(ra_s(j),rb_s(j),ang_s(j),0 ,0, ra_e2,rb_e2,ang_e2_new,tx_e2+tx_step(i),ty_e2+ty_step(i))
            collision = 1;
            return
        end
    end
    for j = 1:N_o
        if characteristicCollisionChecking2D(ra_o(j),rb_o(j),ang_o(j),tx_o(j),ty_o(j),ra_f,rb_f,ang_f_step(i),tx_step(i),ty_step(i)) || ...
                characteristicCollisionChecking2D(ra_o(j),rb_o(j),ang_o(j),tx_o(j),ty_o(j),ra_e1,rb_e1,ang_e1_new,tx_e1+tx_step(i),ty_e1+ty_step(i)) || ...
                characteristicCollisionChecking2D(ra_o(j),rb_o(j),ang_o(j),tx_o(j),ty_o(j),ra_e2,rb_e2,ang_e2_new,tx_e2+tx_step(i),ty_e2+ty_step(i))
            collision = 1;
            return
        end
    end
end




