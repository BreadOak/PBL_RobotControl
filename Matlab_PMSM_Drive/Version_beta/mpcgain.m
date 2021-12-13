function [Phi_Phi, Phi_F, Phi_R, A_e, B_e, C_e] = mpcgain(Ad, Bd, Cd, Nc, Np)

    [m1,n1] = size(Cd);
    [n1,n_in] = size(Bd);
    A_e = eye(n1+m1, n1+m1);
    A_e(1:n1,1:n1) = Ad;
    A_e(n1+1:n1+m1,1:n1) = Cd*Ad;
    B_e = zeros(n1+m1,n_in);
    B_e(1:n1,:) = Bd;
    B_e(n1+1:n1+m1,:) = Cd*Bd;
    C_e = zeros(m1,n1+m1);
    C_e(:,n1+1:n1+m1) = eye(m1,m1);

    [h_row, h_col] = size(C_e);
    [F_row, F_col] = size(C_e*A_e);
    [Phi_row, Phi_col] = size(C_e*B_e);

    n = n1+m1;
    h = zeros(h_row*Np, h_col);
    F = zeros(F_row*Np, F_col);

    h(1:h_row , :) = C_e;
    F(1:F_row , :) = C_e*A_e;
    for kk = 1:Np-1
        hk = kk*h_row;
        Fk = kk*F_row;
        h(hk+1:hk+h_row , :) = h(hk-h_row+1:hk , :)*A_e;
        F(Fk+1:Fk+F_row , :) = F(Fk-F_row+1:Fk , :)*A_e;
    end

    v = zeros(Phi_row*Np, Phi_col);
    for nn = 0:Np-1
        vn = nn*Phi_row;
        v(vn+1:vn+Phi_row, :) = h(vn+1:vn+Phi_row, :)*B_e;
    end

    Phi = zeros(Phi_row*Np, Phi_col*Nc); % declare the dimension of Phi
    Phi(: , 1:Phi_col) = v;               % first column of Phi
    for ii = 1:Nc-1
        ci = ii*Phi_col;
        ri = ii*Phi_row;
        Phi(: , ci+1:ci+Phi_col) = [zeros(ri,Phi_col); v(1:Phi_row*Np-ri , :)]; % Toeplitz matrix
    end

    Bar_Rs = zeros(h_row*Np, h_row);
    for jj = 0:Np-1
        Bj = jj*h_row;
        Bar_Rs(Bj+1:Bj+h_row , :) = eye(h_row);
    end

    Phi_Phi = Phi'*Phi;
    Phi_F = Phi'*F;
    Phi_R = Phi'*Bar_Rs;
end