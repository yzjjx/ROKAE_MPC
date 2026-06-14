function [E,H] = MPC_Matrices(A,B,Q,R,F,N)

    % A为n*n矩阵，得到n
    n = size(A,1);
    % B为n*p矩阵，得到p
    p = size(B,2);
    
    % M是两个矩阵拼起来
    M = [eye(n);zeros(N*n,n)];

    C = zeros((N+1)*n,N*p);

    tmp = eye(n);

    % 更新矩阵M与矩阵C
    for i = 1:N
        rows = i*n+(1:n);
        C(rows,:)=[tmp*B,C(rows-n,1:end-p)];
        tmp = A*tmp;
        M(rows,:) = tmp;
    end

    Q_bar = kron(eye(N),Q);
    Q_bar = blkdiag(Q_bar,F);
    R_bar = kron(eye(N),R);

    G = M' * Q_bar * M;
    E = M' * Q_bar * C;
    H = C' * Q_bar * C + R_bar;
end

