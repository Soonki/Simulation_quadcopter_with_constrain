classdef Dynamics_constrain < handle
    
 properties(GetAccess = public, SetAccess = public)
    x;
    Jc;
    Jc_dot;
    D;
    x_dot;
 end
 
 methods
     
    function C=Dynamics_constrain(x,Jc,Jc_dot)
         C.x=x;
         C.D=Dynamics(x);
         C.Jc=Jc;
         C.Jc_dot=Jc_dot;         
         [n nandemo]=size(Jc);
         C.x_dot=zeros(16+2*n,1);
         y=C.x;
         C.x(7)=y(13);
         C.x(8)=y(15);
         C.x(9:8+n)=zeros(n,1);
         C.x(9+n)=y(7);
         C.x(10+n)=y(8);
         C.x(11+n)=y(9);
         C.x(12+n)=y(10);
         C.x(13+n)=y(11);
         C.x(14+n)=y(12);
         C.x(15+n)=y(14);
         C.x(16+n)=y(16);
         C.x(17+n:16+2*n)=zeros(n,1);
    end
     
    function setConstraincondition(C,Jc,Jc_dot)
        C.Jc=Jc;
        C.Jc_dot=Jc_dot;
    end
    
    function Move(C,u,dt,LEVEL)
        [n nandemo]=size(C.Jc);
        
        y=zeros(16,1);
        y(1:6,1)=C.x(1:6,1);
        y(7:12,1)=C.x(9+n:14+n,1);
        y(13)=C.x(7,1);
        y(14)=C.x(15+n,1);
        y(15)=C.x(8,1);
        y(16)=C.x(16+n,1);
        
        C.D.calculateMatrix(y,u,LEVEL);
        %x=[x y z ph th ps eta1 eta2 x_dot y_dot z_dot ph_dot th_dot ps_dot eta1_dot eta2_dot]
        
        if LEVEL==0
        z=[C.x(1:6,1);C.x(9:8+n);C.x(9+n:14+n,1);C.x(17+n:16+2*n)];
        M=zeros(6+n);
        M(1:6,1:6)=C.D.Mq;
        M(1:6,7:6+n)=C.Jc(:,1:6)';
        M(7:6+n,1:6)=C.Jc(:,1:6);
        
        H=zeros(6+n,1);
        H(1:6,1)=C.D.Fq-C.D.Cq-C.D.Gq;
        H(7:6+n,1)=-C.Jc_dot(:,1:6)*z(7+n:12+n,1);
        
        A=zeros(12+2*n);
        A(1:6+n,1:6+n)=eye(6+n);
        A(7+n:12+2*n,7+n:12+2*n)=M;
        
        B=zeros(12+2*n,1);
        B(1:6+n,1)=z(7+n:12+2*n);
        B(7+n:12+2*n)=H;
        
        if isnan(det(A))
            disp("発散!!");
            disp(A)
            pause
        end
        
        C.x_dot=A\B;
        z=z+A\B.*dt;
        C.x(1:6,1)=z(1:6);
        C.x(9:8+n)=z(7:6+n);
        C.x(9+n:14+n,1)=z(7+n:12+n);
        C.x(17+n:16+2*n)=z(13+n:12+2*n);
        C.x=round(C.x,15);
        
        
        elseif LEVEL==2
        M=zeros(8+n);
        M(1:8,1:8)=C.D.Mq;
        M(1:8,9:8+n)=C.Jc';
        M(9:8+n,1:8)=C.Jc;
        
        H=zeros(8+n,1);
        H(1:8,1)=C.D.Fq-C.D.Cq-C.D.Gq;
        H(9:8+n,1)=-C.Jc_dot*C.x(9+n:16+n,1);
        
        A=zeros(16+2*n);
        A(1:8+n,1:8+n)=eye(8+n);
        A(9+n:16+2*n,9+n:16+2*n)=M;
        
        B=zeros(16+2*n,1);
        B(1:8+n,1)=C.x(9+n:16+2*n);
        B(9+n:16+2*n)=H;
        
        if isnan(det(A))
            disp("発散!!");
            disp(A)
            pause
        end
        
        C.x=C.x+A\B.*dt;
        C.x=round(C.x,15);
        end
    end
 end
end