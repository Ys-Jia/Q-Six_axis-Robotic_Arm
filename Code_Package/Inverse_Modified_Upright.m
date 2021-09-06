function itheta =Inverse_Modified_Upright(roll, pitch, yaw, x, y, z)

    itheta = [0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0];

    DH_JXB =[0 0 0 0; 90 0 0 0; 0 -3 0 0; 0 -3 1.06 0; 90 0 1.14 0; -90 0 1.67 0];
    p2=DH_JXB(2,4);p4=DH_JXB(4,4);
    a2=DH_JXB(3,2);a3=DH_JXB(4,2);d1=DH_JXB(1,3);d4=DH_JXB(4,3);d5=DH_JXB(5,3);
    A = [roll, pitch, yaw; x, y, z];
    X=A(2,1);Y=A(2,2);Z=A(2,3);
    gama=A(1,1)*pi/180;beta=A(1,2)*pi/180;alpha=A(1,3)*pi/180;
    
    theta7=0;a6=0;afa6=0;d7=DH_JXB(6,3);
    T67=[   cos(theta7),-sin(theta7),0,a6;
                sin(theta7)*cos(afa6),cos(theta7)*cos(afa6),-sin(afa6),-sin(afa6)*d7;
                sin(theta7)*sin(afa6),cos(theta7)*sin(afa6),cos(afa6),cos(afa6)*d7;
                0,0,0,1
             ];
   T_goat=[  cos(beta)*cos(gama),-cos(beta)*sin(gama),sin(beta),X;
                   sin(alpha)*sin(beta)*cos(gama)+cos(alpha)*sin(gama), -sin(alpha)*sin(beta)*sin(gama)+cos(alpha)*cos(gama), -sin(alpha)*cos(beta),Y;
                  -cos(alpha)*sin(beta)*cos(gama)+sin(alpha)*sin(gama), cos(alpha)*sin(beta)*sin(gama)+sin(alpha)*cos(gama),  cos(alpha)*cos(beta),Z;
                  0,0,0,1
              ];
    T06=T_goat/T67;
    
    nx=T06(1,1);ny=T06(2,1);
    ox=T06(1,2);oy=T06(2,2);
    ax=T06(1,3);ay=T06(2,3);az=T06(3,3);
    px=T06(1,4);py=T06(2,4);pz=T06(3,4);
    
    k=0;
    ForJudgment=px^2+py^2-d4^2;
    
    if ForJudgment<-1e-6
        % disp('Out of work space, no answer');
    else
        
        if ForJudgment>=-1e-6&&ForJudgment<0
            ForJudgment=0;
        end
        theta1_1=atan2(py,px)-atan2(-d4,sqrt(ForJudgment));
        theta1_2=atan2(py,px)-atan2(-d4,-sqrt(ForJudgment));
        
        S5_1=sqrt((-sin(theta1_1)*nx+cos(theta1_1)*ny)^2+(-sin(theta1_1)*ox+cos(theta1_1)*oy)^2);
        theta5_1=atan2(S5_1,sin(theta1_1)*ax-cos(theta1_1)*ay);
        S5_2=-sqrt((-sin(theta1_1)*nx+cos(theta1_1)*ny)^2+(-sin(theta1_1)*ox+cos(theta1_1)*oy)^2);
        theta5_2=atan2(S5_2,sin(theta1_1)*ax-cos(theta1_1)*ay);
        S5_3=sqrt((-sin(theta1_2)*nx+cos(theta1_2)*ny)^2+(-sin(theta1_2)*ox+cos(theta1_2)*oy)^2);
        theta5_3=atan2(S5_3,sin(theta1_2)*ax-cos(theta1_2)*ay);
        S5_4=-sqrt((-sin(theta1_2)*nx+cos(theta1_2)*ny)^2+(-sin(theta1_2)*ox+cos(theta1_2)*oy)^2);
        theta5_4=atan2(S5_4,sin(theta1_2)*ax-cos(theta1_2)*ay);
        
        S234 = [0; 0; 0; 0];C234 = [0; 0; 0; 0];
        B = [0; 0; 0; 0];B1 = [0; 0; 0; 0];B2 = [0; 0; 0; 0];
        C = [0; 0; 0; 0];
        theta2 = [0; 0; 0; 0; 0; 0; 0; 0];
        theta23 = [0; 0; 0; 0; 0; 0; 0; 0];
        theta234 = [0; 0; 0; 0; 0; 0; 0; 0];
        theta3 = [0; 0; 0; 0; 0; 0; 0; 0];
        theta4 = [0; 0; 0; 0; 0; 0; 0; 0];
        
        if abs(S5_1)>1e-6
            theta6_1=atan2((-sin(theta1_1)*ox+cos(theta1_1)*oy)/S5_1,(sin(theta1_1)*nx-cos(theta1_1)*ny)/S5_1);
            S234(1)=-az/S5_1;C234(1)=-(cos(theta1_1)*ax+sin(theta1_1)*ay)/S5_1;theta234(1)=atan2(S234(1),C234(1));
            B1(1)=cos(theta1_1)*px+sin(theta1_1)*py-d5*S234(1);B2(1)=pz-d1+d5*C234(1);A(1)=-2*B2(1)*a2;B(1)=2*B1(1)*a2;C(1)=B1(1)^2+B2(1)^2+a2^2-a3^2;
            if A(1)^2+B(1)^2-C(1)^2>=0
                theta2(1)=atan2(B(1),A(1))-atan2(C(1),sqrt(A(1)^2+B(1)^2-C(1)^2));
                theta2(2)=atan2(B(1),A(1))-atan2(C(1),-sqrt(A(1)^2+B(1)^2-C(1)^2));
                theta23(1)=atan2((B2(1)-a2*sin(theta2(1)))/a3,(B1(1)-a2*cos(theta2(1)))/a3);theta23(2)=atan2((B2(1)-a2*sin(theta2(2)))/a3,(B1(1)-a2*cos(theta2(2)))/a3);
                theta4(1)=theta234(1)-theta23(1);
                theta4(2)=theta234(1)-theta23(2);
                theta3(1)=theta23(1)-theta2(1);
                theta3(2)=theta23(2)-theta2(2);
                itheta(k+1,:)=[theta1_1 theta2(1)-p2*pi/180 theta3(1) theta4(1)-p4*pi/180 theta5_1 theta6_1];
                itheta(k+2,:)=[theta1_1 theta2(2)-p2*pi/180 theta3(2) theta4(2)-p4*pi/180 theta5_1 theta6_1];
                k=k+2;
            end
        end
        if abs(S5_2)>1e-6
            theta6_2=atan2((-sin(theta1_1)*ox+cos(theta1_1)*oy)/S5_2,(sin(theta1_1)*nx-cos(theta1_1)*ny)/S5_2);
            S234(2)=-az/S5_2;C234(2)=-(cos(theta1_1)*ax+sin(theta1_1)*ay)/S5_2;theta234(2)=atan2(S234(2),C234(2));
            B1(2)=cos(theta1_1)*px+sin(theta1_1)*py-d5*S234(2);B2(2)=pz-d1+d5*C234(2);A(2)=-2*B2(2)*a2;B(2)=2*B1(2)*a2;C(2)=B1(2)^2+B2(2)^2+a2^2-a3^2;
            if A(2)^2+B(2)^2-C(2)^2>=0
                theta2(3)=atan2(B(2),A(2))-atan2(C(2),sqrt(A(2)^2+B(2)^2-C(2)^2));
                theta2(4)=atan2(B(2),A(2))-atan2(C(2),-sqrt(A(2)^2+B(2)^2-C(2)^2));
                theta23(3)=atan2((B2(2)-a2*sin(theta2(3)))/a3,(B1(2)-a2*cos(theta2(3)))/a3);theta23(4)=atan2((B2(2)-a2*sin(theta2(4)))/a3,(B1(2)-a2*cos(theta2(4)))/a3);
                theta4(3)=theta234(2)-theta23(3);
                theta4(4)=theta234(2)-theta23(4);
                theta3(3)=theta23(3)-theta2(3);
                theta3(4)=theta23(4)-theta2(4);
                itheta(k+1,:)=[theta1_1 theta2(3)-p2*pi/180 theta3(3) theta4(3)-p4*pi/180 theta5_2 theta6_2];
                itheta(k+2,:)=[theta1_1 theta2(4)-p2*pi/180 theta3(4) theta4(4)-p4*pi/180 theta5_2 theta6_2];
                k=k+2;
            end
        end
        if abs(S5_3)>1e-6
            theta6_3=atan2((-sin(theta1_2)*ox+cos(theta1_2)*oy)/S5_3,(sin(theta1_2)*nx-cos(theta1_2)*ny)/S5_3);
            S234(3)=-az/S5_3;C234(3)=-(cos(theta1_2)*ax+sin(theta1_2)*ay)/S5_3;theta234(3)=atan2(S234(3),C234(3));
            B1(3)=cos(theta1_2)*px+sin(theta1_2)*py-d5*S234(3);B2(3)=pz-d1+d5*C234(3);A(3)=-2*B2(3)*a2;B(3)=2*B1(3)*a2;C(3)=B1(3)^2+B2(3)^2+a2^2-a3^2;
            if A(3)^2+B(3)^2-C(3)^2>=0
                theta2(5)=atan2(B(3),A(3))-atan2(C(3),sqrt(A(3)^2+B(3)^2-C(3)^2));
                theta2(6)=atan2(B(3),A(3))-atan2(C(3),-sqrt(A(3)^2+B(3)^2-C(3)^2));
                theta23(5)=atan2((B2(3)-a2*sin(theta2(5)))/a3,(B1(3)-a2*cos(theta2(5)))/a3);theta23(6)=atan2((B2(3)-a2*sin(theta2(6)))/a3,(B1(3)-a2*cos(theta2(6)))/a3);
                theta4(5)=theta234(3)-theta23(5);
                theta4(6)=theta234(3)-theta23(6);
                theta3(5)=theta23(5)-theta2(5);
                theta3(6)=theta23(6)-theta2(6);
                itheta(k+1,:)=[theta1_2 theta2(5)-p2*pi/180 theta3(5) theta4(5)-p4*pi/180 theta5_3 theta6_3];
                itheta(k+2,:)=[theta1_2 theta2(6)-p2*pi/180 theta3(6) theta4(6)-p4*pi/180 theta5_3 theta6_3];
                k=k+2;
            end
        end
        if abs(S5_4)>1e-6
            theta6_4=atan2((-sin(theta1_2)*ox+cos(theta1_2)*oy)/S5_4,(sin(theta1_2)*nx-cos(theta1_2)*ny)/S5_4);
            S234(4)=-az/S5_4;C234(4)=-(cos(theta1_2)*ax+sin(theta1_2)*ay)/S5_4;theta234(4)=atan2(S234(4),C234(4));
            B1(4)=cos(theta1_2)*px+sin(theta1_2)*py-d5*S234(4);B2(4)=pz-d1+d5*C234(4);A(4)=-2*B2(4)*a2;B(4)=2*B1(4)*a2;C(4)=B1(4)^2+B2(4)^2+a2^2-a3^2;
            if A(4)^2+B(4)^2-C(4)^2>=0
                theta2(7)=atan2(B(4),A(4))-atan2(C(4),sqrt(A(4)^2+B(4)^2-C(4)^2));
                theta2(8)=atan2(B(4),A(4))-atan2(C(4),-sqrt(A(4)^2+B(4)^2-C(4)^2));
                theta23(7)=atan2((B2(4)-a2*sin(theta2(7)))/a3,(B1(4)-a2*cos(theta2(7)))/a3);theta23(8)=atan2((B2(4)-a2*sin(theta2(8)))/a3,(B1(4)-a2*cos(theta2(8)))/a3);
                theta4(7)=theta234(4)-theta23(7);
                theta4(8)=theta234(4)-theta23(8);
                theta3(7)=theta23(7)-theta2(7);
                theta3(8)=theta23(8)-theta2(8);
                itheta(k+1,:)=[theta1_2 theta2(7)-p2*pi/180 theta3(7) theta4(7)-p4*pi/180 theta5_4 theta6_4];
                itheta(k+2,:)=[theta1_2 theta2(8)-p2*pi/180 theta3(8) theta4(8)-p4*pi/180 theta5_4 theta6_4];
                k=k+2;
            end
        end
        
        if k>0
            itheta=itheta*180/pi;
            for i=1:k
                for j=1:6
                    if itheta(i,j)<=-180
                        itheta(i,j)=itheta(i,j)+360;%restrict angle from -180¡ª+180
                    elseif itheta(i,j)>180
                        itheta(i,j)=itheta(i,j)-360;
                    end
                end
            end
        else 
            %disp('Singular position, infinite answer');
        end
    end
end