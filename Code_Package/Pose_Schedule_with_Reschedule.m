
%ini_robot 
%Path_Schedule  % Terminal path planning
overflow = 0; flow_pace = 1; theta = 1; pppp = 1; angle_change = 1;
finite_states = 100000; iterations = 10000; max_tolerance = 10000;
re_sche_vision = 1;

x1_1=[ 0 1 2 3 4 -1 -2 -3]; % permutation and combination, break down each axis to 45¡ã, from -135¡ãto 180¡ã
x2_2=[ 0 1 2 3 4 -1 -2 -3]; 
x3_3=[ 0 1 2 3 4 -1 -2 -3]; 
[x3_3,x2_2,x1_1] = ndgrid(x3_3,x2_2,x1_1);
angle_orders=[x1_1(:) x2_2(:) x3_3(:)]; % All possible combination

obstacle; obs_2 = obs_1; place_robot = place;
weizi_n = 1; l = 1; finish = 0; cannot = 0;
distance_limited_joint = 0.85;
distance_limited_point = 0.85;
kkk = 0;  % reschedule max times
pick_angle = 1; pick_angle_mark = 1;
rtx = 0; rty = 0; rtz = 0;

q6=[0 0 0 0 0 0]; 
q5=[0 0 0 0 0]; 
q4=[0 0 0 0]; 
q3=[0 0 0]; 
q2=[0 0]; 
q1=[0];       
q0=[1.0122   -0.1764   -2.3202    0.9259    1.5708    0.5586]; 
set_q0 = [1.0122   -0.1764   -2.3202    0.9259    1.5708    0.5586];
%Iniciar Rbt     1.44
% L1 = Link([0    -2       0          0             0],'modified'); % modified represents use optimial DH
% L2 = Link([0       0       0         pi/2           0],'modified');
% L3 = Link([0       0    -2.64          0             0],'modified');
% L4 = Link([0     1.06    -2.36          0             0],'modified');
% L5 = Link([0     1.14       0          pi/2          0],'modified');
% L6 = Link([0     0.67        0         -pi/2          0],'modified');

L1 = Link([0      0      0          0             0],'modified');
L2 = Link([0       0       0         pi/2           0],'modified');
L3 = Link([0       0     -3         0             0],'modified');
L4 = Link([0     1.06    -3        0             0],'modified');
L5 = Link([0     1.14      0          pi/2          0],'modified');
L6 = Link([0     1.67      0         -pi/2          0],'modified');
L3_bu = Link([0     0    -3        0             0],'modified');

Rbt=SerialLink([L1 L2 L3 L4 L5 L6]); % Built whole Arm
Rbt5=SerialLink([L1 L2 L3 L4 L5 ]);
Rbt4=SerialLink([L1 L2 L3 L4 ]);
Rbt3_bu = SerialLink([L1 L2 L3 L3_bu ]);
Rbt3=SerialLink([L1 L2 L3 ]);
Rbt2=SerialLink([L1 L2 ]);
Rbt1=SerialLink([L1]);
atj=zeros(4,4);

while kkk <= 50
    while true
        collision = 0;
        collision_1 = 0;
        path_length = workpath(nppp-1,1);
        sqtraj=Inverse_Modified_Upright(rtx,rty,rtz,place(l,1),place(l,2),place(l,3));
        q6 = sqtraj(weizi_n ,:);
        
        if q6 == [0 0 0 0 0 0]
            overflow(flow_pace,:) = l;
            flow_pace = flow_pace+1;
        end
        
        q6 = q6(1,:)*pi/180;
        t=0:0.02:1;
        q = jtraj(q0,q6,t);
        
        for j=1:1:length(t) 
            for i = 1:1:6
                q6(1,i) = q(j,i);
            end
            for i = 1:1:5
                q5(1,i) = q(j,i);
            end
            for i = 1:1:4
                q4(1,i) = q(j,i);
            end
            for i = 1:1:3
                q3(1,i) = q(j,i);
            end
            for i = 1:1:2
                q2(1,i) = q(j,i);
            end
            for i = 1:1:1
                q1 = q(j,i);
            end
            q3_bu = q4;
            
            atj1=Rbt1.fkine(q1);
            atj2=Rbt2.fkine(q2);
            atj3=Rbt3.fkine(q3);
            atj3_bu = Rbt3_bu.fkine(q3_bu);
            atj4=Rbt4.fkine(q4);
            atj5=Rbt5.fkine(q5);
            atj6=Rbt.fkine(q6);
            
            llength2 = [atj2(1,4),atj2(2,4),atj2(3,4)]; % return all axis positions
            llength3 = [atj3(1,4),atj3(2,4),atj3(3,4)];
            llength3_bu =  [atj3_bu(1,4),atj3_bu(2,4),atj3_bu(3,4)];
            llength4 = [atj4(1,4),atj4(2,4),atj4(3,4)];
            llength5 = [atj5(1,4),atj5(2,4),atj5(3,4)];
            llength6 = [atj6(1,4),atj6(2,4),atj6(3,4)];
            
            for jjj = 1:1:obs_1
                realdistance_1 = shortest_distance(llength2,llength3,obstacle(jjj,:));
                realdistance_2 = shortest_distance(llength3,llength3_bu,obstacle(jjj,:));
                realdistance_3_bu = shortest_distance(llength3_bu,llength4,obstacle(jjj,:));
                realdistance_3 = shortest_distance(llength4,llength5,obstacle(jjj,:));
                realdistance_4 = shortest_distance(llength5,llength6,obstacle(jjj,:));
                % realdistance_5 = Distance(llength2,obstacle(jjj,:))
                realdistance_6 = distance(llength3,obstacle(jjj,:));
                realdistance_7 = distance(llength4,obstacle(jjj,:));
                realdistance_8 = distance(llength5,obstacle(jjj,:));
                realdistance_9 = distance(llength6,obstacle(jjj,:));
                
                if realdistance_1 <= distance_limited_joint
                    collision = 1;
                    realdistance_1
                    break
                end
                if realdistance_2 <= distance_limited_joint
                    collision = 1;
                    realdistance_2
                    break
                end
                if realdistance_3_bu <= distance_limited_joint
                    collision = 1;
                    realdistance_3_bu
                    break
                end
                if realdistance_3 <= distance_limited_joint
                    collision = 1;
                    realdistance_3
                    break
                end
                if realdistance_4 <= distance_limited_joint
                    collision = 1;
                    realdistance_4
                    break
                end
                
                if realdistance_6 <= distance_limited_point
                    collision = 1;
                    realdistance_6
                    break
                end
                if realdistance_7 <= distance_limited_point
                    collision = 1;
                    realdistance_7
                    break
                end
                if realdistance_8 <= distance_limited_point
                    collision = 1;
                    realdistance_8
                    break
                end
                if realdistance_9 <= distance_limited_point
                    collision = 1;
                    realdistance_9
                    break
                end
            end
            
            if max(q0) ~= 0 || min(q0) ~= 0
                if max(q6) <= 1.0e-10 && min(q6) >= -1.0e-10
                    collision = 1;
                end
            end
            %----------------------------------------------------------------------------------
            if collision == 1
                collision = 0;
                collision_1 = 1;
                weizi_n = weizi_n + 1;
                break;  % Jump out and pick next set angles
            else if  j == length(t)
                    marker_weizi_rough(theta,[1,2,3,4,5,6]) = q6; % assign value for the 123456 comlumns of marker_weizi_rough
                    theta = theta + 1;
                    weizi_n = weizi_n + 1;
                end
            end
            
            if  (j == length(t) && theta ~= 1 && weizi_n == 9) % The Angle with the least change with the last Angle is found in the satisfied Angle solution set
                pppp = pppp+1;
                for k_theta = 1:1:(theta-1)
                    qn = marker_weizi_rough(k_theta,[1,2,3,4,5,6]);
                    marker_weizi_rough(k_theta,7) = sum(abs(qn - q0));
                end
                [theta_m,theta_n] = min(marker_weizi_rough(1:1:k_theta,7));
                marker_weizi(l,:) = marker_weizi_rough(theta_n,[1,2,3,4,5,6])
                q0 =  marker_weizi(l,:);
                l = l+1;
                pick_angle_mark = 1;
                weizi_n = 1;
                theta = 1; % the number of sets in marker_weizi_rough£¬1 represents none
                marker_weizi_rough(:,:) = 0;
                pause(1)
            end
            %-----------------------------------------------------------
            if l == path_length + 2
                finish = 1;
                break
            end
        end
        
        if (weizi_n == 9 && theta == 1)
            cannot = 1;
            break;
        end
        
        if  (collision_1 == 1 && theta ~= 1 && weizi_n == 9) % When the last of the set of angles is also touched, see if marker_weizi_rough has a suitable Angle 
            collision_1 = 0;
            for k_theta = 1:1:(theta-1)
                qn = marker_weizi_rough(k_theta,[1,2,3,4,5,6]);
                marker_weizi_rough(k_theta,7) = sum(abs(qn - q0));
            end
            [theta_m,theta_n] = min(marker_weizi_rough(1:1:k_theta,7));
            marker_weizi(l,:) = marker_weizi_rough(theta_n,[1,2,3,4,5,6])
            q0 =  marker_weizi(l,:);
            l = l+1;
            pick_angle_mark = 1;
            weizi_n = 1;
            theta = 1;
            marker_weizi_rough(:,:) = 0;
            pause(1)
        end
        
        %-----------------------------------------------------------
        if l == path_length + 2;
            finish = 1;
        end
        
        if finish == 1;
            break;
        end
    end
    
    if cannot == 1
        cannot = 0;
        weizi_n = 1;
        pick_angle_mark = pick_angle_mark + 1;
        pick_angle = pick_angle + 1
        angle_change = angle_change + 1;
        if pick_angle == length(angle_orders) + 1
            pause(2);
            pick_angle = 1;
        end
        if pick_angle_mark == length(angle_orders) + 1
            if [place(l,1),place(l,2),place(l,3)] == terminal
                obstacle(obs_2+1,:) = [place(l-1,1),place(l-1,2),place(l-1,3)]
                pause(2);
            else obstacle(obs_2+1,:) = [place(l,1),place(l,2),place(l,3)]
                pause(2);
            end
            obs_2 = obs_2 + 1;
            nppp = 1;  % start reschedule
            kkk = kkk+1;
            q0 = [0 0 0 0 0 0];
            l = 1;
            pick_angle = 1;
            pick_angle_mark = 1;
            rtx = 0;
            rty = 0;
            rtz = 0;
        end
    end

    rtx = angle_orders(pick_angle,1)*45;
    rty = angle_orders(pick_angle,2)*45;
    rtz = angle_orders(pick_angle,3)*45;
%---------------------------------------------------------------------------Record that point as new obstacle and reschedule 
    if nppp == 1 && re_sche_vision == 1; Plot_Environment; end
    while nppp <= iterations
        place(:,:)=(0);
        place(1,:) = [0,-2,4];
        workpath(nppp,1) = 0;
        r = 0; getout = 0; getbuout = 0; kongpao = 0; getout_past = 0;
        greedy_set = min(0.9+0.1*nppp/9900, 1); % greedy 
        
        for i=1:1:max_tolerance
            r = 0;
            if nppp <= 9900
                greedy = rand(1,1);
            else
                greedy = 0;
            end
            if greedy <= greedy_set
                [mmm,nnn]=max(action(ppp,:)); % pick max action
                c = nnn;
            else suiji = randi([1,6],1,1);
                c = suiji;
            end
            if c==1
                if place(i,1) < 5
                    place(i+1,:) = place(i,:)+[xxx,0,0];
                else
                    place(i+1,:) = place(i,:);
                    r = r - 4000;
                end
            end
            
            if c==6
                if place(i,1) > -5
                    place(i+1,:) = place(i,:)-[xxx,0,0];
                else
                    place(i+1,:) = place(i,:);
                    r = r - 4000;
                end
            end
            
            if c==3
                if place(i,2) < 5
                    place(i+1,:) = place(i,:)+[0,yyy,0];
                else
                    r = r - 4000;
                    place(i+1,:) = place(i,:);
                end
            end
            
            if c==4 %yÏÂ
                if place(i,2) > -5
                    place(i+1,:) = place(i,:)-[0,yyy,0];
                else
                    place(i+1,:) = place(i,:);
                    r = r - 4000;
                end
            end
            
            if c==2
                if place(i,3) < 5
                    place(i+1,:) = place(i,:)+[0,0,zzz];
                else
                    place(i+1,:) = place(i,:);
                    r = r - 4000;
                end
            end
            
            if c==5
                if place(i,3) > -5
                    place(i+1,:) = place(i,:)-[0,0,zzz];
                else
                    place(i+1,:) = place(i,:);
                    r = r - 4000;
                end
            end

            if getout_past == 1
                if c == (7-c_past)
                    getout_past = 0;
                    r = r - 8000;
                end
                
                if c == c_past
                    place(i+1,:) = place(i,:);
                    getout_past = 0;
                    r = r - 230000;
                end
            end
            distance_ontime = abs(terminal(1,1)-place(i+1,1)) + abs(terminal(1,2)-place(i+1,2)) + abs(terminal(1,3)-place(i+1,3));
            distance_reward = (distance_origin - distance_ontime);
            r = r + distance_reward * 80;
            for obs=1:1:obs_2
                if place(i+1,:) == obstacle(obs,:)
                    punish = punish+1;
                    getout = 1;
                end
            end
            if place(i+1,:) == terminal
                get = get+1;
                getbuout = 1;
            else if getout ~= 1
                kongpao_biao = 1;
                kongpao = kongpao +1;
                 end
            end

            if getout == 1
                r = r - 4000;
            end
            
            if getbuout == 1
                r = r + 20000;
            end
            
            if kongpao_biao == 1
                kongpao_biao = 0;
                r = r - 15000;
            end
            
            ddd = 1;
            truei = 2;
            while true % check whether the state in the states list
                if realplace(ddd,:) == place(i,:)
                    ppp = ddd;
                    if i == 1
                        ppp = 1;
                    end
                end
                
                if realplace(ddd,:) == place(i+1,:)  
                    truei=ddd;
                    break
                end
                
                if (ddd+1) > inde
                    realplace(inde,:) = place(i+1,:)
                    inde = inde + 1;
                    break
                end
                ddd = ddd+1;
            end
            
            q_predict=action(ppp,c);
            [max_action,index]=max(action(truei,:));
            q_target = r + GAMA * max_action;
            action(ppp, c) = action(ppp,c) + ALPHA * (q_target - q_predict);
            ppp_past = ppp;
            ppp = truei;
            c_past = c;
            if re_sche_vision == 1
                set(h,'xdata',place(i,1),'ydata',place(i,2),'zdata',place(i,3)); drawnow; 
            end
            if getout == 1
                if i > 1
                    getout_past = 1;
                    place(i+1,:) = place(i,:);
                    ppp = ppp_past;
                    tingzhi = tingzhi+1;
                    getout = 0;
                end
            end

            workpath(nppp,1)= workpath(nppp,1)+1; % calculate steps
            if  getbuout == 1
                getbuout = 0;
                break
            end
        end
        nppp = nppp+1
    end
    if finish == 1
        finish = 0;
        break
    end
end
show_line = 0;
Plot_Arm_Movement;