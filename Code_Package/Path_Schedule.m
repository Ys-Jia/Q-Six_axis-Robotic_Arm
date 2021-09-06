clc;
clear all;

episode = 1;
tingzhi = 0;
finite_states = 100000;
iterations = 10000;
max_tolerance = 10000;
visualize = 0; % whether keep visual

while episode <= 1
    ALPHA = 0.05;
    GAMA = 0.9;
    inde = 2;
    truei = 1;
    get=0;
    punish=0;
    ppp = 1;
    nppp=1;
    c_past=0;
    r = 0;
    
    workpath(finite_states,:) = (0);
    walk(finite_states,:) = (0);
    terminal =  [-1,2,0];
    obstacle =  [0,1,0;0,3,0;1,1,0;-1,1,0;-1,2,1;-1,3,1;-1,3,0;0,1,1;0,3,1;1,1,1;-1,1,1;0,1,-1;0,3,-1;1,1,-1;1,2,-1;1,3,-1;-1,1,-1;-1,2,-1;-1,3,-1;0,2,-1];
    %[0,1,0; 0,3,0; 1,1,0; -1,1,0; -1,2,1; -1,3,1; -1,3,0; 0,1,1; 0,3,1; 1,1,1; -1,1,1; 0,1,-1; 0,3,-1; 1,1,-1; 1,2,-1; 1,3,-1; -1,1,-1; -1,2,-1;  -1,3,-1; 0,2,-1];
    
    [obs_1,nnnn] = size(obstacle);
    
    step = 1;
    xxx=step;
    yyy=step;
    zzz=step;
    
    pic = linspace(1,finite_states,finite_states);
    place(finite_states,3)=(0);
    place(1,:) = [0,-2,4]; % Start Point
    realplace(finite_states,3)=(0);
    realplace(:,:)=0;
    action(finite_states,6)=(0);
    action(:,:) = 0;
    distance_origin = abs(terminal(1,1)-0) + abs(terminal(1,2)-0) + abs(terminal(1,3)-0);
    Plot_Environment;
%---------------------------------------------------------------------------Training!
    while nppp <= iterations % iterations
        
        place(:,:)=(0);
        place(1,:) = [0,-2,4];
        workpath(nppp,1)=0;
        r = 0;
        getout = 0;
        getbuout = 0;
        kongpao = 0;
        getout_past = 0;
        greedy_set = min(0.9+0.1*nppp/9900, 1);
        
        for i=1:1:max_tolerance % max move time for one iteration
            r = 0;
            
            if nppp <= 9900
                greedy = rand(1,1);
            else
                greedy = 0;
            end
            
            if greedy<=greedy_set
                [mmm,nnn]=max(action(ppp,:));
                c = nnn;
%--------------------------------------------------------------------------------------Action
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
            if c==3 %y Up
                if place(i,2) < 5
                    place(i+1,:) = place(i,:)+[0,yyy,0];
                else
                    r = r - 4000;
                    place(i+1,:) = place(i,:);
                end
            end
            if c==4 %y Down
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

            if getout_past == 1  %-------Go back and Bypass
                if c == (7-c_past) 
                    getout_past = 0;
                    r = r - 10000;
                end
                if c == c_past
                    place(i+1,:) = place(i,:);
                    getout_past = 0;
                    r = r - 230000;
                end
            end

            %---------------------------------------------------------------------------------------The closer you get to the target, the more bonus you get. Vice Versa
            distance_ontime = abs(terminal(1,1)-place(i+1,1)) + abs(terminal(1,2)-place(i+1,2)) + abs(terminal(1,3)-place(i+1,3));
            distance_reward = (distance_origin - distance_ontime);
            r = r + distance_reward * 80;

            for obs=1:1:obs_1 % Collision Detection
                if place(i+1,:) == obstacle(obs,:)
                    punish = punish+1;
                    getout = 1;
                end
            end
            
            if place(i+1,:) == terminal
                get = get+1;
                getbuout = 1;
            end
            
            if getbuout == 0 && getout == 0
                kongpao_biao = 1;
                kongpao = kongpao +1;
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
            
            %---------------------------------------------------------------------------------------- Update Q, (First look for s, s'), update place that never been too
            ddd = 1;
            truei = 2;
            while true
                if realplace(ddd,:) == place(i,:)  %find s
                    ppp=ddd;
                    if i == 1
                        ppp = 1;
                    end
                end
                if realplace(ddd,:) == place(i+1,:)  %find s'
                    truei=ddd;
                    break
                end
                
                if (ddd+1) > inde
                    realplace(inde,:) = place(i+1,:);  %update new place
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
            %---------------------------------------------------------------------go back if collision happens
            if visualize == 1
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
            workpath(nppp,1)= workpath(nppp,1)+1;
            if  getbuout == 1
                getbuout = 0;
                break
            end
        end
        nppp = nppp+1  
    end
    walk(:,episode) = workpath(:,1);
    episode = episode + 1;
end
%-----------------------------------------------------------------------------Draw!!
Plot_Environment;
path_length = workpath(nppp-1,1);
for drawdraw = 1:1:path_length
    plot3(place(drawdraw:drawdraw+1,1),place(drawdraw:drawdraw+1,2),place(drawdraw:drawdraw+1,3),'linewidth',10,'color','b');
end
%-----------------------------------------------------------------------------------
q_table=[realplace, action];
walkrealpath = mean(walk,2);
figure
plot(walkrealpath(1:iterations))
title('Steps by each Iteration'); xlabel('Iteration'); ylabel('Steps')