Plot_Environment; hold on;
path_length = workpath(nppp-1,1);
if show_line == 1
    for drawdraw = 1:1:path_length
        plot3(place(drawdraw:drawdraw+1,1),place(drawdraw:drawdraw+1,2),place(drawdraw:drawdraw+1,3),'linewidth',10,'color','b');
    end
end
hold on;
tt = 0:0.08:1;
for l_mark = 1:1:path_length + 1
    set_q1 = marker_weizi(l_mark,:);
    set_q = jtraj(set_q0,set_q1,tt);
    for i=1:1:length(tt)
        plot(Rbt,set_q(i,:))
    end
    set_q0 = set_q1;
end