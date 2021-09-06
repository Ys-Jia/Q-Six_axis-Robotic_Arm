% plot the obstacle, terminal, start point environment
ha=gca;
set(ha,'xlim',[-5,5],'xtick',[-5:1:5]);set(xlabel('x'),'FontSize',12,'Color','r')
set(ha,'ylim',[-5,5],'ytick',[-5:1:5]);set(ylabel('y'),'FontSize',12,'Color','r')
set(ha,'zlim',[-5,5],'ztick',[-5:1:5]);set(zlabel('z'),'FontSize',12,'Color','r')
grid; h=line(place(1, 1),place(1, 2),place(1, 3),'color','red','marker','.','markersize',60,'erasemode','normal');
hold on; plot3(obstacle(:,1),obstacle(:,2),obstacle(:,3),'ks','MarkerFaceColor','k','MarkerSize',100);   % MarkerSize 表示点的大小，b.表示绿色的点
hold on; plot3(terminal(:,1),terminal(:,2),terminal(:,3),'y.','MarkerSize',100);