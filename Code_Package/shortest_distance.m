function true_realdistance = shortest_distance(poin1, poin2, obstacl1)
     pqx = poin2(1,1) - poin1(1,1);
	 pqy = poin2(1,2) - poin1(1,2);
	 pqz = poin2(1,3) - poin1(1,3);
	 dx = obstacl1(1,1) - poin1(1,1);
	 dy = obstacl1(1,2) - poin1(1,2);
	 dz = obstacl1(1,3) - poin1(1,3);
	 d_1 = pqx*pqx + pqy*pqy + pqz*pqz;  % qp线段长度的平方
	 t_2 = pqx*dx + pqy*dy + pqz*dz;   % p pt向量 点积 pq 向量（p相当于A点，q相当于B点，pt相当于P点）

		t_1 = t_2 / d_1;    %?此时t 相当于 上述推导中的 r。
	if (t_1 < 0)
		t_1 = 0;  %?当t（r）< 0时，最短距离即为 pt点 和 p点（A点和P点）之间的距离。
	else if (t_1 > 1)
		t_1 = 1; %  当t（r）> 1时，最短距离即为 pt点 和 q点（B点和P点）之间的距离。
        else
            t_1 = 2;
        end
    end
    
    if t_1 == 0
        true_realdistance = Distance(poin1,obstacl1);
    end
    if t_1 == 1
         true_realdistance = Distance(poin2,obstacl1);
    end
    if t_1 == 2
         true_realdistance = point_to_line(poin1,poin2,obstacl1);
    end

end

function realdistance = point_to_line(point1, point2, obstacle1)

vx1x2 = point2 - point1;%向量x1到x2
vx1x3 = obstacle1 - point1;%向量x1到x3[
% if  vx1x2'* vx1x2 == 0 %如果x1与x2重合，注意，对浮点数采用等于0的判断不可靠
%     realdistance = 0;
% elseif vx1x3' * vx1x3 == 0%如果x1与x3重合，这种情况下距离是否为0需要自己定义
%     realdistance = 0;
% else
     inner_product = dot(vx1x2, vx1x3);%两个向量内积
     inner_product_2 = inner_product * inner_product;%内积平方
     cos_2 = inner_product_2 / dot(vx1x2,vx1x2) / dot(vx1x3, vx1x3);%夹角cos值的平方
     sin_2 = 1 - cos_2;  %夹角sin的平方
     dis_2 = dot(vx1x3,vx1x3) * sin_2;
     realdistance = sqrt(dis_2); %距离
end

function dist = Distance(X,Y)
     dist = sqrt(sum((X - Y).^2));
end