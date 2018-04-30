% 3D Delaunay triangulation demo
% Beril Sirmacek
% University of Twente April 2018
%%
clear all
close all
clc
%% Create a 3D point cloud

x = rand(1,5);
y = rand(1,5);
z = rand(1,5);
figure, plot3(y,x,z,'*b'); hold on

% fixed axis limits
h = 0.5;
xminlim = min(x)-h;
xmaxlim = max(x)+h;
yminlim = min(y)-h;
ymaxlim = max(y)+h;
zminlim = min(z)-h;
zmaxlim = max(z)+h;
xlim([xminlim, xmaxlim]);
ylim([yminlim, ymaxlim]);
zlim([zminlim, zmaxlim]);
 
%% Find 3D delaunay triangles

% number of points to draw a circle
nc = 1000;
delaunay_edges = [0,0,0];            
count = 1;
 
for(i = 1:size(x,2))
    for(j = 1:size(x,2))
        for(k = 1:size(x,2))         

            if((i~=j) && (i~=k) && (j~=k)) %if they are not the same points

                % select 3 of the points
                plot3(y(i),x(i),z(i),'or'); hold on
                plot3(y(j),x(j),z(j),'or'); hold on
                plot3(y(k),x(k),z(k),'or'); hold on
                xlim([xminlim, xmaxlim]);
                ylim([yminlim, ymaxlim]);
                zlim([zminlim, zmaxlim]);
                grid on;

                p1 = [y(i),x(i),z(i)];
                p2 = [y(j),x(j),z(j)];
                p3 = [y(k),x(k),z(k)];
                [center,rad,v1,v2] = circlefit3d(p1,p2,p3);

                hc1 = plot3(center(1),center(2),center(3),'om'); hold on;
                xlim([xminlim, xmaxlim]);
                ylim([yminlim, ymaxlim]);
                zlim([zminlim, zmaxlim]);

                % visualization of the circle
                 for(ang=1:361)
                     a = ang/180*pi;
                     xc = center(:,1)+sin(a)*rad.*v1(:,1)+cos(a)*rad.*v2(:,1);
                     yc = center(:,2)+sin(a)*rad.*v1(:,2)+cos(a)*rad.*v2(:,2);
                     zc = center(:,3)+sin(a)*rad.*v1(:,3)+cos(a)*rad.*v2(:,3);
                     hc2(ang) = plot3(xc,yc,zc,'r.');                   
                     xlim([xminlim, xmaxlim]);
                     ylim([yminlim, ymaxlim]);
                     zlim([zminlim, zmaxlim]);
                 end

                 %check the closest points 
                 % Find 3D Euclidean distances of the points
                 dist = find3Ddist(x,y,z,center);
                 [minval, indx] = sort(dist);

                 if(minval(4)>rad) %if there is no point in a sphere around   
                    % connect 3 points with edges
                    % 3D line
                    plot3([y(i),y(j)],[x(i),x(j)],[z(i),z(j)],'LineWidth',3);
                    plot3([y(i),y(k)],[x(i),x(k)],[z(i),z(k)],'LineWidth',3);
                    plot3([y(j),y(k)],[x(j),x(k)],[z(j),z(k)],'LineWidth',3);
                    hold on
                    xlim([xminlim xmaxlim]);
                    ylim([yminlim ymaxlim]);
                    zlim([zminlim, zmaxlim]);
                 end


                pause(0.0001)

                delete(hc1);
                delete(hc2);

                plot3(y,x,z,'*b'); hold on
                hold on
                xlim([xminlim xmaxlim]);
                ylim([yminlim ymaxlim]);
                zlim([zminlim, zmaxlim]);

                
            end
        end
    end
end


%%  