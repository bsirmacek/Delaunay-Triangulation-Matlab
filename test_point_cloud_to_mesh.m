% Delaunay triangulation demo
% Beril Sirmacek
% University of Twente April 2018
%%
clear all
close all
clc
%% Create a 2D point cloud

x = rand(1,10);
y = rand(1,10);
figure, plot(y,x,'*b'); hold on

% fixed axis limits
h = 0.5;
xminlim = min(x)-h;
xmaxlim = max(x)+h;
yminlim = min(y)-h;
ymaxlim = max(y)+h;
xlim([xminlim, xmaxlim]);
ylim([yminlim, ymaxlim]);
 
%% Find delaunay triangles

%max_distance =  max([max(x)-min(x), max(y)-min(y)]);

% number of points to draw a circle
nc = 1000;
delaunay_edges = [0,0,0];            
count = 1;
for(i = 1:size(x,2))
    for(j = 1:size(x,2))
        for(k = 1:size(x,2))
            
            if((i~=j) && (i~=k) && (j~=k))
      
                % select 3 of the points
                plot(y(i),x(i),'or'); hold on
                plot(y(j),x(j),'or'); hold on
                plot(y(k),x(k),'or'); hold on
                xlim([xminlim, xmaxlim]);
                ylim([yminlim, ymaxlim]);
                
                % fit a circle on points
                points = [y(i),x(i); y(j), x(j); y(k),x(k)];
                [radius, xcyc] = fit_circle_through_3_points(points);
                
                %if(radius < max_distance)
                % draw circle
                t = linspace(0,2*pi,nc);
                xp = xcyc(1) + radius*sin(t);
                yp = xcyc(2) + radius*cos(t);
                
                xcheck = x;
                ycheck = y;
                xcheck([i,j,k]) = [];  
                ycheck([i,j,k]) = [];  
                in = inpolygon(ycheck,xcheck,xp,yp);
                
                if(sum(in)==0)
                    line(xp,yp,'color', 'r');
                    hold on
                    xlim([xminlim, xmaxlim]);
                    ylim([yminlim, ymaxlim]);
                    
                    delaunay_edges(count,:) = [i,j,k];
                    count = count + 1;
                else
                    line(xp,yp,'color', 'g');
                    hold on
                    xlim([xminlim, xmaxlim]);
                    ylim([yminlim, ymaxlim]);
                end
                %in
                
                clear t xp yp radius xcyc xcheck ycheck;
                % change point colors to the original
                pause(0.01)
                
                %end
                hold off
                plot(y,x,'*b');
                hold on
                xlim([xminlim xmaxlim]);
                ylim([yminlim ymaxlim]);
                
            end
        end
    end
end

%% Draw edges
    
figure, plot(y,x,'*b'); 
hold on
xlim([xminlim xmaxlim]);
ylim([yminlim, ymaxlim]);

for(i = 1:count-1)
    ind1 = delaunay_edges(i,1);
    ind2 = delaunay_edges(i,2);
    ind3 = delaunay_edges(i,3);
    
    xv(1) = x(ind1);
    xv(2) = x(ind2);
    xv(3) = x(ind3);
    
    yv(1) = y(ind1);
    yv(2) = y(ind2);
    yv(3) = y(ind3);
    
    line(yv,xv,'color', 'r'); 
    hold on
    xlim([xminlim xmaxlim]);
    ylim([yminlim, ymaxlim]);
end


%%
