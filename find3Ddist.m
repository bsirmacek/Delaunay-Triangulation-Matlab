function dist = find3Ddist(x,y,z,center)

for(i = 1:size(x,2))
	dist(i) = sqrt((x(i)-center(2))^2 + (y(i)-center(1))^2 + (z(i)-center(3))^2);
end