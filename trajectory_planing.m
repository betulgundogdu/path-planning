clear
clc
trajectory_planning(150,20);

function trajectory_planning(targetX, targetY)

%area coordinates
area_posx = 200;
area_posy = 100;

% create ares include all elements are zero
area = zeros(area_posy,area_posx);

% create & add static obstacles on area
area(65:75, 50:100) = 1;
area(60:100, 140:165) = 1;
area(15:65, 70:100) = 1;

%target positions
robotX = 1;
robotY = 1;

area(targetX, targetY) = -1;
area(robotX,robotY) = 2;

map = area;
count = 1;
current = 2;

x1 = robotX;
y1 = robotY;
x2 = robotX;
y2 = robotY;
x = robotX;
y = robotY;
flag = 0;
while flag == 0
    if x1 < 1
        x1 = 1;
    end
    if y1 < 1
        y1 = 1;
    end
    if x2 > area_posx
        x2 = area_posx;
    end
    if y2 > area_posy
        y2 = area_posy;
    end
    for i=x1:x2
        for j=y1:y2
            if(map(i,j) == -1)
                flag = 1;
                break;
            end
            if(map(i,j) == 0)
                map(i,j) = current;
            end
        end
        if flag == 1
            break;
        end
    end
    x1 = x - count;
    x2 = x + count;
    y1 = y - count;
    y2 = y + count;
    count = count + 1;
    current = current + 1;
end
targetValue = current - 1;
map(targetX, targetY) = targetValue;
disp(map);

%finding robot path with calculated matrix
queue = zeros(200, 2);
initValue = 2;
count2 = 1;
queue(count2,:) = [robotX robotY];
incX = 1;
incY = 1;
pathMatrix = [];
if(robotX > targetX)
    incX = -1;
end
if(robotY > targetY)
    incY = -1;
end
move = 0;
currentx = robotX;
currenty = robotY;
nextX = x + incX;
nextY = y + incY;
while move == 0
    count2 = count2 + 1;
    initValue = initValue + 1;
    if initValue <= targetValue
        if(currentx ~= targetX && currenty ~= targetY && map(nextX, nextY) == initValue)
            queue(count2,:) = [nextX, nextY];
            currentx = nextX;
            currenty = nextY;
            nextX = currentx + incX;
            nextY = currenty + incY;
        elseif(currentx ~= targetX) && map(nextX, currenty) == initValue
            queue(count2,:) = [nextX, currenty];
            currentx = nextX;
            nextX = currentx + incX;
        elseif(currenty ~= targetY) && map(currentx, nextY) == initValue
            queue(count2,:) = [currentx, nextY];
            currenty = nextY;
            nextY = currenty + incY;
        elseif(currentx ~= targetX && currenty ~= targetY && map(nextX, nextY) == initValue - 1)
            queue(count2,:) = [nextX, nextY];
            currentx = nextX;
            currenty = nextY;
            nextX = currentx + incX;
            nextY = currenty + incY;
        elseif(currentx ~= targetX) && map(nextX, currenty) == initValue - 1
            queue(count2,:) = [nextX, currenty];
            currentx = nextX;
            nextX = currentx + incX;
        elseif(currenty ~= targetY) && map(currentx, nextY) == initValue - 1
            queue(count2,:) = [currentx, nextY];
            currenty = nextY;
            nextY = currenty + incY;                      
        else
            move = 1;
            disp(queue);
        end
    else
        pathMatrix = queue(1:count2 - 1,1:2);
        disp("Path is:");
        disp(pathMatrix);
        break;
    end
end

% --visualization
figure
%axes
h1 = axes;
imagesc(map);
hold all;
%plotting path
if size(pathMatrix) ~= 0
    plot(pathMatrix(:,1), pathMatrix(:,2));
else
    disp("path not found");
end
plot(robotX, robotY,'o','markerfacecolor','g');
plot(targetX,targetY,'o','markerfacecolor','r');
colorbar;
set(h1,'YDir','normal');% y coordinate changed as increasing bottom to up
% coordinate ticks
xticks(0:10:area_posx);
yticks(0:10:area_posy);
% minor grid lines
grid(gca,'minor')
h1.XAxis.MinorTickValues = 0:1:area_posx;
h1.YAxis.MinorTickValues = 0:1:area_posy;
grid on

end