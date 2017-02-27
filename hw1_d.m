matrix = [];
max_time = 5;
step = 0.001;
t = 0;
x = 0;
while t <= max_time
    x_dot = 1 - x;
    matrix = vertcat(matrix, [x_dot x]);
    x = x + step * x_dot;
    t = t + step;
end
figure
plot(matrix)

