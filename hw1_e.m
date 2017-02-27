matrix = [];
max_time = 2;
step = 0.001;
t = 0;
x = 0;
x_dot = 0;
while t <= max_time
    x_dot_dot = 25 * (6*(1 - x) - x_dot);
    matrix = vertcat(matrix, [x_dot x]);
    x_dot = x_dot + step * x_dot_dot;
    x = x + step * x_dot;
    t = t + step;
end
figure
plot(matrix)

