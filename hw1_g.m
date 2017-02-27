clear all
global xf xf_dot xf_dot_dot
xf = 1;
xf_dot = 0;
xf_dot_dot = 0;

matrix = [];
max_time = 1;
step = 0.001;
t_togo = max_time;
x = 0;
x_dot = 0;
x_dot_dot = 0;
matrix = vertcat(matrix, [x_dot x]);

while t_togo >= 0
    x = x + step * x_dot;
    x_dot = x_dot + step * x_dot_dot;
    
    x_dot_dot_dot = getX_dot_dot_dot(x, x_dot, x_dot_dot, t_togo)
    x_dot_dot = x_dot_dot + step * x_dot_dot_dot;
    
    matrix = vertcat(matrix, [x_dot x]);
    t_togo = t_togo - step;
end

figure
plot(matrix)


function res = getC0(x)
res = x;
end

function res = getC1(x_dot)
res = x_dot;
end

function res = getC2(x_dot_dot)
res = x_dot_dot / 2.0;
end

function res = getC3(x, x_dot, x_dot_dot, t)
res = getX(x, x_dot, x_dot_dot, t) - t*getY(x, x_dot, x_dot_dot, t) - 3*t*t*getZ(x, x_dot, x_dot_dot, t); 
end

function res = getC4(x, x_dot, x_dot_dot, t)
res = getY(x, x_dot, x_dot_dot, t) - 2*t*getZ(x, x_dot, x_dot_dot, t);
end

function res = getC5(x, x_dot, x_dot_dot, t)
res = getZ(x, x_dot, x_dot_dot, t);
end


function res = getX(x, x_dot, x_dot_dot, t)
global xf 
res1 = -2*x - 2*t*x_dot - t*t*x_dot_dot + 2*xf;
res2 = 2.0 * t*t*t;
res = res1 / res2;
end

function res = getY(x, x_dot, x_dot_dot, t)
global xf xf_dot 
res1 = -6*x + 4*t*x_dot + t*t*x_dot_dot - 6*xf + 2*t*xf_dot;
res2 = 2.0 * t*t*t*t;
res = res1 / res2;
end

function res = getZ(x, x_dot, x_dot_dot, t)
global xf xf_dot xf_dot_dot
res1 = -12*x - 6*t*x_dot - t*t*x_dot_dot + 12*xf - 6*t*xf_dot + t*t*xf_dot_dot;
res2 = 2.0 * t*t*t*t*t;
res = res1 / res2;
end


function res = getX_dot_dot_dot(x, x_dot, x_dot_dot, t) 
res = 6*getC3(x, x_dot, x_dot_dot, t) ;
end


