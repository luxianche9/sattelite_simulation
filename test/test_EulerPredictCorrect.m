clear;
clc;

rv0 = [0,1];
t0 = 0;
dt = 0.1;
time = 10;

rv = EulerPredictCorrect(time, dt, rv0, @acceleration);

figure
plot(rv(:,1))
legend('r')
figure
plot(rv(:,2))
legend('v')

function drvdt = acceleration(~, rv)
    r = rv(1);
    v = rv(2);
    
    drdt = v;
    dvdt = 0;
    drvdt = [drdt, dvdt];
end
