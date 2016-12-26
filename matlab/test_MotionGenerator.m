
clear all
close all
clc

MG = MotionGenerator(200,500,0);

r = randi([-180,180],5,1);
tr = linspace(0,5,length(r));

t = linspace(min(tr),max(tr),100);
u = interp1(tr,r,t,'next');


pos = [];
vel = [];
acc = [];
tt = [];


for kk = 1:length(u)
    
    MG.update(u(kk),t(kk));    
    pos = [pos; MG.pos];
    vel = [vel; MG.vel];
    acc = [acc; MG.acc];
    
end

lw = 1.5;
figure(1),
s1 = subplot(311);
stairs(t, u, '-','LineWidth', lw); hold on
plot(t, pos, '.-', 'LineWidth', lw, 'MarkerSize',10);
ylabel('Position [units]'); legend('reference','actual');
grid
s2 = subplot(312);
plot(t, vel, '.-', 'LineWidth', lw, 'MarkerSize',10); hold on
ylabel('Velocity [units/s]');
grid
s3 = subplot(313);
stairs(t, acc, '.-', 'LineWidth', lw, 'MarkerSize',10); hold on
grid
xlabel('Time [s]')
ylabel('Acceleration [units/s^2]');
linkaxes([s1 s2 s3],'x');
