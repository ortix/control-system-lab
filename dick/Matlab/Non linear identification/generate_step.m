%% Generate a step input for the system after 1 second in simulation 

h = 0.01;
Tend = 10;
t = (0:h:10-h).';
step_input = zeros(length(t),1);

for i = 1:length(t)
    if t(i) > 1 && t(i) < 1.3
        step_input(i) = 1;
    end
    
    if t(i) > 1.8 && t(i) < 2.1
        step_input(i) = -1;
    end
end
figure(1)
plot(t,step_input)

U = [t step_input]