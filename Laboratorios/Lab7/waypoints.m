clc
clear

steps = 20;

init = [0, 0];

goal = [0.5, 0];
wp = [linspace(init(1), goal(1), steps)', linspace(init(2), goal(2), steps)'];
goal = [1.5, 0.5];
wp = [wp; linspace(wp(end,1), goal(1), steps)', linspace(wp(end,2), goal(2), steps)'];
goal = [2.5, 0.5];
wp = [wp; linspace(wp(end,1), goal(1), steps)', linspace(wp(end,2), goal(2), steps)'];
goal = [3.5, 0];
wp = [wp; linspace(wp(end,1), goal(1), steps)', linspace(wp(end,2), goal(2), steps)'];
goal = [4, 0];
wp = [wp; linspace(wp(end,1), goal(1), steps)', linspace(wp(end,2), goal(2), steps)'];

allOneStringx = sprintf('%.5f,' , wp(:,1));
allOneStringx = allOneStringx(1:end-1);% strip final comma
allOneStringy = sprintf('%.5f,' , wp(:,2));
allOneStringy = allOneStringy(1:end-1);% strip final comma
allOneStringx
allOneStringy
