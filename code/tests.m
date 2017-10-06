function [] = tests(mapfile, robotstart, targetstart)

envmap = load(mapfile);

close all;

%draw the environment
image(envmap'*255);

%current positions of the target and robot
robotpos = robotstart;
targetpos = targetstart;

%load motion primitives
[mprim, res, num_angles] = loadmprim('unicycle_8angles.mprim');

%project start and target pos to the center of the cell
robotpos = fix(robotpos/res + 0.5) * res; % Akshay's Idea: add 0.5
targetpos = fix(targetpos/res + 0.5) * res;

tests(envmap, res, robotpos, targetpos, mprim);
