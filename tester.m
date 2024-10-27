clf;
hold on;
lighting none;
PlaceObject('UR3eLink0a.PLY',[0,0,0]);
PlaceObject('UR3eLink1a.PLY',[0,0,0.15185]);
PlaceObject('UR3eLink2a.PLY',[-0.24355,0,0.15185]);
PlaceObject('UR3eLink3a.PLY',[-0.24355-0.2132,0,0.15185]);
PlaceObject('UR3eLink4a.PLY',[-0.24355-0.2132,-0.11235,0.15185]);
PlaceObject('UR3eLink5a.PLY',[-0.24355-0.2132,-0.11235,0.15185-0.08535]);
PlaceObject('UR3eLink6a.PLY',[-0.24355-0.2132,-0.11235-0.0819,0.15185-0.08535]);
% 
% % PlaceObject('Sensors and control mount v3.PLY',[-0.24355-0.2132,-0.11235-0.0819,0.15185-0.08535]);
% PlaceObject('IR_PenMount v6.PLY',[-0.24355-0.2132,-0.11235-0.0819,0.15185-0.08535]);
% PlaceObject('IR_PenTube v7.PLY',[-0.24355-0.2132,-0.11235-0.0819,0.15185-0.08535]);

PlaceObject('table.PLY',[0,0,0]);

axis equal;
teach