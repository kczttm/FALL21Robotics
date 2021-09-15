function mpj1_draw(ax)
% Draw robotics 1 miniproject 1 nonmovalbles
% a)
R_10 = [0, 1; -1, 0];
p10_1 = [-0.5; 4];
p01_1 = -p10_1;
R_01 = R_10';
p01_0 = R_01 * p01_1;
% b)
p21_0 = [3;0];
p02_0 = p01_0 - p21_0;
% c)
p03_0 = [2.5;2.5];
R_03 = rot2(pi/4);
% d)
p04_0 = p02_0 + [0;3.5];
R_04 = rot2(-pi/2);
%% Create Walls
Nwall = collisionBox(5,0.1,1);
Swall = collisionBox(5,0.1,1);
Ewall = collisionBox(0.1,5,1);
Wwall = collisionBox(0.1,5,1);
% Wall Poses
Nwall.Pose(:,4) = [5/2;5+0.1/2;0;1];
Swall.Pose(:,4) = [5/2;-0.1/2;0;1];
Ewall.Pose(:,4) = [-0.1/2;5/2;0;1];
Wwall.Pose(:,4) = [5+0.1/2;5/2;0;1];
% Wall show
hold off
show(Nwall,'Parent',ax)
hold on
show(Swall,'Parent',ax)
show(Ewall,'Parent',ax)
show(Wwall,'Parent',ax)
view(0,90)
xlim([-0.5,5.5])
ylim([-0.5,5.5])
zlim([0,1])
quiver([0;0],[0;0],[1;0],[0;1],'LineWidth',5,'Color','b')
xlabel('X_0 (m)')
ylabel('Y_0 (m)')
%% Non-movables
Table = collisionBox(0.5,0.5,1);
Table.Pose(1:2,1:2) = R_03;
Table.Pose(1:2,4) = p03_0;
[~,patchTable] = show(Table,'Parent',ax);
patchTable.FaceColor = 'g';
patchTable.EdgeColor = 'none';
quiver([2.5;2.5],[2.5;2.5],[0.3;-0.3],[0.3;0.3],'LineWidth',2,'Color','k')

Person = collisionCylinder(0.2,1);
Person.Pose(1:2,4) = p02_0;
[~,patchPerson] = show(Person,'Parent',ax);
patchPerson.FaceColor = 'cyan';
patchPerson.EdgeColor = 'none';
person_X = p02_0(1);
person_Y = p02_0(2);
quiver([person_X;person_X],[person_Y;person_Y],[0.4;0],[0;0.4],'LineWidth',2,'Color','k')

Shelf = collisionBox(0.8,0.3,1);
Shelf.Pose(1:2,1:2) = R_04;
Shelf.Pose(1:2,4) = p04_0;
[~,patchShelf] = show(Shelf,'Parent',ax);
patchShelf.FaceColor = 'y';
patchShelf.EdgeColor = 'none';
quiver([p04_0(1);p04_0(1)],[p04_0(2);p04_0(2)],[0;0.3],[-0.7;0],'LineWidth',2,'Color','k')
%% Annotation
xtext = [0.2, 1.2, 2.7, 1];
ytext = [-0.4, 0.7, 2.3, 4.6];
t = {'(E_0, O_0)', '(E_2, O_2)', '(E_3, O_3)', '(E_4, O_4)'};
text(xtext,ytext,t);
end