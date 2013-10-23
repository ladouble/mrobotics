%-----------------------------------------------------------
% Berechnung der TCP Bahn mit der Matlab Robotics Toolbox
%-----------------------------------------------------------

% in dieser Berechnung wird von der Xhome Position ausgegangen und zum
% Punkt XP1 gefahren dies geschieht in einer Liniearbewegung. 
% Diese Funktion stimmt nicht mit der Realität überein da die Drehwinkel am
% ToolCenterPoint genauer überprüft werden müssen. Um welche Drehkonvention
% es sich handelt.
tic

addpath('../robot_lib');
%addpath kann vom Compiler nicht Interpretiert werden Version 2010a
%Unix Pfad
run('../robot_lib/startup.m');
run('../robot_lib/Kuka240_kr240.m')

%Ausgangspunkt xhome
xhome=[87.78,-131.32,131.97,-0.73,33.70,-1.83];
xhome=deg2rad(xhome);
xhome(1,3)=xhome(1,3)-pi/2; %Offset des 3.Armgelenkes
xhome=xhome*(-1);
forward=fkine(kr240,xhome);
xhomerpy=Tr2rpy(forward(1:3,1:3)); 

%Zielpunkt xp1
xp1=[1516.340, -9.563,149.7,-177.15,20.9,-178.39];
%xp1_drehwinkel=zeros(1,3);
xp1_drehwinkel=deg2rad(xp1(1,4:6));
%xp1_drehwinkel=[2.35 1.573 0.78];
lin1=[xp1(1:3);forward(1:3,4)'];

drehwinkel=[xp1_drehwinkel;xhomerpy];

bahn=[1,2];
%% Berechnung der Linearbewegung des ToolCenterPoints


% for i=1:size(lin1,1)
%     lin1(i,:)=nullpoint(:)+lin1(i,:)';
% end

hold on
N=norm(lin1(bahn(1,1),:)-lin1(bahn(1,2),:));

disp('Bahn wird berechnet');% Ausgabe auf der Konsole


% Aufteilung der Linearstrecke in mehreren Segmenten
for i=2:size(bahn,2)
    line([lin1(bahn(1,i),1) lin1(bahn(1,i-1),1)],[lin1(bahn(1,i),2) lin1(bahn(1,i-1),2)],[lin1(bahn(1,i),3) lin1(bahn(1,i-1),3)],'Marker','.','LineStyle','-')
    T0=transl(lin1(bahn(1,i),1),lin1(bahn(1,i),2),lin1(bahn(1,i),3));
    T1=transl(lin1(bahn(1,i-1),1),lin1(bahn(1,i-1),2),lin1(bahn(1,i-1),3));
    TC = ctraj(T0, T1, N);
    plot3(lin1(bahn(1,i),1),lin1(bahn(1,i),2),lin1(bahn(1,i),3));
end


disp('Animation wird berechnet');
%Roboter Animation
q_start=zeros(size(TC,3),6);

%% Winkel f�r die Robotertransformationsmatrix

%Interpolerationsschritte f�r die einzelnen Drehwinkel um eine flüßige
%drehung zu simulieren
% TCP Drehungwinkel
interp_drehwinkel(:,1)=interp1(1:2,[drehwinkel(1,1),drehwinkel(2,1)],1:3.7300e-004:2,'linear');
interp_drehwinkel(:,2)=interp1(1:2,[drehwinkel(1,2),drehwinkel(2,2)],1:3.7300e-004:2,'linear');
interp_drehwinkel(:,3)=interp1(1:2,[drehwinkel(1,3),drehwinkel(2,3)],1:3.7300e-004:2,'linear');

for i=1:size(TC,3)
    TC(1:3,1:3,i)=rotx(interp_drehwinkel(i,1))*roty(interp_drehwinkel(i,2))*rotz(interp_drehwinkel(i,3));
end

% die inverse Kinematic funktioniert nicht richtig
q_start(1,:)=xhome;
for i=2:size(TC,3)
    q_start(i,:)=ikine(kr240,TC(:,:,i),q_start(i-1,:));
end

plot(kr240,q_start);
hold off

%--------------------------------------------------------------
    %desto hoeher der Wert desto genauer ist die berechnung des Plots hier
    %habe ich die Zahl 500 gewählt
    
    %Berechnung der Achsenwinkel
    %[q qd qdd] = jtraj(q_start(1,:),q_start(2200,:),500);

    [q qd qdd] = jtraj(xhome,q_start(2200,:),500);
    tau = rne(kr240, q, qd, qdd);
    figure(2)
    grid on
    plot(tau);
    xlabel('Zeitachse [s]');
    ylabel('Drehmoment[N*mm]');
    legend('Armteil 1','Armteil 2','Armteil 3','Armteil 4','Armteil 5','Armteil 6',0);
    
    toc
