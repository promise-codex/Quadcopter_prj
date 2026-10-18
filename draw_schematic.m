function draw_schematic()
% DRAW_SCHEMATIC  Clean, labelled top-view schematic of the X-config quadrotor:
%   filled rotor discs, motor hubs, colour-coded CW/CCW spin arrows, body axes
%   and a FRONT indicator. Saves quad_schematic.png. Self-contained.
figure('Name','Quadrotor schematic','Position',[100 100 720 720]); hold on; axis equal off;
navy=[0.12 0.23 0.39]; steel=[0.18 0.42 0.64]; disk=[0.86 0.91 0.95];
cwC=[0.70 0.28 0.12]; ccwC=[0.12 0.48 0.30];
armL=1.0; rotR=0.42; spins={'CCW','CW'};
% rotor layout: x, y, number, isCW
rot = [ armL  armL 1 1; -armL  armL 2 0; -armL -armL 3 1;  armL -armL 4 0];
cols = {cwC, ccwC, cwC, ccwC};
th = linspace(0,2*pi,60);

% arms
for i=1:4, plot([0 rot(i,1)],[0 rot(i,2)],'Color',navy,'LineWidth',7); end
% central body
rectangle('Position',[-0.3 -0.3 0.6 0.6],'Curvature',0.25, ...
          'FaceColor',navy,'EdgeColor','k','LineWidth',1.2);
text(0,-0.64,sprintf('flight controller\n(IMU, ESCs)'), ...
     'HorizontalAlignment','center','VerticalAlignment','top','FontSize',9);

% rotor discs, hubs, spin arrows, labels
for i=1:4
    x=rot(i,1); y=rot(i,2); isCW=rot(i,4); col=cols{i};
    fill(x+rotR*cos(th), y+rotR*sin(th), disk, 'EdgeColor',steel,'LineWidth',2);
    plot(x,y,'k.','MarkerSize',18);
    r=rotR*0.6; span=deg2rad(280);
    if isCW, a=linspace(deg2rad(250), deg2rad(250)-span, 40);
    else,    a=linspace(deg2rad(-70), deg2rad(-70)+span, 40); end
    plot(x+r*cos(a), y+r*sin(a), 'Color',col,'LineWidth',2);
    ae=a(end); tang=[-sin(ae) cos(ae)]; if isCW, tang=-tang; end
    quiver(x+r*cos(ae), y+r*sin(ae), tang(1)*0.16, tang(2)*0.16, 0, ...
           'Color',col,'LineWidth',2,'MaxHeadSize',5,'AutoScale','off');
    text(x*1.44, y*1.44, sprintf('Rotor %d\n%s',rot(i,3),spins{isCW+1}), ...
         'HorizontalAlignment','center','Color',col,'FontWeight','bold','FontSize',10);
end

% FRONT indicator
quiver(0,1.35,0,0.42,0,'k','LineWidth',2,'MaxHeadSize',2,'AutoScale','off');
text(0,1.9,'FRONT','HorizontalAlignment','center','FontWeight','bold','FontSize',11);
% body axes
quiver(0,0,0.95,0,0,'Color',[0.75 0.22 0.17],'LineWidth',2.4,'MaxHeadSize',1.2,'AutoScale','off');
quiver(0,0,0,0.95,0,'Color',[0.15 0.68 0.38],'LineWidth',2.4,'MaxHeadSize',1.2,'AutoScale','off');
text(1.02,0,'x_B (roll)','Color',[0.75 0.22 0.17],'FontSize',11,'VerticalAlignment','middle');
text(0.04,1.06,'y_B (pitch)','Color',[0.15 0.68 0.38],'FontSize',11);
plot(0,0,'o','MarkerEdgeColor',[0.17 0.24 0.62],'MarkerFaceColor','w','MarkerSize',10,'LineWidth',1.6);
plot(0,0,'.','Color',[0.17 0.24 0.62],'MarkerSize',9);
text(-0.12,0.34,'z_B (yaw, up)','Color',[0.17 0.24 0.62],'FontSize',9.5,'HorizontalAlignment','right');

hCW  = plot(nan,nan,'-','Color',cwC,'LineWidth',2.5);
hCCW = plot(nan,nan,'-','Color',ccwC,'LineWidth',2.5);
legend([hCW hCCW],{'CW rotor spin','CCW rotor spin'},'Location','southoutside','Orientation','horizontal','Box','off');
xlim([-2.1 2.1]); ylim([-2.3 2.1]);
title('Quadrotor model — X configuration (top view)','FontSize',13,'FontWeight','bold');
print(gcf,'quad_schematic.png','-dpng','-r140');
fprintf('saved quad_schematic.png\n');
end
