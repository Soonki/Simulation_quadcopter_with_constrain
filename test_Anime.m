close All
plot(0,0,'ro');
hold on

d=50;
j=1:d:length(T);
D=Dynamics(zeros(12:1));
% Block specification
Lx = D.L;
Ly = 0.01;
Lz = 0.01;

root=sqrt(2);
R45=[1/root -1/root 0;
     1/root 1/root 0;
     0        0    1;];

p_center1=[Lx/2, Ly/2, Lz/2]*R45';
p_center2=[Ly/2, Lx/2, Lz/2]*R45';
p1_center=[Ly/2, Ly/2, D.L1/2];
p2_center=[D.L1/2, Ly/2, Ly/2];

n_time = length(T)-1;

% Compute propagation of vertices and patches
for i_time=1:d:n_time
    
    R=Rotation(ph(i_time),th(i_time),ps(i_time));
    p=[xr(i_time) yr(i_time) zr(i_time)];
    R1=[cos(et1(i_time)) 0 sin(et1(i_time));0 1 0;-sin(et1(i_time)) 0 cos(et1(i_time))];
    R2=[cos(et1(i_time)+et2(i_time)) 0 sin(et1(i_time)+et2(i_time));0 1 0;-sin(et1(i_time)+et2(i_time)) 0 cos(et1(i_time)+et2(i_time))];
    p1=p+(R*R1*[0;0;D.l1])';
    p2=p+(R*R1*[0;0;D.L1])'+(R*R2*[D.l2;0;0])';
    
    %---------------------------------------------------------
    H=mean(Uf(i_time,1:4));
    T1=((Uf(i_time,1))/H-1)*2+H*0.00003;
    arrow1=[0.001,0.001,-T1];
    T2=((Uf(i_time,2))/H-1)*2+H*0.00003;
    arrow2=[0.001,0.001,-T2];
    T3=((Uf(i_time,3))/H-1)*2+H*0.00003;
    arrow3=[0.001,0.001,-T3];
    T4=((Uf(i_time,4))/H-1)*2+H*0.00003;
    arrow4=[0.001,0.001,-T4];
    
    
    VertexData1(:,:,i_time) = GeoVerMakeBlock(p-(R*p_center1')',R*R45,[Lx,Ly,Lz]);
    VertexData2(:,:,i_time) = GeoVerMakeBlock(p-(R*p_center2')',R*R45,[Ly,Lx,Lz]);
    
    VertexData3(:,:,i_time) = GeoVerMakeBlock(p1-(R*R1*p1_center')',R*R1,[Ly,Ly,D.L1]);
    VertexData4(:,:,i_time) = GeoVerMakeBlock(p2-(R*R2*p2_center')',R*R2,[D.L2,Ly,Ly]);
    
    VertexDataT1(:,:,i_time) = GeoVerMakeBlock(p+[Lx/2 0 0]*R45'*R',R,arrow1);
    VertexDataT2(:,:,i_time) = GeoVerMakeBlock(p+[0 Lx/2 0]*R45'*R',R,arrow2);
    VertexDataT3(:,:,i_time) = GeoVerMakeBlock(p+[-Lx/2 0 0]*R45'*R',R,arrow3);
    VertexDataT4(:,:,i_time) = GeoVerMakeBlock(p+[0 -Lx/2 0]*R45'*R',R,arrow4);
    
    [Xg1,Yg1,Zg1] = GeoPatMakeBlock(VertexData1(:,:,i_time));
    [Xg2,Yg2,Zg2] = GeoPatMakeBlock(VertexData2(:,:,i_time));
    
    [Xg3,Yg3,Zg3] = GeoPatMakeBlock(VertexData3(:,:,i_time));
    [Xg4,Yg4,Zg4] = GeoPatMakeBlock(VertexData4(:,:,i_time));
    
    [XgT1,YgT1,ZgT1] = GeoPatMakeBlock(VertexDataT1(:,:,i_time));
    [XgT2,YgT2,ZgT2] = GeoPatMakeBlock(VertexDataT2(:,:,i_time));
    [XgT3,YgT3,ZgT3] = GeoPatMakeBlock(VertexDataT3(:,:,i_time));
    [XgT4,YgT4,ZgT4] = GeoPatMakeBlock(VertexDataT4(:,:,i_time));
    
    
    PatchData_X1(:,:,i_time) = Xg1;
    PatchData_Y1(:,:,i_time) = Yg1;
    PatchData_Z1(:,:,i_time) = Zg1;
    PatchData_X2(:,:,i_time) = Xg2;
    PatchData_Y2(:,:,i_time) = Yg2;
    PatchData_Z2(:,:,i_time) = Zg2;
    
    PatchData_X3(:,:,i_time) = Xg3;
    PatchData_Y3(:,:,i_time) = Yg3;
    PatchData_Z3(:,:,i_time) = Zg3;
    
    PatchData_X4(:,:,i_time) = Xg4;
    PatchData_Y4(:,:,i_time) = Yg4;
    PatchData_Z4(:,:,i_time) = Zg4;
    
    PatchData_XT1(:,:,i_time) = XgT1;
    PatchData_YT1(:,:,i_time) = YgT1;
    PatchData_ZT1(:,:,i_time) = ZgT1;
    PatchData_XT2(:,:,i_time) = XgT2;
    PatchData_YT2(:,:,i_time) = YgT2;
    PatchData_ZT2(:,:,i_time) = ZgT2;
    PatchData_XT3(:,:,i_time) = XgT3;
    PatchData_YT3(:,:,i_time) = YgT3;
    PatchData_ZT3(:,:,i_time) = ZgT3;
    PatchData_XT4(:,:,i_time) = XgT4;
    PatchData_YT4(:,:,i_time) = YgT4;
    PatchData_ZT4(:,:,i_time) = ZgT4;
end

% Draw initial figure
figure(1);
%set(gcf,'Renderer','OpenGL'); 
%r=10;
%a=0:pi/20:2*pi; b=0:pi/20:2*pi;
%h = patch(r*cos(a).*cos(b),r*cos(a).*sin(b),r*sin(a),'b');
%set(h,'EraseMode','normal');
%set(h,'FaceLighting','phong');

h1 = patch(PatchData_X1(:,:,1),PatchData_Y1(:,:,1),PatchData_Z1(:,:,1),'g');
h2 = patch(PatchData_X2(:,:,1),PatchData_Y2(:,:,1),PatchData_Z2(:,:,1),'y');
h1.EdgeColor='g';
h2.EdgeColor='y';

h3 = patch(PatchData_X3(:,:,1),PatchData_Y3(:,:,1),PatchData_Z3(:,:,1),'r');
h3.EdgeColor='r';
h4 = patch(PatchData_X4(:,:,1),PatchData_Y4(:,:,1),PatchData_Z4(:,:,1),'r');
h4.EdgeColor='r';



set(h1,'FaceLighting','phong','EdgeLighting','phong');
set(h1,'EraseMode','normal');
set(h2,'FaceLighting','phong','EdgeLighting','phong');
set(h2,'EraseMode','normal');
set(h3,'FaceLighting','phong','EdgeLighting','phong');
set(h3,'EraseMode','normal');
set(h4,'FaceLighting','phong','EdgeLighting','phong');
set(h4,'EraseMode','normal');

hT1 = patch(PatchData_XT1(:,:,1),PatchData_YT1(:,:,1),PatchData_ZT1(:,:,1),'r');
hT1.EdgeColor='r';
hT2 = patch(PatchData_XT2(:,:,1),PatchData_YT2(:,:,1),PatchData_ZT2(:,:,1),'r');
hT2.EdgeColor='r';
hT3 = patch(PatchData_XT3(:,:,1),PatchData_YT3(:,:,1),PatchData_ZT3(:,:,1),'r');
hT3.EdgeColor='r';
hT4 = patch(PatchData_XT4(:,:,1),PatchData_YT4(:,:,1),PatchData_ZT4(:,:,1),'r');
hT4.EdgeColor='r';



% Axes settings
figure(1)
xlabel('x','FontSize',14);
ylabel('y','FontSize',14);
zlabel('z','FontSize',14);
set(gca,'FontSize',14);
set(gca,'ZDir','rev');
set(gca,'YDir','rev');
axis vis3d equal;
view([-37.5,30]);
camlight;
grid on;
 xlim([-1,1]);
 ylim([-1,1]);
 zlim([-1,1]);
 
 
% Animation Loop
for i_time=1:d:n_time
    figure(1)
    p=[xr(i_time) yr(i_time) zr(i_time)];
    xlim([p(1)-0.5,p(1)+0.5]);
    ylim([p(2)-0.5,p(2)+0.5]);
   zlim([p(3)-0.5,p(3)+0.5]);
    %set(h,'XData',p(1));
    %set(h,'YData',p(2));
    %set(h,'ZData',p(3));
    %annotation('textbox',dim,'String',num2str(i_time),'FitBoxToText','on');
    set(h1,'XData',PatchData_X1(:,:,i_time));
    set(h1,'YData',PatchData_Y1(:,:,i_time));
    set(h1,'ZData',PatchData_Z1(:,:,i_time));
    set(h2,'XData',PatchData_X2(:,:,i_time));
    set(h2,'YData',PatchData_Y2(:,:,i_time));
    set(h2,'ZData',PatchData_Z2(:,:,i_time));
    
    set(h3,'XData',PatchData_X3(:,:,i_time));
    set(h3,'YData',PatchData_Y3(:,:,i_time));
    set(h3,'ZData',PatchData_Z3(:,:,i_time));
    set(h4,'XData',PatchData_X4(:,:,i_time));
    set(h4,'YData',PatchData_Y4(:,:,i_time));
    set(h4,'ZData',PatchData_Z4(:,:,i_time));
    
    set(hT1,'XData',PatchData_XT1(:,:,i_time));
    set(hT1,'YData',PatchData_YT1(:,:,i_time));
    set(hT1,'ZData',PatchData_ZT1(:,:,i_time));
    set(hT2,'XData',PatchData_XT2(:,:,i_time));
    set(hT2,'YData',PatchData_YT2(:,:,i_time));
    set(hT2,'ZData',PatchData_ZT2(:,:,i_time));
    set(hT3,'XData',PatchData_XT3(:,:,i_time));
    set(hT3,'YData',PatchData_YT3(:,:,i_time));
    set(hT3,'ZData',PatchData_ZT3(:,:,i_time));
    set(hT4,'XData',PatchData_XT4(:,:,i_time));
    set(hT4,'YData',PatchData_YT4(:,:,i_time));
    set(hT4,'ZData',PatchData_ZT4(:,:,i_time));
    
    
    MM((i_time+d-1)/d)=getframe(gcf);
    drawnow;
end

 v = VideoWriter(char(yyyymmdd(datetime)+".avi"))
 open(v)
 writeVideo(v,MM)
 close(v)