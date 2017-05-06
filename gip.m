clc;
close all;
clear all;
video = mmreader('Aviler\15sn.avi'); 
frameCount=video.NumberOfFrames;
variable=1;
for i=1:3:frameCount
mov(variable).framematrix=read(video,i)
variable=variable+1;

end 
temp = zeros(size(mov(1).framematrix));
[M,N] = size(temp(:,:,1));
for i = 1:75
temp = double(mov(i).framematrix) + temp;
end
background = temp/75;
background(:,1:160,:)=0;
background(:,363:480,:)=0;

centroidx = zeros(frameCount,1);
centroidy = zeros(frameCount,1);
guess = zeros(frameCount,4);
real = zeros(frameCount,4);
R=[[0.2845,0.0045]',[0.0045,0.0455]'];
H=[[1,0]',[0,1]',[0,0]',[0,0]'];
Q=0.01*eye(4);
P = 100*eye(4);
dt=1;
A=[[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];
kfinit = 0;
th = 90;
car=0;
bus=0;
for i=1:frameCount/3
imshow(mov(i).framematrix);
hold on
dene=mov(i).framematrix;
dene(:,1:160,:)=0;
dene(:,363:480,:)=0;
currentFrame = double(dene);
difference = zeros(M,N);
difference = (abs(currentFrame(:,:,1)-background(:,:,1))>th)
| (abs(currentFrame(:,:,2)-background(:,:,2))>th)
| (abs(currentFrame(:,:,3)-background(:,:,3))>th); 
labelimg = bwlabel(difference,4); 
markimg = regionprops(labelimg,['basic']);
[MM,NN] = size(markimg);

for nn = 1:MM
if markimg(nn).Area > markimg(1).Area
tmp = markimg(1);
markimg(1)= markimg(nn);
markimg(nn)= tmp;
end
end
bb = markimg(1).BoundingBox; 
xcorner = bb(1);
ycorner = bb(2);
xwidth = bb(3);
ywidth = bb(4);
cc = markimg(1).Centroid; 
centroidx(i)= cc(1);
centroidy(i)= cc(2);
hold on
rectangle('Position',[xcorner ycorner xwidth ywidth],'EdgeColor','b'); 
hold on
plot(centroidx(i),centroidy(i), 'bx'); 
kalmanx = centroidx(i)- xcorner; 
kalmany = centroidy(i)- ycorner;
if centroidx(i)<390 && centroidx(i)>130 && centroidy(i)>200  
    if ywidth<45 && xwidth<100 && centroidy(i)>245 && centroidy(i)<265
    car=car+1; 
    annotation('textbox',
    [0.15 0.05 0.3 0.1],
    'String',{['Car =' num2str(car)]},
    'FontSize',14,
    'FontName','Arial',
    'LineStyle','--',
    'EdgeColor',[1 1 0],
    'LineWidth',2,
    'BackgroundColor',[0.9  0.9 0.9],
    'Color',[0.84 0.16 0]);
    end
    
else if ywidth>100 
        bus=bus+1; 
annotation('textbox',
    [0.55 0.05 0.3 0.1],
    'String',{['Bus =' num2str(bus)]},
    'FontSize',14,
    'FontName','Arial',
    'LineStyle','--',
    'EdgeColor',[1 1 0],
    'LineWidth',2,
    'BackgroundColor',[0.9  0.9 0.9],
    'Color',[0.84 0.16 0]);
    end
end
   

if kfinit == 0
guess =[centroidx(i),centroidy(i),0,0]' ;
else
guess = A*real(i-1,:)';
end
kfinit = 1;
Ppre = A*P*A' + Q;
K = Ppre*H'/(H*Ppre*H'+R);
real(i,:) = (guess + K*([centroidx(i),centroidy(i)]' - H*guess))';
P = (eye(4)-K*H)*Ppre;
hold on
 rectangle('Position',[(real(i,1)-kalmanx)
(real(i,2)-kalmany) xwidth ywidth],'EdgeColor','r','LineWidth',1.5);
hold on
plot(real(i,1),real(i,2), 'rx','LineWidth',1.5);
drawnow;
end
