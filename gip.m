%Kod baslangici
%Kalman Filtresi uygulaniyor
%Bu örnekte, Kalman filtresinin iki ayr? fazda çal??t??? dü?ünülebilir:
%tahmin et ve güncelle. Tahmin etme faz?nda, kamyonun eski yeri Newton'?n hareket yasalar? göre
%de?i?tirilecek (dinamik veya "durum de?i?tirme" modeli) art? gaz pedal? ve direksiyon taraf?ndan
%üretilen tüm de?i?iklikler kat?lacak. Sadece bir pozisyon tahmini hesaplanmayacak, ancak yeni bir 
%kovaryans da hesaplanacakt?r. Belki de kovaryans kamyonun h?z? ile orant?l?d?r; 
%çünkü yüksek h?zlarda parakete hesab?n?n hassasl???ndan daha az eminiz ancak çok yava? hareket 
%etti?inde baya eminiz. Sonra, güncelleme faz?nda, kamyonun pozisyonunun bir ölçümü GPS biriminden al?n?r.
%Bu ölçümle beraber bir miktar belirsizlik de gelir ve bunun koyaryans?n?n önceki fazdan gelen tahminin
%kovaryans?na oran?, yeni ölçümün güncellenen tahmini ne kadar etkileyece?ini belirler.
%?deal olarak, parakete hesab? tahminleri gerçek pozisyondan uzakla?t?kça,
%GPS ölçümleri pozisyon tahminlerini gerçek pozisyona do?ru, h?zl?ca de?i?im ve 
%gürültülü olmayacak ?ekilde çeker.
clc;%Temizle
close all;%calisanlari kapat 
clear all; %temizle
video = mmreader('Aviler\15sn.avi'); %avi formatinda videolari yuklemek icin
%frameSayisi = length(video);%video uzunlugu
frameSayisi=video.NumberOfFrames;%video icerisindeki frame sayisini verir
degisken=1;
for i=1:3:frameSayisi
mov(degisken).framematrisi=read(video,i)
degisken=degisken+1;%vidyonun belirtilen i ninci frameni mov matrisinde framematris sutununda
%i ninci satira ekle , rgb degerlerini 3 matris bloklari halinde tum pixelleri
%gosterecek sekilde isler.

end 
temp = zeros(size(mov(1).framematrisi));%mov matrisinde 1. framein boyutlarinda sifir ile dolu matris yarat ve
%temp matrisine ata
[M,N] = size(temp(:,:,1));%temp matrisinin ilk sayfadaki satir ve sutun sayisini M ve N degiskenlerine ata 
for i = 1:75
temp = double(mov(i).framematrisi) + temp;
%mov matrisinin ilk 10 framematrisi degerleri toplayip tempe atiyor.
end
arkaPlan = temp/75;%temp matrisini 10 a boler yolun bos goruntusunu alir, nesnenin arkaplani
arkaPlan(:,1:160,:)=0;
arkaPlan(:,363:480,:)=0;

centroidx = zeros(frameSayisi,1);%frame sayisi * 1 lik matris yap sifir ile doldur
centroidy = zeros(frameSayisi,1);%frame sayisi * 1 lik matris yap sifir ile doldur
tahmin = zeros(frameSayisi,4);%frame sayisi * 4 luk matris yap sifir ile doldur 1.faz
gercek = zeros(frameSayisi,4);%frame sayisi * 4 luk matris yap sifir ile doldur  2.faz
R=[[0.2845,0.0045]',[0.0045,0.0455]'];%Ölçme gürültüsü kovaryans matrisi
H=[[1,0]',[0,1]',[0,0]',[0,0]']; % donusum matrisi
Q=0.01*eye(4);%4 * 4 luk birim matris olustur 0.01 ile carp.
P = 100*eye(4);% 4 * 4 luk birim matris olustur 100 ile carp.
dt=1;
A=[[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];
kfinit = 0;
th = 90; %threshold=esik
otomobil=0;
otobus=0;
for i=1:frameSayisi/3
imshow(mov(i).framematrisi);%i ninci framematrisini gosterir
hold on
dene=mov(i).framematrisi;
dene(:,1:160,:)=0;
dene(:,363:480,:)=0;
suAnkiResim = double(dene);%gosterilen frame double a cevrilir. suAnkiResim degiskenine aktarilir
goruntuFarki = zeros(M,N);%120 * 160 lik matrisi sifirlar ile doldurur goruntuFarki degiskenine aktarir
goruntuFarki = (abs(suAnkiResim(:,:,1)-arkaPlan(:,:,1))>th) ...
| (abs(suAnkiResim(:,:,2)-arkaPlan(:,:,2))>th) ...
| (abs(suAnkiResim(:,:,3)-arkaPlan(:,:,3))>th); % abs= mutlak deger, su anki framede yer alan resim
%ile goruntunun ilk hali arasindaki fark incelenir,farkin esik degerinin uzerinde olup olmadigina bakiyor
labelimg = bwlabel(goruntuFarki,4); %pixel baglanti durumu-4 veya 8
markimg = regionprops(labelimg,['basic']); %regionprops=alan ozellikleri,  area,centroid,bounding box ozelliklerini verir
[MM,NN] = size(markimg);

for nn = 1:MM
if markimg(nn).Area > markimg(1).Area
tmp = markimg(1);
markimg(1)= markimg(nn);
markimg(nn)= tmp;
end
end
bb = markimg(1).BoundingBox; %ekranda cizilen kutunun degerlerini hesapliyoruz
xcorner = bb(1); %sol ust kosenin x degeri
ycorner = bb(2); %sol ust kosenin y degeri
xwidth = bb(3); %x uzunlugu
ywidth = bb(4); % y uzunlugu
cc = markimg(1).Centroid; % isaretlenen degerin agirlik merkezi bulunur
centroidx(i)= cc(1);
centroidy(i)= cc(2);
hold on
rectangle('Position',[xcorner ycorner xwidth ywidth],'EdgeColor','b'); %mavi kareyi cizdiriyoruz
hold on
plot(centroidx(i),centroidy(i), 'bx'); %mavi x isaretini cizdiriyoruz
kalmanx = centroidx(i)- xcorner; %nesnenin ortasindan koselerine olan mesafeleri
kalmany = centroidy(i)- ycorner;
if centroidx(i)<390 && centroidx(i)>130 && centroidy(i)>200  %130 ile 400 araliginda ise y yonunde 160 uzerinde ise
    if ywidth<45 && xwidth<100 && centroidy(i)>245 && centroidy(i)<265
    otomobil=otomobil+1; 
    annotation('textbox',...
    [0.15 0.05 0.3 0.1],...
    'String',{['Otomobil =' num2str(otomobil)]},...
    'FontSize',14,...
    'FontName','Arial',...
    'LineStyle','--',...
    'EdgeColor',[1 1 0],...
    'LineWidth',2,...
    'BackgroundColor',[0.9  0.9 0.9],...
    'Color',[0.84 0.16 0]);
    end
    
else if ywidth>100 
        otobus=otobus+1; 
annotation('textbox',...
    [0.55 0.05 0.3 0.1],...
    'String',{['Otobus =' num2str(otobus)]},...
    'FontSize',14,...
    'FontName','Arial',...
    'LineStyle','--',...
    'EdgeColor',[1 1 0],...
    'LineWidth',2,...
    'BackgroundColor',[0.9  0.9 0.9],...
    'Color',[0.84 0.16 0]);
    end
end
   

if kfinit == 0
tahmin =[centroidx(i),centroidy(i),0,0]' ;
else
tahmin = A*gercek(i-1,:)';
end
kfinit = 1;
Ppre = A*P*A' + Q;
K = Ppre*H'/(H*Ppre*H'+R);
gercek(i,:) = (tahmin + K*([centroidx(i),centroidy(i)]' - H*tahmin))';
P = (eye(4)-K*H)*Ppre;
hold on
 rectangle('Position',[(gercek(i,1)-kalmanx)...
(gercek(i,2)-kalmany) xwidth ywidth],'EdgeColor','r','LineWidth',1.5);
hold on
plot(gercek(i,1),gercek(i,2), 'rx','LineWidth',1.5);
drawnow;
end