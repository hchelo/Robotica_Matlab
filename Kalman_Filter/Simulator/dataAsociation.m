function MAPA = dataAsociation(MedPost,MAPA)
[lMapa c] = size(MAPA);
[lpostes c] = size(MedPost);
Th = 0.6;
etiqueta = 1;
newPoste = [];
d = [];
% Get distance Map and Characteristics
for i = 1:lpostes
    for j = 1:lMapa
        d(j) = norm(MedPost(i,1:2)-MAPA(j,1:2));
        if d(j)<Th
            MAPA(j,1:3) = 0.5*(MedPost(i,1:3)+MAPA(j,1:3)); %Update means
        end
    end
    % identify New Point on Map
    if min(d)<Th
    else
        newPoste = [newPoste;MedPost(i,:)];
    end
    d = []; 
end
if(~isempty(newPoste))
    numCar = size(MAPA,1)+1;
    MAPA = [MAPA;[newPoste(:,1:3) numCar]];
else
    MAPA = MAPA;
end
  