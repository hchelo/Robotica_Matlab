function Media_Label = etiquetar(Media_Label,MapaIn,MAPA)
[lMapaIn c] = size(MapaIn);
[lMAPA c] = size(MAPA);
Th = 0.6;
etiqueta = 1;
d = [];
% Get distance Map and Characteristics
for i = 1:lMapaIn 
    for j = 1:lMAPA
        d(j) = norm(MapaIn(i,1:2)-MAPA(j,1:2));
    end
    [val idx] = min(d);
    
    if(val<Th)
        Media_Label = [Media_Label;[MapaIn(i,1:3) idx]];
    else
        Media_Label = [Media_Label;[MapaIn(i,1:3) lMAPA+1]];
        disp('new')
    end
    
end
  