function MapLabel = identificar(Caracteristicas,Thdist)
etiqueta = 1;
MapT=[];
[lenCar,cc] = size(Caracteristicas);
if (~isempty(lenCar))
  for j=1:lenCar-1
      vect = Caracteristicas(j,1:2)- Caracteristicas(j+1,1:2);
      if norm(vect)<Thdist
          CarLabel = [Caracteristicas(j,1:2) Caracteristicas(j,3) etiqueta];
          MapT = [MapT;CarLabel];
      else
          CarLabel = [Caracteristicas(j,1:2) Caracteristicas(j,3) etiqueta];
          MapT = [MapT;CarLabel];
          etiqueta = etiqueta+1;
      end
  end
  if lenCar==0
      Caracteristicas(1,1:2) 
      Caracteristicas(1,3) 
      etiqueta
    CarLabel = [0 0 0];
  else
    CarLabel = [Caracteristicas(lenCar,1:2) Caracteristicas(lenCar,3) etiqueta];
  end
  MapT = [MapT;CarLabel];
  MapLabel = MapT;
end