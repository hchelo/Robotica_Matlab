function zout = ztestimado(k,MapLabel,pose);
muestr = 0;
Zest = [0;0;0];
numMuestrasPorCar = size(MapLabel,1);
for i=1:numMuestrasPorCar
    if(MapLabel(i,4) == k)
        muestr = muestr+1;
        m = MapLabel(i,1:2);
        zestglobal = [m(1)-pose(1);m(2)-pose(2);wrapToPi(atan2(m(2)-pose(2),m(1)-pose(1))-pose(3))]; 
        zest = [cos(pose(3)) sin(pose(3)) 0;-sin(pose(3)) cos(pose(3)) 0; 0 0 1]*zestglobal;
        Zest = Zest+zest;
    end
end
zout = inv(muestr)*Zest;
 