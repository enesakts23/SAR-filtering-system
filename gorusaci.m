dtedfile = "n39_w106_3arc_v2.dt1";
attribution = "SRTM 3 arc-second resolution. Data available from the U.S. Geological Survey.";
[Zterrain,Rterrain] = readgeoraster(dtedfile,"OutputType","double");
latlim = Rterrain.LatitudeLimits;
lonlim = Rterrain.LongitudeLimits;
latspc = Rterrain.SampleSpacingInLatitude;
lonspc = Rterrain.SampleSpacingInLongitude;
disp("Latitude limits of terrain: " + mat2str(latlim) + newline + ...
"Longitude limits of terrain: " + mat2str(lonlim) + newline + ...
"Terrain resolution in latitude: " + latspc*3600 + " arc seconds" + newline + ...
"Terrain resolution in longitude: " + lonspc*3600 + " arc seconds")
%addCustomTerrain("southboulder",dtedfile,"Attribution",attribution)
fig = uifigure;
g = geoglobe(fig,"Terrain","southboulder");
hold(g,"on")
rdrlat = 39.913756;
rdrlon = -105.118062;
rdrtowerht = 10;
rdralt = 1717 + rdrtowerht;
geoplot3(g,rdrlat,rdrlon,rdralt,"co", ... 
"LineWidth",6, ...
"MarkerSize",1)
tlat0 = 39.80384;
tlon0 = -105.49916;
azs = 1:540;
r = 5000;
[X,Y] = pol2cart(deg2rad(azs),r);
Z = linspace(0,1000,numel(azs));
wgs84 = wgs84Ellipsoid;
%[tlat,tlon,tht] = enu2geodetic(X,Y,Z,tlat0,tlon0,tht0,wgs84);
traj = geoplot3(g,tlat,tlon,tht,"y", ...
    "HeightReference","ellipsoid", ...
    "LineWidth",3);
campos(g,39.77114,-105.62662,6670)
camheading(g,70)
campitch(g,-12)
numwaypts = numel(tlat);
isvis = zeros(1,numwaypts);
talt = tht - egm96geoid(tlat,tlon);
for k = 1:numwaypts
    isvis(k) = los2(Zterrain,Rterrain,rdrlat,rdrlon,tlat(k),tlon(k),rdralt,talt(k),"MSL","MSL");
end
isvis = logical(isvis);
delete(traj)
geoplot3(g,tlat(isvis),tlon(isvis),tht(isvis),"og", ...
    "HeightReference","ellipsoid", ...
    "LineWidth",2, ...
    "MarkerSize",1)
geoplot3(g,tlat(~isvis),tlon(~isvis),tht(~isvis),"or", ...
    "HeightReference","ellipsoid", ...
    "LineWidth",2, ...
    "MarkerSize",1)
rdrht = rdralt + egm96geoid(rdrlat,rdrlon);
[camlat,camlon,camht] = enu2geodetic(900,200,100,rdrlat,rdrlon,rdrht,wgs84);
campos(g,camlat,camlon,camht)
camheading(g,-110)
campitch(g,0)
figure
geoplot(rdrlat,rdrlon,"co", ...
    "LineWidth",6, ...
    "MarkerSize",3, ...
    "DisplayName","Radar location")
hold on
geobasemap topographic
gx = gca;
gx.InnerPosition = gx.OuterPosition;
latmin = latlim(1);
latmax = latlim(2);
lonmin = lonlim(1);
lonmax = lonlim(2);
geoplot([latmin latmin latmax latmax latmin],[lonmin lonmax lonmax lonmin lonmin], ...
    "LineWidth",1, ...
    "Color","k", ...
    "DisplayName","Terrain limits")
legend("Location","northwest")
tgtalts = [3000 4000 5000];

minVertices = 10;
cfig = figure("Visible","off"); % Suppress contour plot using invisible figure
cax = axes("Parent",cfig);
for tgtalt = tgtalts
    vis = viewshed(Zterrain,Rterrain,rdrlat,rdrlon,rdralt,tgtalt,"MSL","MSL");
    
    C = contourm(vis,Rterrain,"LevelList",1,"Parent",cax);
    clat = C(2,:);
    clon = C(1,:);
    
    clats = [];
    clons = []; 
    k = 1;   
    while k < size(C,2)
        numVertices = clat(k);
        if numVertices > minVertices % Do not plot small segments 
            clats = [clats clat(k+1:k+numVertices) NaN]; %#ok<AGROW> 
            clons = [clons clon(k+1:k+numVertices) NaN]; %#ok<AGROW> 
        end
        k = k + numVertices + 1;
    end
    
    geoplot(gx,clats,clons,"LineWidth",2, ...
        "DisplayName", "Aircraft: " + string(tgtalt) + " m");
end
if isvalid(fig)
    close(fig)
end
removeCustomTerrain("southboulder")

