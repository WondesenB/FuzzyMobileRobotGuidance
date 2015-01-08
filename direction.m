function dir = direction(lat1,lon1,lat2,lon2)
     dlon=DegToRad(lon2-lon1);
     y=sin(dlon)*cos(DegToRad(lat2));
     x=cos(DegToRad(lat1))*sin(DegToRad(lat2))-sin(DegToRad(lat1))*cos(DegToRad(lat2))*cos(dlon);
     brng=RadToDeg(atan2(y,x));
    if (brng >= 0)
      dir= brng;
    else
    dir= (brng+360);
    end
end