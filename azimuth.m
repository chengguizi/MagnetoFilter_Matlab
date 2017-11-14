function output = azimuth(x,z)


    % Azimuth = atan2(Yh, Xh);
    % magx = Yh;
    % magz = Xh;
    
    output = zeros(size(x,1),1);
    
    for i = 1:size(x,1)
        
        magx= x(i);
        magz= z(i);
        
        if (magz == 0 && magx < 0)
            Azimuth = pi*0.5;
        end

        if (magz == 0 && magx > 0)
            Azimuth = pi*1.5;
        end

        if (magz < 0)
            Azimuth = pi - atan(magx/magz);
        end

        if (magz > 0 && magx < 0)
            Azimuth = - atan(magx/magz);
        end

        if (magz > 0 && magx > 0)
            Azimuth = 2*pi - atan(magx/magz);
        end
        
        output(i) = Azimuth;
    end

end