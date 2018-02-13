function segs = image_segmentation_region_grow(im, th)

im = rgb2lab(im);
[rows, cols, c] = size(im);

mask = uint8(zeros(rows, cols));
offsetx = [-1,-1,-1,0,0,1,1,1];
offsety = [-1,0,1,-1,1,-1,0,1];
num_region = 1;
regions = {};
for i=1:rows
    for j=1:cols
        if(mask(i,j))
            continue;
        end
        
        % begin region growing
        ys = i;
        xs = j;
        ls = im(i,j,1);
        as = im(i,j,2);
        bs = im(i,j,3);
        
        num = 1;
        pixels = [];
        pixels(num,:) = [ys,xs];
        mask(ys,xs) = 1;
        
        count = 1;
        while count <= num
            y0 = pixels(count,1);
            x0 = pixels(count,2);
            l0 = im(y0,x0,1);
            a0 = im(y0,x0,2);
            b0 = im(y0,x0,3);
            for m=1:8
                y = y0 + offsety(m);
                x = x0 + offsetx(m);
                if x<1 || x>cols || y<1 || y>rows
                    continue;
                end
                
                if( mask(y,x))
                    continue;
                end
                
                l = im(y,x,1);
                a = im(y,x,2);
                b = im(y,x,3);
                
                if (abs(l-l0)+abs(a-a0)+abs(b-b0))/3 < th
                    if (abs(l-ls)+abs(a-as)+abs(b-bs))/3 < 4*th
                        num = num + 1;
                        pixels(num,:) = [y,x];
                        mask(y,x) = 1;
                    end
                end
            end
            count = count + 1;
        end
        
        if(length(pixels) < 5)
            continue;
        end
        regions{num_region} = pixels;
        num_region = num_region + 1;
    end
end

segs = int16(zeros(rows, cols));
count = 1;
for i=1:length(regions)
    len = length(regions{i});    
    for j=1:len
        y = regions{i}(j,1);
        x = regions{i}(j,2);
        segs(y,x) = count;
    end
    count = count + 1;
end


