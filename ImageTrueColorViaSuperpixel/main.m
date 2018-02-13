function main()

%% step1: read in image 
im_path = [pwd '/data/7.pbm'];

im = imread(im_path);
[im_rows,im_cols,depth] = size(im);
im_size = im_rows*im_cols;
if depth ~= 3
    disp('Please choose a RGB image...');
    return;
end

%% step2: imaeg segmentation
method = 3;

if method == 1 % SLIC super pixel
    addpath('./vlfeat-0.9.21/toolbox');
    vl_setup();
    region_size = 10;
    regularizer = 0.0 ;
    im_lab = rgb2lab(im);
    segs = vl_slic(single(im_lab), region_size, regularizer);
elseif method == 2 % Region grow
    th = 3.0;
    segs = image_segmentation_region_grow(im, th);
else % Graph cut
    addpath('./segment');
    cd('./segment');
    mex ./segment.cpp
    segs = segment(im_path,0.5,500,20);
    cd('../');
end

% draw segmentation result
labels = unique(segs);
if(1)
    im_slic = zeros(size(im),'like',im);
    for i=1:length(labels)
        l = labels(i);
        r_idx = find(segs==l);
        g_idx = r_idx+im_size;
        b_idx = g_idx+im_size;
        
        im_slic(r_idx) = mean(im(r_idx));
        im_slic(g_idx) = mean(im(g_idx));
        im_slic(b_idx) = mean(im(b_idx));
    end
    subplot(2,3,1);
    imshow(im_slic);
    title('segmentation result');
end

%% step3: slove the function
superpixels = {};
im = im2single(im);
for i=1:length(labels)
    l = labels(i);
    r_idx = find(segs==l);
    g_idx = r_idx+im_size;
    b_idx = g_idx+im_size;
    superpixels{i}(:,1) = fix((r_idx-1)/im_rows)+1;   % x
    superpixels{i}(:,2) = rem(r_idx-1,im_rows)+1; % y
    superpixels{i}(:,3) = im(r_idx); % r
    superpixels{i}(:,4) = im(g_idx); % g
    superpixels{i}(:,5) = im(b_idx); % b
end

md = ones(im_rows, im_cols);
ms = rand(im_rows, im_cols)*0.1;
Cd = double(im);
Cs = rand(3, 1)*0.1;
[md,ms,Cd,Cs] = solver_gaussian_seidel(superpixels,md,ms,Cd,Cs);

subplot(2,3,2);
imshow(uint8(md*100));
title('md');

subplot(2,3,3);
imshow(uint8(ms*100));
title('ms');

subplot(2,3,4);
imshow(uint8(Cd*255));
title('Cd');

im_mdcd = zeros(size(im),'like',im);
im_mdcd(:,:,1) = Cd(:,:,1).*md;
im_mdcd(:,:,2) = Cd(:,:,2).*md;
im_mdcd(:,:,3) = Cd(:,:,3).*md;
subplot(2,3,5);
imshow(uint8(im_mdcd*255));
title('md*Cd');

subplot(2,3,6);
imshow(uint8(im*255));
title('original image');
% 


