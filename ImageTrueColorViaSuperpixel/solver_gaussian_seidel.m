function [md_k,ms_k,Cd_k,Cs_k] = solver_gaussian_seidel(superpixels,md_k,ms_k,Cd_k,Cs_k)

md_k1 = zeros(size(md_k),'like',md_k);
ms_k1 = zeros(size(ms_k),'like',ms_k);
Cd_k1 = zeros(size(Cd_k),'like',Cd_k);
Cs_k1 = zeros(size(Cs_k),'like',Cs_k);

n_sp = length(superpixels);
iter = 0;
while(iter<5)
    iter = iter + 1;
    % the solving is performed for each super pixel individually
    for i=1:n_sp
        % step1: calculate Cd_k1 for each super pixel
        up = zeros(3,1);
        low = 0.0;
        len = size(superpixels{i},1);
        for j=1:len
            x = superpixels{i}(j,1);
            y = superpixels{i}(j,2);
            Cl_ij = [superpixels{i}(j,3);superpixels{i}(j,4);superpixels{i}(j,5)];
            md_k_ij = md_k(y,x);
            ms_k_ij = ms_k(y,x);
            up = up + md_k_ij*(Cl_ij-ms_k_ij*Cs_k);
            low = low + md_k_ij*md_k_ij;
        end
        Cd_k1_i = up/low;
        
        for j=1:len
            x = superpixels{i}(j,1);
            y = superpixels{i}(j,2);
            Cd_k1(y,x,:) = Cd_k1_i;
        end
        
        % step2: calculate md_k1 for each pixel
        for j=1:len
            x = superpixels{i}(j,1);
            y = superpixels{i}(j,2);
            Cl_ij = [superpixels{i}(j,3);superpixels{i}(j,4);superpixels{i}(j,5)];
            ms_k_ij = ms_k(y,x);
            Cd_k1_ij = [Cd_k1(y,x,1);Cd_k1(y,x,2);Cd_k1(y,x,3)];
            md_k1(y,x) = (Cd_k1_ij'*(Cl_ij-ms_k_ij*Cs_k))/(Cd_k1_ij'*Cd_k1_ij);
        end
        
        % step3: calculate ms_k1 for each pixel
        for j=1:len
            x = superpixels{i}(j,1);
            y = superpixels{i}(j,2);
            Cl_ij = [superpixels{i}(j,3);superpixels{i}(j,4);superpixels{i}(j,5)];
            
            %Cd_k_ij = [Cd_k(y,x,1);Cd_k(y,x,2);Cd_k(y,x,3)];
            %md_k_ij = md_k(y,x);
            %ms_k1(y,x) = (Cs_k'*(md_k_ij*Cd_k_ij-Cl))/(Cs_k'*Cs_k);
            Cd_k1_ij = [Cd_k1(y,x,1);Cd_k1(y,x,2);Cd_k1(y,x,3)];
            md_k1_ij = md_k1(y,x);
            ms_k1(y,x) = (Cs_k'*(Cl_ij-md_k1_ij*Cd_k1_ij))/(Cs_k'*Cs_k);
        end
    end
    
    % step4: calculate Cs_k1 via all the pixels
    up = zeros(3,1);
    low = 0.0;
    for i=1:n_sp
        len = size(superpixels{i},1);
        for j=1:len
            x = superpixels{i}(j,1);
            y = superpixels{i}(j,2);
            Cl_ij = [superpixels{i}(j,3);superpixels{i}(j,4);superpixels{i}(j,5)];
            
            %Cd_k_ij = [Cd_k(y,x,1);Cd_k(y,x,2);Cd_k(y,x,3)];
            %up = up + ms_k(y,x)*(md_k(y,x)*Cd_k_ij-Cl);
            %low = low + ms_k(y,x)*ms_k(y,x);
            
            Cd_k1_ij = [Cd_k1(y,x,1);Cd_k1(y,x,2);Cd_k1(y,x,3)];
            md_k1_ij = md_k1(y,x);
            ms_k1_ij = ms_k1(y,x);
            up = up + ms_k1_ij*(Cl_ij-md_k1_ij*Cd_k1_ij);
            low = low + ms_k1_ij*ms_k1_ij;
        end
    end
    Cs_k1 = up/low;
    
    % step5: update the new values
    md_k = md_k1;
    ms_k = ms_k1;
    Cd_k = Cd_k1;
    Cs_k = Cs_k1;
end

