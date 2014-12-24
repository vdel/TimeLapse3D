function annot = ssem_unpack_annot(packed, label)
%   ssem_unpack_annot(packed, label): 'label' is a structure as returned by 
%   "ssem_load_annots.m". It uncompress the ground truth segmentation into a 3D
%   logical matrix. Each coefficient '(x, y, l)' of the matrix is true iff pixel 
%   '(x, y)' has label 'l'.

    if nargin == 1
        annot = false(size(packed.annot, 1), size(packed.annot, 2), packed.nlabels);

        for i = 1 : packed.nlabels
            annot(:, :, i) = mod(floor(double(packed.annot) / (2 ^ (i-1))), 2);        
        end
        annot = annot(:, :, packed.map);
    else
        annot = logical(mod(floor(double(packed.annot) / (2 ^ (packed.map(label)-1))), 2));        
    end
    
    annot = imresize(annot, packed.size, 'nearest');
end
