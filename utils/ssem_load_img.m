function [path img] = ssem_load_img(params, img)
%   [path img] = ssem_load_img(params, vid): Returns the image path for image 'img'. 
%   'img' should be the name of a sub-video as those returned by 
%   'ssem_load_subvideos(params) or by ssem_load_3rdpartyDBs(params);'.
        
    if ssem_isVid(img)
        path = fullfile(params.root, 'annots', 'back', [img.id '.jpg']);        
    else
        path = ssem_load_3rdpartyDBs_img(params, img);        
    end
    if nargin >= 2
        img = double(imread(path)) / 255;
    end
end
