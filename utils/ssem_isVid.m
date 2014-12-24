function isVid = ssem_isVid(img)
    isVid = ~strcmp(img.id(1 : 3), 'img');
end