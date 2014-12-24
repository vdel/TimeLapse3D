function path = ssem_load_3rdpartyDBs_img(params, img)
    eval(sprintf('path = load_3rdpartyDB_img_%s(params.db_3rdparty(img.db).path, img.imgpath);', params.db_3rdparty(img.db).name));
end