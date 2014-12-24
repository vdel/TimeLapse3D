function gt = ssem_load_3rdpartyDBs_gt(params, img)
    eval(sprintf('gt = load_3rdpartyDB_gt_%s(params.db_3rdparty(img.db).path, img.gtpath);', params.db_3rdparty(img.db).name));    
end