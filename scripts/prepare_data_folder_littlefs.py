Import('env', 'projenv')

import os
import gzip
import shutil
import re
from pathlib import Path
import subprocess

def prepare_www_files(source, target, env):
    data_dest_dir = env.GetProjectOption("data_dest_dir")
    data_origin_dirs = env.GetProjectOption("data_origin_dirs")
    compress_ext = env.GetProjectOption("compress_ext")
    compressor = env.GetProjectOption("compressor", default='gzip').split()
    pattern = re.compile("^\s+|\n|\s*,\s*|\s+$")
    ext_list = [x for x in list(map(str.strip, pattern.split(compress_ext))) if x]
    origin_list = [x for x in list(map(str.strip, pattern.split( data_origin_dirs))) if x]

    print(f"{ext_list=}")      
    print(f"{origin_list=}")      

    if(os.path.exists(data_dest_dir)):
        print(f"Deleting {data_dest_dir=}")      
        shutil.rmtree(data_dest_dir)
    
    for o in origin_list:
        print(f"syncing {o} to {data_dest_dir}")      
        subprocess.call(["rsync", "-ap", "--mkpath", o, data_dest_dir])

    ul =  [[ str(f) for f in Path(data_dest_dir).rglob(p)] for p in ext_list]
    files_to_compress = [item for sublist in ul for item in sublist]
    print(f" {files_to_compress=}")
    subprocess.call(compressor + files_to_compress)

   
env.AddPreAction('$BUILD_DIR/littlefs.bin', prepare_www_files)

