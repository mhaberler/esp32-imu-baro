Import('env', 'projenv')

import os
import gzip
import shutil
import glob
from pathlib import Path
import subprocess

# usage:

# data_origin_dir = teleplot/server/www
# data_dest_dir = data
# compress_ext = *.html, *.js, *.css
# compressor = brotli --rm

def prepare_www_files(source, target, env):
    data_dest_dir = env.GetProjectOption("data_dest_dir")
    data_foo_dir = env.GetProjectOption("data_foo_dir")
    data_origin_dir = env.GetProjectOption("data_origin_dir")
    compress_ext = env.GetProjectOption("compress_ext")
    compressor = env.GetProjectOption("compressor", default='gzip').split()
    ext_list = [x.strip() for x in compress_ext.split(',')]
    
    print(f" {compressor=}")      

    if(os.path.exists(data_foo_dir)):
        print(f" Deleting {data_foo_dir=}")      
        shutil.rmtree(data_foo_dir)
    
    print(f" syncing  {data_origin_dir} to {data_dest_dir}")      
    subprocess.call(["rsync", "-ap", "--mkpath", data_origin_dir, data_dest_dir])

    ul =  [[ str(f) for f in Path(data_foo_dir).rglob(p)] for p in ext_list]
    files_to_compress = [item for sublist in ul for item in sublist]
    print(f" {files_to_compress=}")
    subprocess.call(compressor + files_to_compress)

   
env.AddPreAction('$BUILD_DIR/littlefs.bin', prepare_www_files)

