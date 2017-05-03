import os
import glob
import argparse

def rename_image_files(*args):
    if args[0] == None:
        path = os.path.expanduser('~') + "/Documents/TrainImage"
    else:
        path = args[0]
    id_cnt = int(args[1])
    os.chdir(path)
    files = glob.glob('*.jpg')
    for file in files:
        parts = file.split('.')
        new_name = '{0:03}.jpg'.format(id_cnt)
        os.rename(file, new_name)
        id_cnt += 1
    print ("Successfully rename {} images".format(id_cnt - int(args[1])))

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--folder_full_path', default=None)
    parser.add_argument('-id', '--begin_image_id', required=True)
    args = parser.parse_args()

    rename_image_files(args.folder_full_path, args.begin_image_id)
