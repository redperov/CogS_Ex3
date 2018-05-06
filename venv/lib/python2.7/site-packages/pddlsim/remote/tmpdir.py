import os, uuid, shutil
class TmpDir():
    def __init__(self, root_dir, remove_root_if_empty=True):
        self.root_dir = root_dir
        self.remove_root_if_empty = remove_root_if_empty
        self.path = os.path.join(self.root_dir, uuid.uuid4().hex)
        os.makedirs(self.path)
    
    def join(self, *paths):
        return os.path.join(self.path, *paths)

    def __del__(self):        
        shutil.rmtree(self.path)
        if self.remove_root_if_empty:
            files = os.listdir(self.root_dir)
            if len(files) == 0:
                os.rmdir(self.root_dir)