import subprocess
import os
import time
import threading
from pathlib import Path




max_threads = 6
#folder = "D:/benchmark"
#folder = "E:/Thingi10K/Thingi10K/raw_meshes"
folder = "E:/benchmark/files"
folder_libgl = "C:/Users/Basti/Documents/git/csg-comparison-libgl/build/Release/csg-comparison-libigl.exe"
folder_cgal = "C:/Users/Basti/Documents/git/csg-comparison-cgal/bin/csg-comparison.exe"

Path("E:/benchmark/files").mkdir(parents=True, exist_ok=True)
file_count = len(os.listdir(folder))

class Starter():
    def __init__(self):
        self.meshError = 0
        self.success = 0
        self.errors = 0
        self.loopCount = 0

    def startExec(self, filename):
        DEVNULL = open(os.devnull, 'wb')
        with open(f"E:/benchmark/logs/{os.path.splitext(filename)[0]}.log",'w+') as fout:
            return_val = subprocess.run(["./bin/PlaneOctreeCSG.exe", filename], stdout=fout,stderr=fout)
            self.loopCount += 1
            if(return_val.returncode == 0): 
                print(f"{self.loopCount}/{file_count} {filename} [SUCCESS] {self.success}:{self.meshError}:{self.errors}")
                self.success+=1
            elif(return_val.returncode == 2) : 
                print(f"{self.loopCount}/{file_count} {filename} [MESH_ERROR] {self.success}:{self.meshError}:{self.errors}")
                self.meshError+=1
            else : 
                print(f"{self.loopCount}/{file_count} {filename} [ERROR] {self.success}:{self.meshError}:{self.errors}")
                self.errors+=1
            if(return_val.returncode != 0):
                #print("return")
                return
        return
        return_val = subprocess.run([folder_libgl, "--libgl", filename], stdout=DEVNULL, stderr=DEVNULL) 
        if(return_val.returncode == 0):
            print("libgl [SUCCESS]")
        else:
            print("libgl [ERROR]")
        return
        self.loopCount += 1
        print(f"Mesh: {self.loopCount}")
        return_val = subprocess.run([folder_libgl, "--cork", filename], stdout=DEVNULL, stderr=DEVNULL) 
        if(return_val.returncode == 0):
            print("cork [SUCCESS]")
        else:
            print("cork [ERROR]")
        return_val = subprocess.run([folder_cgal, "--cgal", filename], stdout=DEVNULL, stderr=DEVNULL) 
        if(return_val.returncode == 0):
            print("Cgal [SUCCESS]")
        else:
            print("Cgal [ERROR]")
        

starter = Starter()
my_threads = []
for filename in os.listdir(folder):       
    #print(len(my_threads)) 
    while len(my_threads) >= max_threads:
        my_threads = [t for t in my_threads if t.is_alive()]
        time.sleep(0)       
    new_thread = threading.Thread(target = starter.startExec, args = (filename,))
    new_thread.start()
    my_threads.append(new_thread)

for my_thread in my_threads:
    my_thread.join()

              
print(f"---------------------------------")
print(f"{starter.success}/{file_count} [SUCCESS]")
print(f"{starter.errors}/{file_count} [ERROR]")
print(f"{starter.meshError}/{file_count} [MESH_ERROR]")