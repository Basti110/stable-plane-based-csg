import subprocess
import os
import time
import threading
from pathlib import Path


folder = "E:/benchmark/files"
fileCount = len(os.listdir(folder))
Path("E:/benchmark/files").mkdir(parents=True, exist_ok=True)
maxThreads = 1

class Starter():
    def __init__(self):
        self.meshError = 0
        self.success = 0
        self.errors = 0
        self.loopCount = 0

    def startExec(self, filename):
        with open(f"E:/benchmark/logs/{os.path.splitext(filename)[0]}.log",'w+') as fout:
            test = subprocess.run(["./bin/PlaneOctreeCSG.exe", filename], stdout=fout,stderr=fout)
            self.loopCount += 1
            if(test.returncode == 0): 
                print(f"{self.loopCount}/{fileCount} {filename} [SUCCESS]")
                self.success+=1
            elif(test.returncode == 2) : 
                print(f"{self.loopCount}/{fileCount} {filename} [MESH_ERROR]")
                self.meshError+=1
            else : 
                print(f"{self.loopCount}/{fileCount} {filename} [ERROR]")
                self.errors+=1

starter = Starter()
my_threads = []
for filename in os.listdir(folder):       
    #print(len(my_threads)) 
    while len(my_threads) >= maxThreads:
        my_threads = [t for t in my_threads if t.is_alive()]
        time.sleep(0)       
    new_thread = threading.Thread(target = starter.startExec, args = (filename,))
    new_thread.start()
    my_threads.append(new_thread)

for my_thread in my_threads:
    my_thread.join()

              
print(f"---------------------------------")
print(f"{starter.success}/{fileCount} [SUCCESS]")
print(f"{starter.errors}/{fileCount} [ERROR]")
print(f"{starter.meshError}/{fileCount} [MESH_ERROR]")