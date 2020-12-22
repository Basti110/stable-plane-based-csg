import subprocess
import os
from pathlib import Path


folder = "E:/benchmark/files"
fileCount = len(os.listdir(folder))
meshError = 0
success = 0
errors = 0
loopCount = 0
Path("E:/benchmark/files").mkdir(parents=True, exist_ok=True)

for filename in os.listdir(folder):
    with open(f"E:/benchmark/logs/{os.path.splitext(filename)[0]}.log",'w+') as fout:
        test = subprocess.run(["./bin/PlaneOctreeCSG.exe", filename], stdout=fout,stderr=fout)
        loopCount += 1

        if(test.returncode == 0): 
            print(f"{loopCount}/{fileCount} {filename} [SUCCESS]")
            success+=1
        elif(test.returncode == 2) : 
            print(f"{loopCount}/{fileCount} {filename} [MESH_ERROR]")
            meshError+=1
        else : 
            print(f"{loopCount}/{fileCount} {filename} [ERROR]")
            errors+=1

            
        
print(f"---------------------------------")
print(f"{success}/{fileCount} [SUCCESS]")
print(f"{errors}/{fileCount} [ERROR]")
print(f"{meshError}/{fileCount} [MESH_ERROR]")