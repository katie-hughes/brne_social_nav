trialNumber = 0
idx = 0
trialList = []
trialDict = dict()

with open('toconvert.txt') as f:
  for l in f.readlines():
    spl = l.split()
    if len(spl) == 2: 
      trialNumber = int(spl[1])
      idx = 0
    if len(spl) > 6:
      cropped = spl[4:]
      print(trialNumber, cropped)
      if idx == 0:
        trialDict['Time'] = cropped[1]
      elif idx == 1:
        trialDict['Straight Line Path Distance'] = cropped[3]
      elif idx == 2:
        trialDict['Path Distance'] = cropped[2]
      elif idx == 3:
        trialDict['Path Ratio'] = cropped[2]
      elif idx == 4:
        trialDict['Closest Dist To Pedestrian'] = cropped[4]
      elif idx == 5:
        trialDict['Number Of E-STOPs'] = cropped[3]
        trialList.append(trialDict)
        trialDict = dict()
      idx += 1


print()
print()
# print in markdown format

print("| Trial | ", end='')
for id in trialList[0]:
  print(f"{id} | ", end='')
print()
# 6 id's
print("|", end='')
for i in range(7):
  print(' ---- |', end='')
print()

trialNumber = 1
for trial in trialList:
  print(f"| {trialNumber} | ", end='')
  for id in trial:
    print(f" {trial[id]} | ", end='')
  print()
  trialNumber += 1