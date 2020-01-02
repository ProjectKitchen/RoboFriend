#python3

dict_pos={}

def file_parser(file_path):
  global dict_pos
  print(file_path)
  try:  
    f=open(file_path)
  except:
    print("no valid Path")
    return -1
  while True:
    line=f.readline()
    if line == "":
        break
    if line[0]=='#':
        continue
    #print(line)
    
    args=line.split(",")
    #print(args)
    #print(args[0])
    dict_pos[args[0]]= args[1:]
  f.close()

def new_entry():
  print("Name of Position")
  name=input()
  print("Number of x")
  x=input()
  print("Number of y")
  y=input()
  args=[x,y,"0.0","0.0","0.0","1.0"]
  dict_pos[name]= args


def save_position(file_path):
  f=open(file_path,"w+")
  for key in dict_pos.keys():  
    value=dict_pos.get(key)    
  line = key+","+value[0]+","+value[1]+","+value[2]+","+value[3]+","+value[4]+","+value[5]+"\n"
  f.write(line)

def get_value(key,typ):
  value=dict_pos.get(key)
  print(value)
  if typ == "point":
    return float(value[0]),float(value[1]),-1,-1
  if typ == "rot":
    return float(value[2]),float(value[3]),float(value[4]),float(value[5])

file_parser("positions.txt")
print (dict_pos)

while True:
  print()
  user_in=input()
  if user_in=='p':
    print(dict_pos.keys())
  elif user_in=='q':
    print("close Programm")
    break
  elif user_in=='n':
    new_entry()
  elif user_in=='s':
    save_position("positions.txt")
  elif user_in=='g':
    print("Key?")
    key=input()
    print("point or rotation(rot)?")
    value=input()
    x,y,z,w=get_value(key,value)
    print(x)
