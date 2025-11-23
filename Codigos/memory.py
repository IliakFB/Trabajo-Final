import sys

file_rep=sys.argv[1]

file = open(file_rep, "r") 
content = file.readlines()
file.close()
mem_len = len(content)
mem_new=[0 for i in range (0,(mem_len+2)*4)]
for x in range(0,mem_len):
	mem_new[x*4]= content[x][6:8]
	mem_new[x*4+1] = content[x][4:6]
	mem_new[x*4+2] = content[x][2:4]
	mem_new[x*4+3] = content[x][0:2]

end_bst = mem_len+1

file2 = open("mem_model.txt","w")
for x in range(0,end_bst*4):
	if(x<end_bst*4-4):
		file2.write(mem_new[x]+"\n")
	else:
		if(x<end_bst*4-1):
			file2.write("ff\n")
		else:
			file2.write("ff")
file2.close()