
import csv 
import math
# generate Temp, V array in matlab format
input_file = "filter_data.txt"
output  = "c2varray.m"
separator = ','  
 

# Arrays to hold the columns
colTemp = []
colV = [] 
# tempAnchors values must be eqnique and must in the colTemp
tempAnchors = [16.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.25, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0, 90.0, 95.0]
vAnchors = []
anchorIndex = 0

with open(input_file, 'r', newline='', encoding='utf-8') as infile :
    reader = csv.reader(infile, delimiter=separator) 
    # Skip header
    header = next(reader) 
    for row in reader:
        v = float(row[1])
        t = float(row[0]) 
        # if(row[0] != tempAnchors[anchorIndex]):
        #    anchorIndex += 1
        #    vAnchors.append(row[1])
        #    print(f"t= {row[0]} { tempAnchors[anchorIndex] } v=: {row[1]} \n") ;      
        colTemp.append(row[0]) 
        colV.append(row[1])   

def check_array( values ):
    for i in range(0, len(values) ):
        print(f" v=: {values[i]}") 

def checkUniqe(values)  :
    unique_values = list(set(values))
    if (len(values) != len( unique_values)):
        seen = set()
        duplicates = list()
        for item in values:
           if item in seen:
                duplicates.append(item)
           else:
                seen.add(item)
        print(f"duplicates={duplicates}") 
        # print(duplicates)   # {1, 2, 3}
    #print(f"{len(values)},{len( unique_values)}")
    #print(unique_values)
    #unique_values.sort(reverse=True );
    #    print(unique_values)

def format_matlab_array(name, values, per_line=10):
    """Format a MATLAB array with line breaks for readability."""
    lines = []
    for i in range(0, len(values), per_line):
        chunk = values[i:i+per_line]
        lines.append(" ".join(chunk))
    # Join with "..." continuation
    array_str = (" ...\n    ").join(lines)
    return f"{name} = [{array_str}];"

def format_c_array(name, values):
    line = "{"
    for i in range(0, len(values)):
        if(i > 0):
            line += ", "
        line += values[i ]  

    line += "}"
    
    return line
def format_anchor_array(name, keys, tempvalues, vVal):
    keyLen = len(keys )
    keyI = 0
    line = name + "["
    for i in range(0, len(tempvalues)): 
        # print(f"k={keys[keyI]}  t= { tempvalues[i] } {vVal[i]}  \n") ; 
        
        if( math.isclose(keys[keyI], float(tempvalues[i]) ) ):
            line += " "
            line += vVal[i ]              
            print(f"found: key= {keys[keyI]} temp ={ tempvalues[i] } v = {vVal[i]}  \n") ;  
            
            keyI += 1
            if(keyI == keyLen):
                break


    line += "]"
    
    return line
# Build MATLAB code
matlab_code = []
colTemp.reverse()
colV.reverse() 
# print(format_c_array("v", colV))
print(format_anchor_array("anchorsX = ", tempAnchors, colTemp, colV  ))
matlab_code.append(format_matlab_array("y", colTemp))  # temp
matlab_code.append(format_matlab_array("x", colV))    # v
matlab_code.append(format_anchor_array("anchorsX = ", tempAnchors, colTemp, colV  ))
matlab_code.append("%% anchors temp " + str(tempAnchors))
#check_array(col1)
#check_array(col2)
checkUniqe(colV) 
# Write MATLAB code to file
with open(output, "w") as f:
    for line in matlab_code:
        f.write(line + "\n\n")

print("MATLAB code generated in c2varray.m")

