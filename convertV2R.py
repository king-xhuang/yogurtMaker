
import csv 
# convert voltage to NTC resistor value
input_file = "york-OvenProbeData.txt"
crv_file  = input_file + "C-R-V_data.txt"
separator = ','  

Vtest = 2505.0  # mv
Vbase = 507.0   # mv
R     = 50.06   # kOhm
Rntc  = 0.0   # kOhm

# Arrays to hold the columns
col1 = []
col2 = []
rntc = []
data = []
def v2Rntc( v ): 
    Vr = v + Vbase 
    Rntc = R*Vtest/Vr - R
    return round(Rntc*1000,2)
         

with open(input_file, 'r', newline='', encoding='utf-8') as infile , \
     open(crv_file, 'w', newline='', encoding='utf-8') as outfile:
    reader = csv.reader(infile, delimiter=separator)
    fieldnames = [  "temp" , "r" , "vt", "v" ]
    writer = csv.DictWriter(outfile, fieldnames=fieldnames, delimiter=separator)
    writer.writeheader() 
    
    # Skip header
    header = next(reader)
    
    for row in reader:
        v = float(row[1])
        t = float(row[0]) 

    #    # print(f" v=: {v}") ;      
        col1.append(row[0]) 
        col2.append(row[1])  
        Rntc  = v2Rntc(v)
        writer.writerow({ 
                # "index": index,
                "temp": t,
                "r": Rntc,
                "vt": v + Vbase ,
                "v": v })

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

# Build MATLAB code
matlab_code = []
col1.reverse()
rntc.reverse()
matlab_code.append(format_matlab_array("temp", col1))
matlab_code.append(format_matlab_array("R", rntc))

#check_array(col1)
#check_array(col2)
checkUniqe(data)
#checkUniqe(col2)
# Write MATLAB code to file
with open("c-r.m", "w") as f:
    for line in matlab_code:
        f.write(line + "\n\n")

print("MATLAB code generated in c-r.m")

