
import csv 

input_file = "york-OvenProbeC-R-V_data.txt"
separator = ','  

# Arrays to hold the columns
col1 = []
col2 = []
data = []

with open(input_file, 'r', newline='', encoding='utf-8') as infile:
    reader = csv.reader(infile, delimiter=separator)

    # Skip header
    header = next(reader)

    for row in reader:
        v = float(row[2])
       # print(f" v=: {v}") ;
        data.append(v)
        col1.append(row[0]) 
        col2.append(row[1])
    
    
       
def check_array( values ):
    for i in range(0, len(values) ):
        print(f" v=: {values[i]}") ;
def checkUniqe(values)  :
    unique_values = list(set(values))
    if (len(values) != len( unique_values)):
        seen = set()
        duplicates = set()
        for item in values:
           if item in seen:
                duplicates.add(item)
           else:
                seen.add(item)

        print(duplicates)   # {1, 2, 3}
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
col1.reverse()
col2.reverse()
matlab_code = []
matlab_code.append(format_matlab_array(header[0], col1))
matlab_code.append(format_matlab_array(header[1], col2))

#check_array(col1)
#check_array(col2)
checkUniqe(data)
#checkUniqe(col2)
# Write MATLAB code to file
outputFile = "york-OvenProbeC-R-V_data-arrays.m"
with open(outputFile, "w") as f:
    for line in matlab_code:
        f.write(line + "\n\n")

print(f"MATLAB code generated in  {outputFile}")