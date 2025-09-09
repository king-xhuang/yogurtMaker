#  out of date version for DS18B20 and oven NTC temperature reader data processing
import csv

input_file = "NTCthormost2025-08-31.txt"
output_file = "calculated_data.txt"
separator = ','  
with open(input_file, 'r', newline='', encoding='utf-8') as infile, \
     open(output_file, 'w', newline='', encoding='utf-8') as outfile:

    reader = csv.reader(infile, delimiter=separator)

    # Define only the fields you want to output
    fieldnames = ["index", "temp" , "vAvg" , "vMax", "vMin", "gIndex"]
    writer = csv.DictWriter(outfile, fieldnames=fieldnames, delimiter=separator)
    writer.writeheader()
    LastTemp = 100.0
    temp = 0.0
    v = 0.0
    index = -1
    
    gFirst = True
    gVMax = 100.0
    gVMin = -100.0
    gVTotal = 0.0
    gVAvg = 0.0
    gIndex= 0
    rowNum = 0 

    for row in reader:
        # row is a list, so you can access by index
        rowNum += 1
        gVMin = v
        temp = float(row[0] )  # column 1
        v = float(row[1])  # column 2  
         
        if (temp < LastTemp): # start a new group with the same temp
           index +=  1
           if gFirst:           
               gFirst = False
           else:
               # last group end
               gVAvg = gVTotal/gIndex  
               writer.writerow({ 
                "index": index,
                "temp": temp,
                "vAvg": round(gVAvg),
                "vMax": gVMax,
                "vMin": gVMin,
                "gIndex": gIndex })
               print(f"index:{index}, temp: {LastTemp}, vAvg: {round(gVAvg)}, vMax: {gVMax}, vMin: {gVMin}, gIndex: {gIndex}")
           # deal with new group
           gIndex= 1
           gVTotal = v
           gVMax = v            
           LastTemp = temp  
        else: #for rest of the values in the group
            if (temp > LastTemp):
                print(f"row:{rowNum}, temp: {temp}")
            else:
                gVTotal += v
                gIndex += 1
        