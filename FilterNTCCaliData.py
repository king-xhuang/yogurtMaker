 
import csv
#  DS18B20 and oven NTC temperature reader data processing
# raw data recorded by adcTemp.ino serial output
input_file = "NTCthormost2025-08-31.txt" 
# input_file = "small.txt"
output_file = "filter_data.txt"
separator = ','  
my_dict = dict() # use directory

with open(input_file, 'r', newline='', encoding='utf-8') as infile, \
     open(output_file, 'w', newline='', encoding='utf-8') as outfile:

    reader = csv.reader(infile, delimiter=separator) 
    temp = 0.0
    # v = 0.0  

    for row in reader:
        # row is a list, so you can access by index
        
        temp = float(row[0] )  # column 1
        v = float(row[1])  # column 2  
        data = my_dict.get(temp)
        if(data == None):
          gVMax  = v
          gVMin  = v
          gVCount = 1
          data = [ gVMax, gVMin , gVCount]    
          my_dict[temp] = data
        else:
          data[2] +=1
          if v > data[0]:
            data[0] = v
          if v < data[1]:
            data[1] = v
          my_dict[temp] = data
        
    #end of reading input
    print(my_dict)

# Define only the fields you want to output
    # fieldnames = ["index", "temp" , "vAvg" , "vMax", "vMin", "vCount"]
    fieldnames = [  "temp" , "vAvg" , "vMax", "vMin", "vCount"]
    writer = csv.DictWriter(outfile, fieldnames=fieldnames, delimiter=separator)
    writer.writeheader()  
    
    index = 0
    for t in  my_dict :
        index += 1
        d = my_dict.get(t)
        vMax = float(d[0])
        vMin = float(d[1])
        vCount = d[2]
        vAvg = (vMax + vMin)/2 
        writer.writerow({ 
                # "index": index,
                "temp": t,
                "vAvg": vAvg,
                "vMax": vMax,
                "vMin": vMin,
                "vCount": vCount})
            #    print(f"index:{index}, temp: {LastTemp}, vAvg: {round(gVAvg)}, vMax: {gVMax}, vMin: {gVMin}, gIndex: {gIndex}")
           