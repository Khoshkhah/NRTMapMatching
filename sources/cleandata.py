import pandas as pd
from geotools import distance2d, dfLonLat2XY, calculate_bearing_angle
import numpy as np

# Two consecutive stop lables consider as a one stop and the location is somewhere between two tops
# stopindex=0 it means "moving"
# observation data must be have all collumns "lon","lat", "timestamp", "speed" and "bearing"
    
def cleaningData(obs, net,id=0, MINSPEED_FOR_BEARING=1, MAX_SPEED_FOR_OUTLIER=100 ):

    # check necessary column in data
    # convert lon,lat to x,y
    # remove outlier based on max speed
    # assing id

    mycols = {"lon","lat", "timestamp", "speed", "bearing"}
    missedcols = mycols.difference(set(obs.columns))
    if missedcols:
        print(f"missed the columns : {missedcols}")
        return 0

    output = []
    row_current = obs.iloc[0]
    point_current = net.convertLonLat2XY(row_current["lon"],row_current["lat"])
    timestamp_current = row_current["timestamp"]
    stopsnumber = 1

    for index in range(1,len(obs)):
        row_next = obs.iloc[index]
        point_next = net.convertLonLat2XY(row_next["lon"],row_next["lat"])
        timestamp_next  = row_next["timestamp"]
        dist = distance2d(point_current, point_next)
        speed_current_estimated = dist/(timestamp_next-timestamp_current)

        speed_current = row_current["speed"]
        bearing_current = row_current['bearing']
        if speed_current_estimated < MAX_SPEED_FOR_OUTLIER: # max speed for outlier detection

            if speed_current < MINSPEED_FOR_BEARING and len(output) > 0: #minimum speed for calculating a new bearing
                bearing_current = output[-1]["bearing"]

            if speed_current>0:
                stopindex = 0
            else:
                if len(output) and output[-1]["stopindex"] == 0:
                    stopsnumber+=1
                stopindex = stopsnumber

            output.append({"id":id, "x":point_current[0], 'y':point_current[1], 'timestamp':int(timestamp_current),
                          "speed":speed_current, 'bearing':bearing_current, "stopindex":stopindex})
            point_current = np.array(point_next)
            timestamp_current = timestamp_next
            row_current = row_next


    speed = row_current["speed"]
    bearing = row_current['bearing']

    if speed < MINSPEED_FOR_BEARING and len(output) > 0:  #minimum speed for calculating a new bearing
        bearing = output[-1]["bearing"]

    if speed_current>0:
        stopindex = 0
    else:
        if len(output) and output[-1]["stopindex"] == 0:
            stopsnumber+=1
        stopindex = stopsnumber

    output.append({"id":id,"x":point_current[0], 'y':point_current[1], 'timestamp':int(timestamp_current),
                          "speed":speed, 'bearing':bearing, "stopindex":stopindex})

    df = pd.DataFrame(output)

    # assign the same 'x' and 'y' based on median to every consecutive point with the same stopindex
    df = xystop_point_editing(df)
    return df



def xystop_point_editing(df):
    """
    set the same 'x' and 'y' based on median to every consecutive point with stop lable
    Args:
        df (pandas.DataFrame): columns = {'id', lon','lat', 'timestamp', 'speed', 'bearing', 'stopindex'}.

    Returns:
        df (pandas.DataFrame): columns = {'id', lon','lat', 'timestamp', 'speed', 'bearing', 'stopindex'}.
    """
    
    # stopindex zero is for moving type.
    
    df_x = df.groupby("stopindex")["x"].apply(np.median)
    df_y = df.groupby("stopindex")["y"].apply(np.median)
    df['x'] = df.apply(lambda row: df_x[row.stopindex] if row.stopindex > 0 else row.x, axis=1)
    df['y'] = df.apply(lambda row: df_y[row.stopindex] if row.stopindex > 0 else row.y, axis=1)
    #df["point"] = df.apply(lambda row: (row.x, row.y), axis=1)
    #df = df.drop(columns=["x", "y"])
    return df


def removeOutlier(obs, MAX_SPEED_FOR_OUTLIER = 50):
    
    output = []
    row_previous = obs.iloc[0]
    point_previous = (row_previous["x"], row_previous["y"])
    timestamp_previous = row_previous["timestamp"]
    output.append({"x":point_previous[0], 'y':point_previous[1], 'timestamp':int(timestamp_previous)})

    for index in range(1,len(obs)):
        row_current = obs.iloc[index]
        point_current = (row_current["x"],row_current["y"])
        timestamp_current  = row_current["timestamp"]
        dist = distance2d(point_current, point_previous)
        speed_current = dist/(timestamp_current-timestamp_previous)    
        if speed_current < MAX_SPEED_FOR_OUTLIER: # max speed for outlier detection
            output.append({"x":point_current[0], 'y':point_current[1], 'timestamp':int(timestamp_current)})
            point_previous = point_current
            timestamp_previous = timestamp_current
            row_previous = row_current
    
    return pd.DataFrame(output)



def smoothingPoint(obs, alpha=.7, type="local"): #type in {"local", "aggregated"}
    
    df2 = obs.join(obs.shift(-1).rename(columns={"timestamp":"timestamp2","x":"x2","y":"y2"}))
    df2.at[df2.index[-1],"timestamp2"] = df2.iloc[-1]["timestamp"] + 1
    df2["dtime"] = df2.apply(lambda row: int(row["timestamp2"] - row["timestamp"]), axis=1)
    
    df2["vx"]=df2.apply(lambda row: (row.x2 - row.x)/row.dtime, axis =1)
    df2["vy"]=df2.apply(lambda row: (row.y2 - row.y)/row.dtime, axis =1)
    df2["vx2"] = df2["vx"].ewm(alpha=alpha).mean()
    df2["vy2"] = df2["vy"].ewm(alpha=alpha).mean()
    
    if type=="local":
        df2["new_x2"] = df2.apply(lambda row: row.x + row.dtime*row.vx2, axis =1)
        df2["new_y2"] = df2.apply(lambda row: row.y + row.dtime*row.vy2, axis =1)
        df2["new_x"] = df2["new_x2"].shift(1)
        df2["new_y"] = df2["new_y2"].shift(1)
        df2.at[0,"new_x"] = df2.iloc[0]["x"]
        df2.at[0,"new_y"] = df2.iloc[0]["y"]
        df3 = df2[["new_x","new_y","timestamp"]].rename(columns={"new_x":"x", "new_y":"y"})
    else:
        pass
        # need to complete it
    return df3


# clean and add speed and bearing to the data
# 1 - convert to x, y
# 2 - remove outlier
# 3 - smoothing vx and vy and update x,y
# 4 - calculate bearing and speed
# 5 - assing zero to speeds less than MINSPEED_FOR_MOVING
# 6 - update bearings based on speeds less than MINSPEED_FOR_BEARING
# 7 - assing stopindex and fix locations in stop cases

def richdata(obs, net, id=0, alpha=.7, type="local", MIN_SPEED=1, MAX_SPEED_FOR_OUTLIER=50, MINSPEED_FOR_BEARING = 2):
    df = dfLonLat2XY(obs, net)
    df1 = removeOutlier(df)
    df2 = smoothingPoint(df1, alpha=alpha, type=type)
    
    output = []
    row_current = df2.iloc[0]
    point_current = (row_current["x"], row_current["y"])
    timestamp_current = row_current["timestamp"]
    stopsnumber = 1

    for index in range(1,len(df2)):
        row_next = df2.iloc[index]
        point_next = (row_next["x"],row_next["y"])
        timestamp_next  = row_next["timestamp"]
        dist = distance2d(point_current, point_next)
        speed_current = dist/(timestamp_next-timestamp_current)
        if speed_current < MIN_SPEED :
            speed_current = 0 
        bearing_current = calculate_bearing_angle(point_current, point_next)
        
        if speed_current < MAX_SPEED_FOR_OUTLIER: # max speed for outlier detection
    
            if speed_current < MINSPEED_FOR_BEARING and len(output) > 0: #minimum speed for calculating a new bearing
                bearing_current = output[-1]["bearing"]
                
            if speed_current>0:
                stopindex = 0
            else:
                if len(output) and output[-1]["stopindex"] == 0:
                    stopsnumber+=1
                stopindex = stopsnumber
    
            output.append({"id":id, "x":point_current[0], 'y':point_current[1], 'timestamp':int(timestamp_current),
                          "speed":speed_current, 'bearing':bearing_current, "stopindex":stopindex})
            point_current = point_next
            timestamp_current = timestamp_next
            row_current = row_next
    
    
    speed = output[-1]["speed"]
    bearing = output[-1]["bearing"]
    
    if speed < MINSPEED_FOR_BEARING and len(output) > 0:  #minimum speed for calculating a new bearing
        bearing = output[-1]["bearing"]

    if speed_current>0:
        stopindex = 0
    else:
        if len(output) and output[-1]["stopindex"] == 0:
            stopsnumber+=1
        stopindex = stopsnumber
    
    output.append({"id":id,"x":point_current[0], 'y':point_current[1], 'timestamp':int(timestamp_current),
                          "speed":speed, 'bearing':bearing, "stopindex":stopindex})
    
    df_output = pd.DataFrame(output)

# assign the same 'x' and 'y' based on median to every consecutive point with the same stopindex
    return xystop_point_editing(df_output)
