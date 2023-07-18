import pandas as pd
from geotools import distance2d 
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

            if speed_current < 1 and len(output) > 1: #minimum speed for calculating a new bearing
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

    if speed < 1 and len(output) > MINSPEED_FOR_BEARING:  #minimum speed for calculating a new bearing
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

    return df