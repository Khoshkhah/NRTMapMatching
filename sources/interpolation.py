from geotools import distance2d,calculate_bearing_angle
import pandas as pd
import numpy as np

def Bezier_3(p0,p1,p2,p3,t):
    b0 = (1-t)**3
    b1 = 3*((1-t)**2)*t
    b2 = 3*(1-t)*(t**2)
    b3 = t**3   
    p = b0*p0 + b1*p1 + b2*p2 + b3*p3
    return p


def Bezier_2(p0,p1,p2,t):
    b0 = (1-t)**2
    b1 = 2*(1-t)*t
    b2 = (t**2)    
    p=b0*p0 + b1*p1 + b2*p2
    return p


def derivative_Bezier_3(p0,p1,p2,p3,t):
    q0 = 3*(p1-p0)
    q1 = 3*(p2-p1)
    q2 = 3*(p3-p2)  
    derivative = Bezier_2(q0,q1,q2,t)
    
    return derivative
   
    
def derivative2Bearing(p):
    if p[0]!= 0:
        slope = p[1]/p[0]
    else:
        slope = np.inf
    
    angle = np.arctan(slope) * 180/ np.pi
    bearing = 90 - angle
    
    if p[0] < 0 or (p[0]==0 and p[1]<0):
        bearing += 180 
    return bearing



def bezierInterpolation(a,b, sample_rate=1):
    items = []
    delta_time = b["timestamp"] - a["timestamp"]
    point_a = np.array([a['x'], a["y"]]) 
    point_b = np.array([b['x'], b['y']]) 
    
    if a["stopindex"]>0 and b["stopindex"]>0:
        time = a["timestamp"]
        while time  <= b["timestamp"]:

            if time==a["timestamp"] or time==b["timestamp"]:
                types = "origin"
            else:
                types = "extra"
            items.append({'x':point_a[0], 'y':point_a[1],  "timestamp":int(time),
                    "speed":0, "bearing":a["bearing"], "type":types, "stopindex":a["stopindex"]})
            
            time+=sample_rate
            if time > b['timestamp'] and (time -sample_rate)!= b['timestamp']:
                time = b['timestamp']

        return pd.DataFrame(items)
    
    dist = distance2d(tuple(point_a), tuple(point_b))
    #dist = (a["speed"] + b["speed"] + 2)*delta_time/2
    
    #dist = max(distance2d(point_a, point_b), (a["speed"] + b["speed"] + 2)*delta_time/2)

    v_a_x = (a["speed"]+1)*np.sin(a["bearing"]* np.pi / 180.)
    v_a_y = (a["speed"]+1)*np.cos(a["bearing"]* np.pi / 180.)
    v_b_x = (b["speed"]+1)*np.sin(b["bearing"]* np.pi / 180.)
    v_b_y = (b["speed"]+1)*np.cos(b["bearing"]* np.pi / 180.)

    v_a = np.array((v_a_x, v_a_y))
    v_b = np.array((v_b_x, v_b_y))

    control = 10+max(a["speed"], b["speed"])**(1.2)
    alpha = ((.5*(np.sqrt(a["speed"]+1))/(np.sqrt(a["speed"]+1) + control))) * dist*3
    beta =  ((.5*(np.sqrt(b["speed"]+1))/(np.sqrt(b["speed"]+1) + control))) * dist*3

   # control = 50
    #alpha = (.5*(a["speed"]+1)/((a["speed"]+1) + control)) * dist*3
    #beta =  (.5*(b["speed"]+1)/((b["speed"]+1) + control)) * dist*3
    #alpha = .1 + delta_time * (a["speed"]+1)/5
    #beta = .1 + delta_time * (b["speed"]+1)/5


    p0 = point_a + (alpha * (v_a)) /3.0
    p1 = point_b - (beta * (v_b)) /3.0

    time = 0
    zero_point = np.array((0,0))

    while time + a["timestamp"] <= b["timestamp"]:
        
        t = time/delta_time
        p_t = Bezier_3(point_a,p0,p1,point_b,t)
        

        if t==0:
            types = "origin"
            p_t_1 = Bezier_3(point_a,p0,p1,point_b,t+(sample_rate/delta_time))
            speed_t = distance2d(p_t, p_t_1)/sample_rate

        elif t==1:
            types = "origin"
            speed_t = None
        else:
            types = "extra"
            p_t_1 = Bezier_3(point_a,p0,p1,point_b,t+(sample_rate/delta_time))
            speed_t = distance2d(p_t, p_t_1)/sample_rate
            
        v_t = derivative_Bezier_3(point_a,p0,p1,point_b,t)
        if ((1-t)*alpha+t*beta) == 0:
            print(f"alpha = {alpha}, beta = {beta}, t = {t}, dist={dist}")
            
        #speed_t = max(distance2d(zero_point, v_t)/((1-t)*alpha+t*beta) - 1, 0)
    
        bearing_t = calculate_bearing_angle(zero_point, v_t)
        items.append({"x":p_t[0], 'y':p_t[1], "timestamp":int(time + a["timestamp"]),
                    "speed":speed_t, "bearing":bearing_t, "type":types,"stopindex":0})

        
        time = time + sample_rate
        if time > delta_time and (time -sample_rate)!= delta_time:
            time = delta_time
            
    df = pd.DataFrame(items)
    return df




def interpolateTrajectory(traj, sample_rate=1):
    interpol_list = []
    time = traj.iloc[0]["timestamp"]
    a = dict(traj.iloc[0])
    for index,row in traj[1:-1].iterrows():
            b=dict(row.copy())
            mydict = bezierInterpolation(a,b, sample_rate=sample_rate)            
            interpol_list.append(mydict)
            a = b.copy()
            if a["stopindex"]>0:
                a["x"] = mydict.iloc[-1]["x"]
                a["y"] = mydict.iloc[-1]["y"]

            time = row["timestamp"]

    last_speed = traj["speed"].iloc[-1]
    lastindex = interpol_list[-1].index[-1]
    interpol_list[-1].at[lastindex,"speed"] = last_speed
    df = pd.concat(interpol_list).reset_index(drop=True)
    df = df.dropna()
    return df

