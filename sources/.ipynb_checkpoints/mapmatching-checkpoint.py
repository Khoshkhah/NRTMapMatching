#import networkx as nx
#import numpy as np
import pandas as pd
import time

import network
from network import combineShapesSumo,getGeoShape
from geotools import distance2d,offsetBearing,polyLength,road_distance, polygonOffsetWithMinimumDistanceToPoint




class MapMatcher:
    
    def __init__(self, net, MAX_GPS_ERROR=60, 
                 MAX_MAP_ERROR=40,
                 MAP_ONE_WAY_FIX=True,
                 U_TURN_ON_ONEWAY=False,
                 LOOP=True,
                 MAX_SPEED=100,
                 DIFF_GPS_ERROR=10,
                 MAX_RUNNING_TIME=5):
        
        if not isinstance(net, network.Net):
            raise ValueError("network.Net expected")
            
        self.net = net
        self.radius = MAX_GPS_ERROR + MAX_MAP_ERROR
        self.matchdf = None
        self.routedf = None
        
        self.MAX_GPS_ERROR = MAX_GPS_ERROR #meters
        self.MAX_MAP_ERROR = MAX_MAP_ERROR #meters
        self.MAP_ONE_WAY_FIX = MAP_ONE_WAY_FIX
        self.U_TURN_ON_ONEWAY = U_TURN_ON_ONEWAY
        self.LOOP = LOOP
        self.MAX_SPEED = MAX_SPEED #m/s
        self.MAX_RUNNING_TIME = MAX_RUNNING_TIME  #s
        self.DIFF_GPS_ERROR = DIFF_GPS_ERROR #meters
        self.path= []  # item: {"edge":,"reverese":,"length":}
        self.MINSPEED_BEARING = 1 #m/s
        
        
    
    def reset(self):
        self.matchdf = None
        self.routedf = None
        
        
     
    
    def decision_stay_change_nodecide(self,remind_offset, speed, maxspeed, deltaTime):
        """
        Determines the decision (stay, change, or no decision) based on edge length, offset, speed, and time.

        Args:

            remind_offset (float): the length of rest of the edge after offset.
            speed (float): Current speed.
            deltaTime (float): Time interval.

        Returns:
            str: Decision (stay, change, or no decision).

        """

        if remind_offset < (speed * deltaTime) - self.DIFF_GPS_ERROR:  # RADIUS
            return "CHANGE"
        elif remind_offset >= (maxspeed * deltaTime) + self.DIFF_GPS_ERROR:  # RADIUS
            return "STAY"
        else:
            return "NODECISION"
    

    
    
    def first_point_matching(self,x,y):
        
        #x, y = self.net.convertLonLat2XY(lon, lat)
        edges = self.net.getNeighboringEdges(x, y, self.radius)
        edges = [edge for edge,dist in edges]
        reversedict = {edge:False for edge in edges}
        return set(edges),reversedict
    
    
    
    
    def cost_calculate(self, bearing_error, match_point_distance, air_distance_error, road_distance_error, reverse):
        """
        Calculates the cost for a match point based on different error values.

        Args:
            bearing_error (float): Bearing error for the match point.
            match_point_distance (float): Distance to the match point.
            air_distance_error (float): Air distance error for the match point.
            road_distance_error (float): Road distance error for the match point.

        Returns:
            float: Cost value for the match point.

        """
        cost = 0
        if reverse == True:
            cost+= 100000
        cost+= 1*bearing_error + 30*match_point_distance  + 10*air_distance_error  +5*road_distance_error
        return  cost

    
    def outgoinglist(self, edge, reverse):
        """
        Retrieves a list of outgoing edge IDs from a given edge.

        Args:
            edge (SumoEdge): Current Sumo edge.

        Returns:
            list: List of outgoing edge IDs.

        """
        #out = list(edge.getOutgoing().keys())
        if self.MAP_ONE_WAY_FIX: 
            if reverse==True:
                node = edge.getFromNode()
            else:
                node = edge.getToNode()

            out = node.getOutgoing() + node.getIncoming()
            if (not self.U_TURN_ON_ONEWAY) and (edge in out):
                out.remove(edge)
            
            path = [item["edge"] for item in self.path]
            
            if( not self.LOOP):
                for item in path:
                    if item in out:
                        out.remove(item)
            reversedict={edge:True for edge in node.getIncoming()}

            reversedict.update({edge:False for edge in node.getOutgoing()})
        else:
            node = edge.getToNode()
            out = node.getOutgoing()
            if( not self.LOOP):
                for item in path:
                    if item in out:
                        out.remove(item)
            reversedict={edge:False for edge in out}


        return set(out),reversedict


        #print({edge:False for edge in tonode.getOutgoing()})
        #return tonode.getOutgoing(), {edge:False for edge in tonode.getOutgoing()}

    
    
    
    
    def match(self, sample_gps):
        """
        Matches GPS observations to road network edges.

        Args:
            observations (DataFrame): GPS observations DataFrame.

        Returns:
            DataFrame: DataFrame containing the matched points.

        """
        #sample_gps = self.reconstruct_observations(observations)
        if not isinstance(sample_gps, pd.DataFrame):
            return 0
        sample_gps.to_csv("new_sample.csv",index=False)
        
        #initialpoint
        p = sample_gps.iloc[0]
        matchedpoints = []
        edges, reversedict = self.first_point_matching(p["x"],p["y"])
        #print(reversedict)
        myindex = 0
        last_edge = None
        last_offset = None
        #current_time = sample_gps.iloc[0]["timestamp"]
        #next_time = sample_gps.iloc[1]["timestamp"]
        decisionlist = []
        decisionlist.append({"index":0, "result":set(edges), "last_edge":None, "offset":None ,"reversedict":reversedict})
        includeJunctions = False
  
        last_edge_reverse = None
        
        start_time = time.time()


        current_time = time.time()

        # other point matching
        while(myindex <len(sample_gps)):
      
            for index,row in sample_gps.iloc[myindex:].iterrows():
                if last_edge!= decisionlist[-1]["last_edge"]:
                    raise ValueError("error in the algorithm")

                check=False
                if last_edge!=None:
                    remind_offset = self.path[-1]["length"]-last_offset
                    #print(f"index = {index}, edge={last_edge.getID()} , last_offset = {last_offset},remind_offset = {remind_offset}")
                    decision = self.decision_stay_change_nodecide(remind_offset,
                                                             row.speed,last_edge.getSpeed(), 1)
                    check =True
                else:
                    decision = "CHANGE"
                    edges = decisionlist[-1]["result"]
                    reversedict = decisionlist[-1]["reversedict"]
                    #reversedict remove lastedge


                if decision == "STAY":
                    edges = {last_edge}
                    reversedict = {last_edge:last_edge_reverse}

                elif decision == "CHANGE" and last_edge!=None:
                    edges, reversedict = self.outgoinglist(last_edge, last_edge_reverse)

                elif decision == "NODECISION":
                    edges, reversedict = self.outgoinglist(last_edge, last_edge_reverse)    
                    edges.add(last_edge)
                    reversedict[last_edge] = last_edge_reverse
                


                #print(f"index = {index}, decision = {decision}")
                #print(decisionlist[-1]["result"])
                #if index+1 < len(sample_gps):
                #    next_time = sample_gps.iloc[index+1]["timestamp"]
                x, y = row.x, row.y
                edgesinfo = []
                

                if len(edges) > 0:
                    for edge in edges:

                       
                        if (edge!=last_edge and last_edge!=None):
                            temp_edge = last_edge
                            #print(f"last_edge = {last_edge}, reversedict = {reversedict}")
                            temp_reverse = last_edge_reverse
                            #print(f"{index} , last_edge_reverse = {last_edge_reverse}")

                        elif len(self.path)<2:
                            temp_edge = None
                            temp_reverse = False
                        else:
                            temp_edge = self.path[-2]["edge"]
                            temp_reverse = self.path[-2]["reverse"]
                            #print(f"{index} , from_reverse = {from_reverse}")

                        currentedge_shape = combineShapesSumo(edge,temp_edge, reversedict[edge],temp_reverse)
                        #print(f"index = {index}, edge = {edge.getID()}, temp_edge={temp_edge}")
                        #print(f" reverseedge={reversedict[edge]}, temp_reverse = {temp_reverse}")

                        currentedge_length = polyLength(currentedge_shape)
                        #print(currentedge_length)
                        tempedge_length = polyLength(currentedge_shape)
                        offset , matchpoint= polygonOffsetWithMinimumDistanceToPoint([x,y], currentedge_shape)
                        #if(len(matchedpoints)>0):
                        #    if(last_edge!=None):
                        #        if(edge==last_edge):
                        #            offset = max(offset, last_offset)

                        #matchpoint = geomhelper.positionAtShapeOffset(currentedge_shape, offset)

                        dist = distance2d(matchpoint, (x,y))

                        matched_bearing = offsetBearing(currentedge_shape, offset)
                        dist_bearing = min(abs(row.bearing - matched_bearing), 360-abs(row.bearing - matched_bearing))

                        if(len(matchedpoints)>0):
                            lastedgeinfo = matchedpoints[-1]
                            last_edge_length = lastedgeinfo["edge_length"]
                            lastedge = self.net.getEdge(lastedgeinfo["edgeid"])
                            lastoffset = lastedgeinfo["offset"]
                            lasttimestamp = lastedgeinfo["timestamp"]
                            last_sample =(lastedgeinfo["x_sample"],lastedgeinfo["y_sample"])
                            last_matched= (lastedgeinfo["x"],lastedgeinfo["y"])

                        else:
                            lastedge = edge
                            lastoffset = offset
                            lasttimestamp = row.timestamp
                            last_sample = (x,y)
                            last_matched= matchpoint
                            last_edge_length = 0

                        air_sample_distance = distance2d(last_sample, (x,y)) #row.distance 
                        air_matched_distance = distance2d(last_matched, matchpoint)
                        cost_air = abs(air_sample_distance - air_matched_distance)

                        speed = row.speed
                        deltaTime = 1 #(int(row.timestamp) - int(lasttimestamp))

                        #print(f"lattimestamp = {int(lasttimestamp)}, currenttimestamp = {int(row.timestamp)}, delta = {deltaTime}")
                        predict_distance = float(speed) * deltaTime
                        matched_road_distance =  road_distance(edge, offset, lastedge , lastoffset, last_edge_length)
                        #print(f"index = {index}, edge = {edge.getID()}, offset={offset} lastoffset={lastoffset}")
                        #print(f"lastge={lastedge.getID()}, lastedgelen = {last_edge_length} roaddistance = {matched_road_distance}")

                        # rd is based on speed
                        rd = abs(matched_road_distance - predict_distance)

                        #if matched_road_distance == 0 and predict_distance > 3 and offset!=0:
                        #    rd = rd*50
                        cost = 1*dist_bearing + 10*dist  + 10*cost_air  +10*rd
                        cost = self.cost_calculate(dist_bearing, dist, cost_air, rd, reversedict[edge])

                        edgesinfo.append({"edge":edge, "dist":dist, "matchbearing":matched_bearing, "timestamp":row.timestamp,
                                          "predict_distance":predict_distance, "matched_road_distance": matched_road_distance,
                                          "speed":speed, "cost_air":cost_air,"from_edge":temp_edge.getID() if temp_edge!=None else None,
                                         "cost":cost, "matchpoint":matchpoint, "offset":offset, "distbearing":dist_bearing, "rd":rd,
                                         "edge_length":currentedge_length, "from_edge_reverse":temp_reverse})


                    bestedgeinfo = min(edgesinfo, key=lambda item:item["cost"])

                    matchpoint = bestedgeinfo["matchpoint"]
                    matchbearing = bestedgeinfo["matchbearing"]
                    bestedge = bestedgeinfo["edge"]
                    offset = bestedgeinfo["offset"]
                    distbearing = bestedgeinfo["distbearing"]
                    x,y = matchpoint
                    #if index==176:
                            #display(pd.DataFrame(edgesinfo))
           
                    if (bestedgeinfo["dist"] >  self.radius): # back to first changing edge and start on that point as initial point
                        if bestedge in decisionlist[-1]["result"]:
                            decisionlist[-1]["result"].remove(bestedge)

                        counter = 0
                        while len(decisionlist)>0 and len(decisionlist[-1]["result"])==0:
                            counter+=1
                            decisionlist.pop()
                            #print(f"befor pop = {len(self.path)}")
                            self.path.pop()
                            #print(f"after pop = {len(self.path)}")
                        
                        if counter == 0:
                            self.path.pop()
                            
                        if len(decisionlist)==0:
                            print("error: decision list is empty!")
                            return 0
                       #if counter > 5:
                       #     print(f"map error in index {index} and timestamp {row.timestamp} ...")
                       #     return 0
                        decisionlist[-1]["offset"] = None

                        decisionlist[-1]["last_edge"] = None
                        decisionlist[-1]["last_edge_reverse"] = None
                        #print(decisionlist[-1]["result"])
                        myindex = decisionlist[-1]["index"]
                        #print(f"len(decision) = {len(decisionlist)}")
                        print(f"decision back index = {myindex}")
                        self.show_path()
                       
                        try:
                            while len(matchedpoints) > 0 and matchedpoints[-1]["index"]>=myindex :
                                    matchedpoints.pop()
                        except IndexError:
                            print("indexError")
                            return 0
                        
                        #ttt = [len(item["result"]) for item in decisionlist]
                        #edgename = [e.getID() for e in decisionlist[-1]["result"]]
                        #print(f"{ttt}, edges = {edgename}")

                        last_offset = None
                        last_edge = None
                        last_edge_reverse = None


                        break

                    else:
                        last_offset = offset

                        matchedpoints.append({"index":index, "timestamp":row.timestamp, "x":x, "y":y,"bearing":row.bearing,
                                              "rd":bestedgeinfo["rd"],"dist":bestedgeinfo["dist"],"from_edge":bestedgeinfo["from_edge"],
                                              "predict_distance":bestedgeinfo["predict_distance"],"speed":bestedgeinfo["speed"],
                                              "matched_road_distance": bestedgeinfo["matched_road_distance"],"y_sample":row.y,
                                              "x_sample":row.x, "cost_air": bestedgeinfo["cost_air"],"decitsion":decision,
                                              "matchbearing":matchbearing, "edgeid":bestedge.getID(), "offset":offset, "distbearing":distbearing,
                                             "edge_length":bestedgeinfo["edge_length"],"type":row["type"], "edge_reverse":reversedict[bestedge],
                                            "from_edge_reverse":bestedgeinfo["from_edge_reverse"]})
 
                        
                        if last_edge==None:   # first iteration after initial point or back to change path
                            decisionlist[-1]["last_edge"]=bestedge
                            decisionlist[-1]["last_edge_reverse"]=reversedict[bestedge]

                            decisionlist[-1]["offset"]=offset
                            decisionlist[-1]["result"].remove(bestedge)
                            decisionlist[-1]["reversedict"] = reversedict
                        
                            
                            if len(self.path)>0:
                                from_edge = self.path[-1]["edge"]
                                from_reverse = self.path[-1]["reverse"]
                            else:
                                from_edge = None
                                from_reverse = False
                                
                            edgelength = polyLength(combineShapesSumo(bestedge,from_edge,
                                                                                   reversedict[bestedge], from_reverse))
                            
                            self.path.append({"edge":bestedge, "reverse":reversedict[bestedge],
                                             "length":edgelength})
                            
                    
                            last_offset = offset
                            last_edge = bestedge
                            last_edge_reverse = reversedict[bestedge]
                                #print(f"len(decision) = {len(decisionlist)}")

                            print(f"decision index start = {index}")
                            self.show_path()


                        elif bestedge==last_edge: # stay on edge
                            decisionlist[-1]["offset"]=offset
                            last_offset = offset




                        elif bestedge!=last_edge :#and last_edge!=None: # changing edge

                            if last_edge in edges:
                                edges.remove(last_edge)

                            edges.remove(bestedge)
                           
                            if len(self.path)>0:
                                from_edge = self.path[-1]["edge"]
                                from_reverse = self.path[-1]["reverse"]
                            else:
                                from_edge = None
                                from_reverse = False
                                
                            edgelength = polyLength(combineShapesSumo(bestedge,from_edge,
                                                                                   reversedict[bestedge], from_reverse))
                            
                            self.path.append({"edge":bestedge, "reverse":reversedict[bestedge],
                                             "length":edgelength})

                            last_edge = bestedge
                            last_offset = offset
                            last_edge_reverse = reversedict[bestedge]

                            if(len(edges)>=0): 
                                decisionlist.append({"index":index, "result":edges, "last_edge":bestedge,
                                                     "offset":last_offset, "last_edge_reverse":reversedict[bestedge],
                                                    "reversedict":reversedict})
                                
                                #ttt = [len(item["result"]) for item in decisionlist]
                                #index_list = [item["index"] for item in decisionlist]
                                #mpath = [item["last_edge"].getID() for item in decisionlist]
                                #print(f"len(decision) = {len(decisionlist)}")

                                print(f"decision index = {index}")
                                self.show_path()

                                #print(f"path = {mpath}")


                        if last_edge!= decisionlist[-1]["last_edge"]:
                            raise ValueError("error 2 in the algorithm.")
                        
                        myindex = index
            
                else:
                    raise ValueError("error in the algorithm. ln(edges)==0")
            if index== len(sample_gps) - 1:
                myindex = index+1
            current_time = time.time()

            # Calculate the elapsed time
            if(current_time - start_time) > 10:
                print(f"runnig time is more than {self.MAX_RUNNING_TIME} seconds.")
                return 0
        self.matchdf = pd.DataFrame(matchedpoints)

        return 1
    
    
    
        
    def save_routematch(self, routematchfile=None):
        """
        Saves the route matching results to a file and returns the route dataframe.

        Args:
            routematchfile (str): File path to save the route matching results (optional).

        Returns:
            pandas.DataFrame: Route dataframe containing the route matching results.

        """
        # Create a new dataframe with unique from_edge and edgeid combinations
        self.routedf = self.matchdf[["from_edge", "edgeid","edge_reverse","from_edge_reverse"]].drop_duplicates()
        self.routedf = self.routedf.set_index(["from_edge", "edgeid","edge_reverse","from_edge_reverse"])

        # Calculate departure, arrival, stop time, and travel time for each from_edge and edgeid combination
        self.routedf["departure"] = self.matchdf.groupby(["from_edge", "edgeid"], dropna=False)["timestamp"].first()
        self.routedf["arrival"] = self.matchdf.groupby(["from_edge", "edgeid"], dropna=False)["timestamp"].last()
        self.routedf["stop_time"] = self.matchdf.groupby(["from_edge", "edgeid"], dropna=False)["speed"].apply(lambda col: sum([1 if x == 0 else 0 for x in col]))
        self.routedf["travel_time"] = self.routedf.apply(lambda row: int(row.arrival - row.departure), axis=1)
        self.routedf.reset_index(inplace=True)

        # Get the shape for each edgeid and from_edge combination
        
        self.routedf["shape"] = self.routedf.apply(lambda x: getGeoShape(self.net.getEdge(x.edgeid),
                                                             self.net,
                                                             edge_reverse=x.edge_reverse,
                                                             from_reverse=x.from_edge_reverse,
                                                             fromedge=self.net.getEdge(x.from_edge) if isinstance(x.from_edge, str) else None), axis=1)

        if routematchfile is not None:
            self.routedf.to_csv(routematchfile, index=False)
        return self.routedf

    

    
    def save_pointmatch(self, pointmatchfile=None):
        """
        Saves the point matching results to a file and returns the match dataframe.

        Args:
            pointmatchfile (str): File path to save the point matching results (optional).

        Returns:
            pandas.DataFrame: Match dataframe containing the point matching results.

        """
        if pointmatchfile is not None:
            self.matchdf.to_csv(pointmatchfile, index=False)
        return self.matchdf

    def show_path(self):
        print([item["edge"].getID() for item in self.path])


# Example usage
# Create a directed graph


# Create a map matcher instance

# Define a sequence of observations

# Perform map matching
