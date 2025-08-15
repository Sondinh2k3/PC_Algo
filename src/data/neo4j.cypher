// Create Traffic Light Nodes with spatial coordinates for layout
// Red circled traffic lights (gating_intersection)
CREATE (tl1:Cross {name: "TL_North_Gate", id: "1166230678", type: "gating_intersection"})
CREATE (tl2:Cross {name: "TL_West_Gate_1", id: "1677153107", type: "gating_intersection"})
CREATE (tl3:Cross {name: "TL_West_Gate_2", id: "1677153087", type: "gating_intersection"})
CREATE (tl4:Cross {name: "TL_East_Gate", id: "357410392", type: "gating_intersection"})
CREATE (tl5:Cross {name: "TL_South_Gate", id: "TL0cluster3081188939_4750234661_9502375205", type: "gating_intersection"})

// Inner traffic lights (inner_intersection) - 4 total
CREATE (tl6:Cross {name: "TL_Central_1", id: "1166229128", type: "inner_intersection"})
CREATE (tl7:Cross {name: "TL_Central_2", id: "445389106", type: "inner_intersection"})
CREATE (tl8:Cross {name: "TL_Central_3", id: "1166229489", type: "inner_intersection"})
CREATE (tl9:Cross {name: "TL_Central_4", id: "2763613224", type: "inner_intersection"})

// Outside intersection
CREATE (tl10:Cross {name: "TL_West_Outside", id: "TL010", type: "outside_intersection"})
CREATE (tl11:Cross {name: "TL_South_Outside_1", id: "TL011", type: "outside_intersection"})
CREATE (tl12:Cross {name: "TL_South_Outside_2", id: "TL012", type: "outside_intersection"})
CREATE (tl13:Cross {name: "TL_East_Outside_1", id: "TL013", type: "outside_intersection"})
CREATE (tl14:Cross {name: "TL_East_Outside_2", id: "TL014", type: "outside_intersection"})
CREATE (tl15:Cross {name: "TL_North_Outside_Left", id: "TL015", type: "outside_intersection"})
CREATE (tl16:Cross {name: "TL_North_Outside_Top", id: "TL016", type: "outside_intersection"})


// No trafficlight intersection (no traffic light)
CREATE (tl17:Cross {name: "TL_No_Traffic_Light1", id: "95024799", type: "no_traffic_light"})
CREATE (tl18:Cross {name: "TL_No_Traffic_Light2", id: "3413819978", type: "no_traffic_light"})
CREATE (tl19:Cross {name: "TL_No_Traffic_Light3", id: "cluster4081631808_7723566408", type: "no_traffic_light"})
CREATE (tl20:Cross {name: "TL_No_Traffic_Light4", id: "357410351", type: "no_traffic_light"})
CREATE (tl21:Cross {name: "TL_No_Traffic_Light5", id: "1166229125", type: "no_traffic_light"})
CREATE (tl22:Cross {name: "TL_No_Traffic_Light6", id: "7075295154", type: "no_traffic_light"})
CREATE (tl23:Cross {name: "TL_No_Traffic_Light7", id: "1166229127", type: "no_traffic_light"})
CREATE (tl24:Cross {name: "TL_No_Traffic_Light8", id: "1677153109", type: "no_traffic_light"})
CREATE (tl25:Cross {name: "TL_No_Traffic_Light9", id: "1677153111", type: "no_traffic_light"})
CREATE (tl26:Cross {name: "TL_No_Traffic_Light10", id: "2763613227", type: "no_traffic_light"})
CREATE (tl27:Cross {name: "TL_No_Traffic_Light11", id: "2763613228", type: "no_traffic_light"})

// Create bidirectional APPROACH_TO relationships
// outside_left_north to north gate
CREATE (tl15)-[:APPROACH_TO {length: 1454.89, lane: "-100954245#13_0", id: "-100954245#13"}]->(tl1)
CREATE (tl1)-[:APPROACH_TO {length: 1454.63, lane: "100954245#0_0", id: "100954245#0"}]->(tl5)

// outside_top_north to north gate
CREATE (tl16)-[:APPROACH_TO {length: 2041.07, lane: "1149582046#0_0, 1149582046#0_1", id: "1149582046#0"}]->(tl1)
CREATE (tl1)-[:APPROACH_TO {length: 2041.07, lane: "-1149582046#3_0, -1149582046#3_1", id: "-1149582046#3"}]->(tl6)

//triangle north cross
CREATE (tl1)-[:APPROACH_TO {length: 53.47, lane: "163652291_0", id: "163652291"}]->(tl17)
CREATE (tl17)-[:APPROACH_TO {length: 53.47, lane: "-163652291_0", id: "-163652291"}]->(tl1)

CREATE (tl1)-[:APPROACH_TO {length: 101.83, lane: "723990056_0", id: "723990056"}]->(tl20)
CREATE (tl20)-[:APPROACH_TO {length: 101.83, lane: "-723990056_0", id: "-723990056"}]->(tl1)

CREATE (tl20)-[:APPROACH_TO {length: 85.07, lane: "723990054_0", id: "-723990054"}]->(tl17)
CREATE (tl17)-[:APPROACH_TO {length: 85.07, lane: "723990054_0", id: "723990054"}]->(tl20)















CREATE (tl1)-[:APPROACH_TO {length: 245.5, lane: 2, id: "-723990057#2"}]->(tl6)
CREATE (tl6)-[:APPROACH_TO {length: 245.5, lane: 2, id: "REL002"}]->(tl1)

// West Gate 1 connections (to North Gate, Central_1, and Central_4)
CREATE (tl2)-[:APPROACH_TO {length: 280.3, lane: 2, id: "REL003"}]->(tl1)
CREATE (tl1)-[:APPROACH_TO {length: 280.3, lane: 2, id: "REL004"}]->(tl2)

CREATE (tl2)-[:APPROACH_TO {length: 220.7, lane: 2, id: "REL005"}]->(tl6)
CREATE (tl6)-[:APPROACH_TO {length: 220.7, lane: 2, id: "REL006"}]->(tl2)

CREATE (tl2)-[:APPROACH_TO {length: 240.5, lane: 2, id: "REL025"}]->(tl9)
CREATE (tl9)-[:APPROACH_TO {length: 240.5, lane: 2, id: "REL026"}]->(tl2)

// West Gate 2 connections (to Central_4)
CREATE (tl3)-[:APPROACH_TO {length: 180.4, lane: 2, id: "REL007"}]->(tl9)
CREATE (tl9)-[:APPROACH_TO {length: 180.4, lane: 2, id: "REL008"}]->(tl3)

// Outside intersection connections (to both West Gates)
CREATE (tl10)-[:APPROACH_TO {length: 150.5, lane: 2, id: "REL027"}]->(tl2)
CREATE (tl2)-[:APPROACH_TO {length: 150.5, lane: 2, id: "REL028"}]->(tl10)

CREATE (tl10)-[:APPROACH_TO {length: 200.8, lane: 2, id: "REL029"}]->(tl3)
CREATE (tl3)-[:APPROACH_TO {length: 200.8, lane: 2, id: "REL030"}]->(tl10)

// South outside intersection connections (to South Gate)
CREATE (tl11)-[:APPROACH_TO {length: 120.5, lane: 2, id: "REL031"}]->(tl5)
CREATE (tl5)-[:APPROACH_TO {length: 120.5, lane: 2, id: "REL032"}]->(tl11)

CREATE (tl12)-[:APPROACH_TO {length: 110.3, lane: 2, id: "REL033"}]->(tl5)
CREATE (tl5)-[:APPROACH_TO {length: 110.3, lane: 2, id: "REL034"}]->(tl12)

// East outside intersection connections (to East Gate)
CREATE (tl13)-[:APPROACH_TO {length: 125.7, lane: 2, id: "REL035"}]->(tl4)
CREATE (tl4)-[:APPROACH_TO {length: 125.7, lane: 2, id: "REL036"}]->(tl13)

CREATE (tl14)-[:APPROACH_TO {length: 115.2, lane: 2, id: "REL037"}]->(tl4)
CREATE (tl4)-[:APPROACH_TO {length: 115.2, lane: 2, id: "REL038"}]->(tl14)

// North outside intersection connections (to North Gate)
CREATE (tl15)-[:APPROACH_TO {length: 105.8, lane: 2, id: "REL039"}]->(tl1)
CREATE (tl1)-[:APPROACH_TO {length: 105.8, lane: 2, id: "REL040"}]->(tl15)

CREATE (tl16)-[:APPROACH_TO {length: 105.8, lane: 2, id: "REL041"}]->(tl1)
CREATE (tl1)-[:APPROACH_TO {length: 105.8, lane: 2, id: "REL042"}]->(tl16)

// East Gate connections (to North Gate, South Gate, and Central_3)
CREATE (tl4)-[:APPROACH_TO {length: 280.5, lane: 2, id: "REL009"}]->(tl1)
CREATE (tl1)-[:APPROACH_TO {length: 280.5, lane: 2, id: "REL010"}]->(tl4)

CREATE (tl4)-[:APPROACH_TO {length: 195.2, lane: 2, id: "REL011"}]->(tl8)
CREATE (tl8)-[:APPROACH_TO {length: 195.2, lane: 2, id: "REL012"}]->(tl4)

CREATE (tl4)-[:APPROACH_TO {length: 320.8, lane: 2, id: "REL013"}]->(tl5)
CREATE (tl5)-[:APPROACH_TO {length: 320.8, lane: 2, id: "REL014"}]->(tl4)

// South gate connections
CREATE (tl7)-[:APPROACH_TO {length: 275.9, lane: 2, id: "REL015"}]->(tl5)
CREATE (tl5)-[:APPROACH_TO {length: 275.9, lane: 2, id: "REL016"}]->(tl7)

// Inner intersection connections
CREATE (tl6)-[:APPROACH_TO {length: 180.3, lane: 3, id: "REL017"}]->(tl7)
CREATE (tl7)-[:APPROACH_TO {length: 180.3, lane: 3, id: "REL018"}]->(tl6)

CREATE (tl7)-[:APPROACH_TO {length: 165.8, lane: 2, id: "REL019"}]->(tl8)
CREATE (tl8)-[:APPROACH_TO {length: 165.8, lane: 2, id: "REL020"}]->(tl7)

CREATE (tl7)-[:APPROACH_TO {length: 155.4, lane: 1, id: "REL021"}]->(tl9)
CREATE (tl9)-[:APPROACH_TO {length: 155.4, lane: 1, id: "REL022"}]->(tl7)

CREATE (tl6)-[:APPROACH_TO {length: 190.6, lane: 1, id: "REL023"}]->(tl8)
CREATE (tl8)-[:APPROACH_TO {length: 190.6, lane: 1, id: "REL024"}]->(tl6)