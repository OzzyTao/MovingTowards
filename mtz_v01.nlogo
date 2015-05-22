__includes["./gsn_mtz.nls" "./env_mtz.nls" "./btp_mtz.nls" "./geometry_mtz.nls" "./tabular_mtz.nls"]
;; Define a new breed of turtle called motes (i.e. the (static) sensor nodes)
breed [motes mote]
motes-own [m zr history neighbourhood towards-neighbour similar-neighbour tree-parent tree-depth]
;; history is a list of tables that each one keep records about one specific object
;; Define a new breed of turtle called motes (i.e. moving objects)
breed [objects object]
objects-own [predis]
;; anchor vertices for target zone
breed [tzonevertices tzonevertex]

breed [gridpoints gridpoint]
undirected-link-breed [gridlines gridline]

breed [motegridpoints motegridpoint]
undirected-link-breed [motegridlines motegridline]
;; bounding box is represented as list of cor: [top left bottom right]
globals [targetzone-boundingbox motegridanchor-list global-history filename testresultline movement-seed
         interior-num boundary-num move-step max-tree-depth ct-event ct-dgt ct-cgt maincomponent predicate-list groundtruth-list]
;; System setup and initialization
to initialize
  ;; set target region
  let anchornum 5
  ;; set a size of square in which the z-zone can be located
  let boxsize world-width / 7
  let box-top random-box-top boxsize
  let box-right random-box-right boxsize
  create-ordered-tzonevertices anchornum [
    let x-offset random boxsize
    let y-offset random boxsize
    setxy (box-right - x-offset) (box-top - y-offset) 
    set size 2
    set color red
    ] 
  ask patches with [in-convex-hull] [
    set pcolor red
    ]
  ;; store the info of z-zone in a global variable
  set-target-bounding-box
  
  create-grid targetzone-boundingbox
  create-mote-grid
  
  ;; network structure
  if NetworkStructure = "UDG" [create-udg]
  if NetworkStructure = "GG" [create-udg create-gg]
  if NetworkStructure = "RNG" [create-udg create-rng]
  
  set-largest-component
    
  set maincomponent motes with [componentID = maincomponentID]
  ask maincomponent [ become "INIT"]
  ask one-of maincomponent [ become "INIZ" ]
  
  ask motes [ 
    set m [] 
    set history []
    set zr []
    set neighbourhood []
    set towards-neighbour []
    set similar-neighbour []
    set node-degree count comlink-neighbors
    ]
  
  ask objects [
    set predis 0
    ]
  
  set testresultline []
  set global-history []
  set movement-seed current-seed
  set interior-num 0
  set boundary-num 0
  
  set filename "../mtz-tests/flooding-cdc.csv"
  file-close-all
  if output-to-file [
  if file-exists? filename [file-delete filename]
  file-open filename
  file-print "Object, ticks, decentralized, centralized"
  ]
  print "Object, ticks, decentralized, centralized"
  
  set ct-event 0
  set ct-dgt -1
  set ct-cgt -1
  set predicate-list []
  set groundtruth-list []
  reset-ticks
end

;; Run the algorithm
to go
  clear-ctmsgs
  set ct-event 0
  set ct-dgt -1
  set ct-cgt -1
  set predicate-list []
  set groundtruth-list []
  ask maincomponent [step]
  if remainder ticks CMR = 0 [ 
    move-objects
    mote-labels
    object-tails
    set move-step move-step + 1
  ]
  tick
end

;;
;; Mote protocols
;;

;; Step through the current state
to step
  if state = "INIZ" [step_INIZ stop]
  if state = "INIT" [step_INIT stop]
  if state = "IDLE" [step_IDLE stop]
  if state = "XPNB" [step_XPNB stop]
  if state = "WTNB" [step_WTNB stop]
  if state = "INER" [step_INER stop]
  if state = "BNDY" [step_BNDY stop]
  if state = "INIT_DB" [step_INIT_DB stop]
  if state = "WAIT_DB" [step_WAIT_DB stop]
  if state = "IDLE_DB" [step_IDLE_DB stop]
  if state = "IDLE_NB" [step_IDLE_NB stop]
  if state = "ROOT_TE" [step_ROOT_TE stop]
  if state = "IDLE_TE" [step_IDLE_TE stop]
  if state = "DONE_TE" or state = "ROOT" [step_DONE_TE stop]
  if state = "IDLE_CT" [step_IDLE_CT stop]
end

;; propogate information about zone of interest
to step_INIZ
  broadcast (list "ZBOX" targetzone-boundingbox)
  if communicationstrategy = "Hybrid" [
    become "XPNB"
  ]
  if communicationstrategy = "Flooding" [
    become "IDLE"
    ]
  if communicationstrategy = "Direction-based" or communicationstrategy = "CDC-similarity" or communicationstrategy = "CDC-towards" [
    become "INIT_DB"
    ]
  if communicationstrategy = "Neighbourhood-based" [
    become "IDLE_NB"
    ]
  if communicationstrategy = "Shortest-path-tree" [
    become "ROOT_TE"
    ]
end

to step_INIT
  if has-message "ZBOX" [
    let msg received "ZBOX"
    let zbox item 1 msg
    set zr CDC-dir bounding-box zbox
    broadcast (list "ZBOX" zbox)
    if communicationstrategy = "Hybrid" [
      become "XPNB"
      ]
    if communicationstrategy = "Flooding" [
      become "IDLE"
      ]
    if communicationstrategy = "Direction-based" or communicationstrategy = "CDC-similarity" or communicationstrategy = "CDC-towards" [
      become "INIT_DB"
      ]
    if communicationstrategy = "Neighbourhood-based" [
      become "IDLE_NB"
      ]
    if communicationstrategy = "Shortest-path-tree" [
      become "IDLE_TE"
      ]
    ]
end

;; Hybrid method
;; neighbourhood exploration
to step_XPNB
  broadcast (list "RANGE" who xcor ycor)
  become "WTNB"
end

to step_WTNB
  if has-message "RANGE" [
    let msg received "RANGE"
    let record but-first msg
    set neighbourhood lput record neighbourhood
    ]
  if length neighbourhood = count comlink-neighbors [
    ifelse is-surrounded [
      set interior-num interior-num + 1
      become "INER"
      ][
      set boundary-num boundary-num + 1
      become "BNDY"
      ]
    ]
end

to step_INER 
  ;; on receiving an entering message from neighbour
  if has-message "OETR" [
    let msg received "OETR"
    let record but-first msg
    if not is-old record [
      update-local-history record TRUE
    ]
    ]
  
  if has-message "FLOD" [
    let msg received "FLOD"
    let record but-first msg
    if not is-old record [
      update-local-history record TRUE
    ]
    ]
  ;; on sensing an entering event
  let msgs on-sensing-movement TRUE
  foreach msgs [
    broadcast fput "OETR" ?
    ]
end

to step_BNDY
  if has-message "OETR" [
    let msg received "OETR"
    let record but-first msg
    if not is-old record [
      update-local-history record TRUE
    ]
    ]
  
  if has-message "FLOD" [
    let msg received "FLOD"
    let record but-first msg
    if not is-old record [
      update-local-history record TRUE
      broadcast msg
      ]
    ]
  ;; on sensing an entering event
  let msgs on-sensing-movement TRUE
  foreach msgs [
    broadcast fput "FLOD" ?
    ]
end

;; flooding method
to step_IDLE
  ;; when receieve a message AEXT
  if has-message "AEXT" [
    let msg received "AEXT"
    let steps last msg
    let record but-last (but-first msg)
    if not is-old record [
      update-local-history record TRUE
      if (searching-steps != 0 and steps < searching-steps) or searching-steps = 0 
      [ broadcast lput (steps + 1) (fput "AEXT" record) ]
    ]
  ]  
  let msgs on-sensing-movement TRUE
  foreach msgs [
    broadcast lput 1 (fput "AEXT" ?)
    ]
end


;; direction-based method
to step_INIT_DB
  broadcast (list "RANGE" who xcor ycor bounding-box zr)
  become "WAIT_DB"
end

to step_WAIT_DB
  if has-message "RANGE" [
    let msg received "RANGE"
    let record but-first msg
    set neighbourhood lput record neighbourhood
    ]
  if length neighbourhood = count comlink-neighbors [
    ifelse communicationstrategy = "CDC-similarity" [
      rank-neighbour-cdc-similarity
      set-towards-neighbours
      if length neighbourhood != length towards-neighbour [
        set-similar-neighbours-cdc 2
        ]
      ]
    [
      rank-neighbour-dir
      set-towards-neighbours
      if length neighbourhood != length towards-neighbour [
        set-similar-neighbours
      ]
    ]
    ifelse communicationstrategy = "CDC-towards" [
      become "IDLE_CT"
      ]
    [
      become "IDLE_DB"
    ]
    ]
end


to step_IDLE_DB
  if has-message "AEXT" [
    let msg received "AEXT"
    let targets last msg
    let towards-targets last but-last msg
    let record but-first but-last but-last msg
    if not is-old record [
      update-local-history record TRUE
      if member? who targets or empty? targets [
        set msg lput (map [first ?] towards-neighbour) (fput "AEXT" record)
        ifelse member? who towards-targets [
          multicast msg []
          ]
        [
          ifelse empty? towards-neighbour [
            multicast msg (map [first ?] similar-neighbour)
            ] 
          [
            multicast msg (map [first ?] towards-neighbour)
            ]
          ]
        ask patch-here [set pcolor black]
        ]
      ]
    ]

  let msgs on-sensing-movement TRUE
  foreach msgs [
    let tmpmsg ?
    let msg lput (map [first ?] towards-neighbour) (fput "AEXT" tmpmsg)
    ifelse empty? towards-neighbour [
        multicast msg (map [first ?] similar-neighbour)
        ] 
      [
        multicast msg (map [first ?] towards-neighbour)
        ]
      ]
end

;; if targets is an empty list, the msg will be broadcasted
to multicast [msg targets]
  broadcast (lput targets msg)
end

;; redirection based on whether if it is actually moving towards
to step_IDLE_CT
  if has-message "AEXT" [
    let msg received "AEXT"
    let targets last msg
    let is-target (member? who targets or empty? targets)
    let record but-first but-last msg
    if is-target and (not is-old record) [
      ask patch-here [set pcolor black]
      update-local-history record TRUE
      ifelse moving-towards record-bbox record bounding-box targetzone-boundingbox [
        multicast fput "AEXT" record []
        ]
      [
        ifelse empty? towards-neighbour [
          multicast fput "AEXT" record (map [first ?] similar-neighbour)
          ] [
          multicast fput "AEXT" record (map [first ?] towards-neighbour)
          ]
        ]
      ]
    ]
  let msgs on-sensing-movement TRUE
  foreach msgs [
    multicast (fput "AEXT" ?) []
    ]
end

;; direct-neighbour-based method
to step_IDLE_NB
  ;; when receieve a message AEXT
  if has-message "AEXT" [
    let msg received "AEXT"
    let record but-first msg
    update-local-history record False
    ]
  
  let msgs on-sensing-movement TRUE
  foreach msgs [
    broadcast fput "AEXT" ?
    ]
end 


;; centralized method : shortest path tree
to step_ROOT_TE 
  ask comlinks [ hide-link ]
  set tree-parent -1
  set tree-depth 0
  broadcast (list "TREE" who tree-depth)
  become "ROOT"
end

to step_IDLE_TE
  if has-message "TREE" [
    let msg received "TREE"
    set tree-parent item 1 msg
    ask comlink who tree-parent [show-link]
    set tree-depth item 2 msg + 1
    broadcast (list "TREE" who tree-depth)
    become "DONE_TE"
    ]
end

to step_DONE_TE
  if has-message "TREE" [
    let msg received "TREE"
    if item 2 msg + 1 < tree-depth [
      ask comlink who tree-parent [hide-link]
      set tree-parent item 1 msg
      ask comlink who tree-parent [show-link]
      set tree-depth item 2 msg + 1
      broadcast (list "TREE" who tree-depth)
      ]
    ]
  
  if has-message "AEXT" [
    let msg received "AEXT"
    ifelse tree-parent = -1 [
      let record but-first msg
      if ground-truth-check [update-global-history record]
      decide-on-history record
      ]
    [
      send msg mote tree-parent
      ] 
    ]
  
  ;; on sensing entering event
  ifelse tree-parent = -1 [
    let msgs on-sensing-movement TRUE
    ]
  [
    let sensor self
    let in-range-objects objects with [within-sensing-range sensor]
    ;;ifelse count in-range-objects > 0 [
    ;;  highlight-sensing-range
    ;;  ]
    ;;[
    ;;  clear-sensing-range
    ;;  ]
    foreach sort in-range-objects [
      if which-active-record [who] of ? = -1 [
        let temprecord (list "NULL" 0 "NULL")
        set temprecord replace-item 0 temprecord [who] of ?
        set temprecord replace-item 1 temprecord ticks
        set m lput temprecord m
        let msg (list first temprecord who bounding-box item 1 temprecord)
        send (fput "AEXT" msg) mote tree-parent
        ]
      ]
    close-inactive-records
    ]
end


to centralized-cdc-validation [obj-id]
  let location history-location global-history obj-id
  if location >= 0 [
    let its-history item location global-history
    if length its-history > 1 [
      let currentBBOX record-bbox (last its-history)
      let previous-record proper-previous-record its-history record-timestamp (last its-history)
      if not empty? previous-record [
      let previousBBOX record-bbox previous-record
      let predir CDC-dir previousBBOX currentBBOX
      let curdir CDC-dir currentBBOX targetzone-boundingbox
      ifelse not empty? (filter [member? ? predir] curdir) [log-gt TRUE] [log-gt FALSE]
      ]
    ]
  ]
end

;; on sensing entering events, report messages to be spreaded 
to-report on-sensing-movement [keep-history]
  ;; on sensing an entering event
  let msgs []
  let sensor self
  let in-range-objects objects with [within-sensing-range sensor]
  
  ;;visual effect
  ;;ifelse count in-range-objects > 0 [
  ;;  highlight-sensing-range
  ;;  ]
  ;;[
  ;;  clear-sensing-range
  ;;  ]
  
  foreach sort in-range-objects [
    if which-active-record [who] of ? = -1 [
      let temprecord [ "NULL" 0 "NULL"]
      set temprecord replace-item 0 temprecord [who] of ?
      set temprecord replace-item 1 temprecord ticks
      set m lput temprecord m
      let msg (list item 0 temprecord who bounding-box item 1 temprecord)
      update-local-history msg keep-history
      
      set testresultline []
      set testresultline lput "," (lput item 0 temprecord testresultline)
      set testresultline lput "," (lput item 1 temprecord testresultline)
      
      let hindex locate-local-record first temprecord
      if hindex >= 0 [
        let its-history item hindex history
        let previous proper-previous-record its-history ticks
        if not empty? previous [
          let predir CDC-dir (item 2 previous) bounding-box
          ifelse not empty? (filter [member? ? zr] predir) [log-predicate TRUE] [log-predicate FALSE]
        ]
      ] 
      set msgs lput msg msgs   
      if ground-truth-check [
        update-global-history msg
        centralized-cdc-validation first msg
      ]
      log-results testresultline
      count-event
    ]
  ]
  close-inactive-records
  report msgs
end

to decide-on-history [record]
  let obj-id first record
  update-local-history record TRUE
  let location locate-local-record obj-id
  if location >= 0 [
    set testresultline (list obj-id "," (last record) ",")
    let its-history item location history
    if length its-history > 1 [
      let currentBBOX record-bbox (last its-history)
      let previous-record proper-previous-record its-history record-timestamp (last its-history)
      if not empty? previous-record [
        let previousBBOX record-bbox previous-record
        ifelse moving-towards previousBBOX currentBBOX targetzone-boundingbox [log-predicate TRUE] [log-predicate FALSE]
      ]
      if ground-truth-check [
      centralized-cdc-validation obj-id
      ]
      log-results testresultline
      count-event
    ]
  ]
end

to-report moving-towards [azone bzone zzone]
  let predir CDC-dir azone bzone
  let curdir CDC-dir bzone zzone
  ifelse not empty? (filter [member? ? predir] curdir) [
    report TRUE
    ]
  [
    report FALSE
    ]
end

;; Move object (based on modified correlated random walk)
to move-objects
  if move-type = "Simple Linear" [
    ask objects [
      fd 1
    ]
  ]
  if move-type = "CRW" [
    ask objects [
      random-seed movement-seed
      rt random-normal 0 15 ;; Change the heading based on Gaussian distribution with standard deviation of 15 degrees
      set movement-seed next-seed
      fd 1
    ]
  ]
end

;; Assign labels to motes based on the MoteLable dropdown list
to mote-labels
  ask motes [
    if MoteLabel = "none" [set label ""] ;; Hide the label
    if MoteLabel = "mote id" [set label who] ;; Show mote id
    if MoteLabel = "m" [set label m] ;; Show contents of m
    if MoteLabel = "zr" [set label zr] ;; Show contents of m
  ]
end

;; Draw tail of objects' movement trajectory (width as assigned)
to object-tails
  ask objects [
    ifelse show-tails 
    [ask objects [pen-down
                set pen-size tail-width]]
    [ask objects [pen-up]]
  ]
end



to highlight-sensing-range
  ;ask patches in-radius s [
   ; set pcolor yellow
    ;]
  ;set size s + world-width / 21
  set shape "active_sensor"
  adjust-mote-grid bounding-box
end

to clear-sensing-range
  ;ask patches in-radius s [
   ; set pcolor white
    ;]
  ;set size world-width / 21
  become state
end

;; report position of the open record with id of obj-id in the histroy table
to-report which-active-record [obj-id]
  let temp -1
  foreach filter [ item 2 ? = "NULL" ] m [
    if item 0 ? = obj-id [ set temp position ? m ]
    ]
  report temp
end

to close-inactive-records
  foreach filter [ item 2 ? = "NULL" ] m [
    let tempobj object item 0 ?
    if distance tempobj > s [
      let temprecord replace-item 2 ? ticks
      set m replace-item position ? m m temprecord
      ]
    ]
end




to display-history
  clear-output
  output-print "object-ID   mote-ID  entering-TIME"
  foreach history [
    foreach ? [
      output-type first ?
      output-type "          "
      output-type item 1 ?
      output-type "          "
      output-print last ?
    ]
    output-print " "
    ]
end


to report-true-dir
  let currentdis point-region-distance (list xcor ycor) targetzone-boundingbox
  if currentdis < predis [
    type "Object "
    type self
    type " "
    print "True moving towards"
    ]
  set predis currentdis
end

to create-grid [bbox]
  let topcor item 0 bbox
  let leftcor item 1 bbox
  let bottomcor item 2 bbox
  let rightcor item 3 bbox
  let previous 0
  create-gridpoints 1 [
    setxy min-pxcor topcor
    set previous self]
  create-gridpoints 1 [
    setxy max-pxcor topcor
    create-gridline-with previous
    ]
  create-gridpoints 1 [
    setxy min-pxcor bottomcor
    set previous self
    ]
  create-gridpoints 1 [
    setxy max-pxcor bottomcor
    create-gridline-with previous
    ]
  create-gridpoints 1 [
    setxy leftcor min-pycor
    set previous self
    ]
  create-gridpoints 1 [
    setxy leftcor max-pycor
    create-gridline-with previous
    ]
  create-gridpoints 1 [
    setxy rightcor min-pycor
    set previous self
    ]
  create-gridpoints 1 [
    setxy rightcor max-pycor
    create-gridline-with previous
    ]
  ask gridlines [
    set shape "gridline"
    set color red
    ]
end

to create-mote-grid
  let previous 0
  let xmintop 0
  let xminbottom 0
  let leftymin 0
  let rightymin 0
  let xmaxbottom 0
  let xmaxtop 0
  let rightymax 0
  let leftymax 0
  create-motegridpoints 1 [
    setxy min-pxcor min-pycor
    set previous self
    set xmintop self
    ]
  create-motegridpoints 1 [
    setxy max-pxcor min-pycor
    create-motegridline-with previous
    set xmaxtop self
    ]
  create-motegridpoints 1 [
    setxy min-pxcor min-pycor
    set previous self
    set xminbottom self
    ]
  create-motegridpoints 1 [
    setxy max-pxcor min-pycor
    create-motegridline-with previous
    set xmaxbottom self
    ]
  create-motegridpoints 1 [
    setxy min-pxcor min-pycor
    set previous self
    set leftymin self
    ]
  create-motegridpoints 1 [
    setxy min-pxcor max-pycor
    create-motegridline-with previous
    set leftymax self
    ]
  create-motegridpoints 1 [
    setxy max-pxcor min-pycor
    set previous self
    set rightymin self
    ]
  create-motegridpoints 1 [
    setxy max-pxcor max-pycor
    create-motegridline-with previous
    set rightymax self
    ] 
  ask motegridlines [
    set shape "gridline"
    set color green
    ]
  set motegridanchor-list (list xmintop xminbottom leftymin rightymin xmaxbottom xmaxtop rightymax leftymax) 
end

to adjust-mote-grid [bbox]
  let topcor item 0 bbox
  if topcor > max-pycor [set topcor max-pycor]
  let leftcor item 1 bbox
  if leftcor < min-pxcor [set leftcor max-pxcor]
  let bottomcor item 2 bbox
  if bottomcor < min-pycor [set bottomcor min-pycor]
  let rightcor item 3 bbox
  if rightcor > max-pxcor [set rightcor max-pxcor]
  ask item 0 motegridanchor-list [setxy min-pxcor topcor]
  ask item 1 motegridanchor-list [setxy min-pxcor bottomcor]
  ask item 2 motegridanchor-list [setxy leftcor min-pycor]
  ask item 3 motegridanchor-list [setxy rightcor min-pycor]
  ask item 4 motegridanchor-list [setxy max-pxcor bottomcor]
  ask item 5 motegridanchor-list [setxy max-pxcor topcor]
  ask item 6 motegridanchor-list [setxy rightcor max-pycor]
  ask item 7 motegridanchor-list [setxy leftcor max-pycor]
end

to count-event
  if componentID = maincomponentID [
    set ct-event ct-event + 1
    ]
end

to log-gt [predicate]
  if ct-cgt < 0 [set ct-cgt 0]
  ifelse predicate [
    set testresultline lput TRUE testresultline
    set ct-cgt ct-cgt + 1
    set groundtruth-list lput 1 groundtruth-list
    ]
  [
    set testresultline lput FALSE testresultline
    set groundtruth-list lput 0 groundtruth-list
    ]
end

to log-predicate [predicate]
  if ct-dgt < 0 [set ct-dgt 0]
  ifelse predicate [
    set testresultline lput "," (lput TRUE testresultline)
    set ct-dgt ct-dgt + 1
    set predicate-list lput 1 predicate-list
    ] [
    set testresultline lput "," (lput FALSE testresultline)
    set predicate-list lput 0 predicate-list
    ]
end
@#$#@#$#@
GRAPHICS-WINDOW
355
20
1153
839
100
100
3.9204
1
12
1
1
1
0
0
0
1
-100
100
-100
100
1
1
1
ticks
30.0

INPUTBOX
10
45
60
105
Netsize
1000
1
0
Number

BUTTON
10
10
75
43
Setup
setup\ninitialize
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

BUTTON
75
10
140
43
Go!
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

SWITCH
10
110
210
143
trackmsg
trackmsg
0
1
-1000

INPUTBOX
110
45
160
105
c
20
1
0
Number

INPUTBOX
160
45
210
105
s
5
1
0
Number

BUTTON
140
10
202
43
Go-1
go
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
1

INPUTBOX
60
45
110
105
ObjNo
1
1
0
Number

SWITCH
10
145
210
178
show-links
show-links
1
1
-1000

SWITCH
10
180
210
213
show-tails
show-tails
0
1
-1000

INPUTBOX
130
220
210
280
tail-width
3
1
0
Number

CHOOSER
10
220
110
265
MoteLabel
MoteLabel
"none" "mote id" "m" "zr"
0

OUTPUT
15
425
305
605
21

MONITOR
15
300
97
345
sent length
sent-length-msg-totals
17
1
11

MONITOR
110
300
202
345
sent number
sent-number-msg-totals
17
1
11

MONITOR
15
355
97
400
recv length
recv-length-msg-totals
17
1
11

MONITOR
110
355
202
400
recv number
recv-number-msg-totals
17
1
11

CHOOSER
15
650
155
695
Seed
Seed
"none" "random" "manual"
2

INPUTBOX
15
695
155
755
current-seed
-640111348
1
0
Number

CHOOSER
15
765
155
810
move-type
move-type
"Simple Linear" "CRW"
1

SWITCH
15
610
157
643
output-to-file
output-to-file
1
1
-1000

CHOOSER
175
615
313
660
NetworkStructure
NetworkStructure
"UDG" "GG" "RNG"
0

CHOOSER
175
670
330
715
CommunicationStrategy
CommunicationStrategy
"Flooding" "Hybrid" "Direction-based" "CDC-similarity" "Neighbourhood-based" "Shortest-path-tree" "CDC-towards"
5

MONITOR
215
300
312
345
Interior nodes
interior-num
17
1
11

MONITOR
215
360
327
405
Boundary nodes
boundary-num
17
1
11

INPUTBOX
170
730
322
790
searching-steps
0
1
0
Number

INPUTBOX
210
45
260
105
CMR
100
1
0
Number

SWITCH
170
795
352
828
ground-truth-check
ground-truth-check
0
1
-1000

@#$#@#$#@
## PROTOCOL

Monitoring movement towards a zone of interest using qualitative directional information

## SUMMARY



## OPERATION



## NOTICE


## TRY

## CREDITS

Builds on and extends code from Duckham, 2013

Copyright 2011, 2012 Matt Duckham

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License http://www.gnu.org/licenses/ for more details.
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

active_sensor
true
0
Circle -13840069 true false 2 2 297

fish
true
0
Polygon -7500403 true true 150 60 120 90 105 150 135 210 105 240 195 240 165 210 195 150 180 90
Polygon -16777216 false false 150 60 120 90 105 150 135 210 105 240 195 240 165 210 195 150 180 90

mote
true
15
Circle -16777216 true false 90 90 120
Circle -1 true true 105 105 90

mote_communicate
true
0
Circle -16777216 true false 90 90 120
Circle -7500403 true true 105 105 90
Polygon -7500403 true true 239 94 254 105 265 120 273 150 266 179 254 194 240 204 251 219 267 205 280 188 290 164 289 135 279 110 265 92 250 80
Polygon -7500403 true true 61 94 46 105 35 120 27 150 34 179 46 194 60 204 49 219 33 205 20 188 10 164 11 135 21 110 35 92 50 80
Polygon -7500403 true true 224 105 236 119 243 137 245 158 240 175 229 191 218 183 227 169 230 156 229 142 224 126 215 113
Polygon -7500403 true true 76 105 64 119 57 137 55 158 60 175 71 191 82 183 73 169 70 156 71 142 76 126 85 113

sensor
true
0
Circle -7500403 false true 0 0 300

@#$#@#$#@
NetLogo 5.1.0
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
<experiments>
  <experiment name="e1" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
      <value value="&quot;Hybird&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e2" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
      <value value="2041885769"/>
      <value value="808731895"/>
      <value value="-1015969799"/>
      <value value="1991405964"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="250"/>
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
      <value value="1250"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
      <value value="&quot;Hybird&quot;"/>
      <value value="&quot;Direction-based&quot;"/>
      <value value="&quot;Neighbourhood-based&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e3" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
      <value value="&quot;Hybird&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e4" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
      <value value="&quot;Hybird&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e5" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
      <value value="&quot;Hybird&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="0"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e8" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
      <value value="&quot;Hybrid&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e9" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e10" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e11" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="500"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Shortest-path-tree&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e12" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="50"/>
      <value value="250"/>
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
      <value value="1250"/>
      <value value="1500"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
      <value value="&quot;Hybrid&quot;"/>
      <value value="&quot;Direction-based&quot;"/>
      <value value="&quot;CDC-similarity&quot;"/>
      <value value="&quot;Neighbourhood-based&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;CDC-towards&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e13" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-1338276584"/>
      <value value="1310021339"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="50"/>
      <value value="250"/>
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
      <value value="1250"/>
      <value value="1500"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
      <value value="&quot;Hybrid&quot;"/>
      <value value="&quot;Direction-based&quot;"/>
      <value value="&quot;CDC-similarity&quot;"/>
      <value value="&quot;Neighbourhood-based&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;CDC-towards&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e14" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
      <value value="-1338276584"/>
      <value value="1310021339"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="50"/>
      <value value="250"/>
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
      <value value="1250"/>
      <value value="1500"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="3"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e15" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
      <value value="-1338276584"/>
      <value value="1310021339"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="50"/>
      <value value="250"/>
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
      <value value="1250"/>
      <value value="1500"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Shortest-path-tree&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="3"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e16" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="100"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
      <value value="&quot;Hybrid&quot;"/>
      <value value="&quot;Direction-based&quot;"/>
      <value value="&quot;CDC-similarity&quot;"/>
      <value value="&quot;Neighbourhood-based&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;CDC-towards&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e17" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
      <value value="&quot;Hybrid&quot;"/>
      <value value="&quot;Direction-based&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e18" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;CDC-similarity&quot;"/>
      <value value="&quot;Neighbourhood-based&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;CDC-towards&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e19" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Shortest-path-tree&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e20" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "BCST"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TOZZ"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="500"/>
      <value value="750"/>
      <value value="1000"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
      <value value="&quot;GG&quot;"/>
      <value value="&quot;RNG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="0"/>
      <value value="1"/>
      <value value="2"/>
      <value value="3"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e21" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="750"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
      <value value="&quot;GG&quot;"/>
      <value value="&quot;RNG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
      <value value="&quot;Hybrid&quot;"/>
      <value value="&quot;Direction-based&quot;"/>
      <value value="&quot;CDC-similarity&quot;"/>
      <value value="&quot;Neighbourhood-based&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;CDC-towards&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e22" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="20"/>
      <value value="40"/>
      <value value="60"/>
      <value value="80"/>
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ObjNo">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="20"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="s">
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
      <value value="&quot;Hybrid&quot;"/>
      <value value="&quot;Direction-based&quot;"/>
      <value value="&quot;CDC-similarity&quot;"/>
      <value value="&quot;Neighbourhood-based&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;CDC-towards&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="move-type">
      <value value="&quot;CRW&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="trackmsg">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="output-to-file">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="searching-steps">
      <value value="0"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
</experiments>
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180

gridline
0.0
-0.2 0 0.0 1.0
0.0 1 2.0 2.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180

@#$#@#$#@
1
@#$#@#$#@
