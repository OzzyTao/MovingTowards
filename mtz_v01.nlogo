__includes["./gsn_mtz.nls" "./env_mtz.nls" "./btp_mtz.nls" "./geometry_mtz.nls" "./tabular_mtz.nls" "./move_mtz.nls" "./group_mtz.nls" "./mtz_apperance.nls"]
;; Define a new breed of turtle called motes (i.e. the (static) sensor nodes)
breed [motes mote]
motes-own [m zr history neighbourhood towards-neighbour similar-neighbour tree-parent tree-depth pedding-msgs group-position-history num-msgsent closest-neighbour root]
;; history is a list of tables that each one keep records about one specific object
;; Define a new breed of turtle called motes (i.e. moving objects)
breed [objects object]
objects-own [predis flockmates nearest-neighbor]
;; anchor vertices for target zone
breed [tzonevertices tzonevertex]

breed [gridpoints gridpoint]
undirected-link-breed [gridlines gridline]

breed [motegridpoints motegridpoint]
undirected-link-breed [motegridlines motegridline]
;; bounding box is represented as list of cor: [top left bottom right]
globals [targetzone-boundingbox motegridanchor-list global-history filename testresultline movement-seed total-events
         interior-num boundary-num move-step max-tree-depth ct-event ct-dgt ct-cgt maincomponent predicate-list groundtruth-list diameter init-msg-ct moniter-msg-ct output-cache]
;; System setup and initialization
to initialize
  set setup-msg-headers (list "ZBOX" "RANGE" "TREE" "RTPS")
  set moniter-msg-headers (list "AEXT" "OETR" "FLOD" "GRDY" "FACE")
  
  setup-zzone
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
  if calc-diameter [set diameter network-diameter maincomponent]
  ask maincomponent [ become "INIT"]
  ask one-of maincomponent [ become "INIZ" ]
  
  ask motes [ 
    set num-msgsent 0
    set pedding-msgs []
    set m [] 
    set history []
    set zr []
    set neighbourhood []
    set towards-neighbour []
    set similar-neighbour []
    set node-degree count comlink-neighbors
    ]
  
  setup-objects
  setup-output
  
  ;; setup global variables
  set testresultline []
  set global-history []
  set movement-seed current-seed
  set interior-num 0
  set boundary-num 0
  
  set total-events 0  
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
  ask maincomponent [step]
  if remainder ticks CMR = 0 [ 
    output-logline
    if visual-aids [ask motes [clear-sensing-range]]
    move-objects
    mote-labels
    object-tails
    set move-step move-step + 1
  ]
  if sensorfailure [
  if remainder ticks CMR = (CMR - 1) [print-log]
  if ticks = fail-ticks [kill-random-sensor]
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
  ;;if state = "IDLE_GROUP" [step_fixed_group_IDLE stop]
  
  if communicationstrategy = "Hybrid" [protocal_step_hybrid]
  if communicationstrategy = "Flooding" [protocal_step_flooding]
  if communicationstrategy = "Direction-based" [protocal_step_direction_based]
  if communicationstrategy = "CDC-similarity" [protocal_step_CDC_similarity]
  if communicationstrategy = "CDC-towards" [protocal_step_CDC_towards]
  if communicationstrategy = "GPSR" [protocal_step_GPSR]
  if communicationstrategy = "Shortest-path-tree" [protocal_step_tree]
end

;; propogate information about zone of interest
to step_INIZ
  broadcast (list "ZBOX" targetzone-boundingbox)
  finish_zbox_initialization TRUE
end

to step_INIT
  if has-message "ZBOX" [
    let msg received "ZBOX"
    let zbox item 1 msg
    set zr CDC-dir bounding-box zbox
    broadcast (list "ZBOX" zbox)
    finish_zbox_initialization FALSE
    ]
end


to finish_zbox_initialization [leader]
  ;;if objNo > 1 [become "IDLE_GROUP" stop]
  if communicationstrategy = "Hybrid" [become "XPNB" stop]
  if communicationstrategy = "Flooding" [become "IDLE" stop]
  if communicationstrategy = "Direction-based" [become "XPNB" stop]
  if communicationstrategy = "CDC-similarity" [become "XPNB" stop]
  if communicationstrategy = "CDC-towards" [become "XPNB" stop]
  if communicationstrategy = "Shortest-path-tree" [ifelse leader [become "ROOT_TE"] [become "IDLE_TE"] stop]  
  if communicationstrategy = "GPSR" [ifelse leader [become "ROOT_GR"] [become "WTRT"] stop]
end


;;;;;;; flooding algorithm start;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;states ("IDLE"), msgs ("AEXT");;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
to protocal_step_flooding
  if state = "IDLE" [step_IDLE stop]
end

to step_IDLE
  ;; when receieve a message AEXT
  if has-message "AEXT" [
    let msg but-first received "AEXT"
    let record protocal_hopcount_decode msg
    if not is-old record [
      if visual-aids [ask patch-here [set pcolor black]]
      update-local-history record TRUE
      let nextmsg (protocal_hopcount_nextmsg msg [])
      if not empty? nextmsg [broadcast fput "AEXT" nextmsg]
      ]
  ] 
  ;; when sensing entering events
  if ticks > 1000 [  
  let msgs on-sensing-movement TRUE
  foreach msgs [ broadcast fput "AEXT" (protocal_hopcount_encode ?)]
  ]
end
;;;;;;;;;;;;flooding algorithm end;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;; hybrid algorithm start;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;setup states ("XPNB","WTNB"), monitering states ("INER","BNDY"), setup msg ("RANGE"), monitering msg ("OETR","FLOD")
to protocal_step_hybrid
  if state = "XPNB" [step_XPNB (list who xcor ycor) stop]
  if state = "WTNB" [step_WTNB "set-surrounded-state" stop]
  if state = "INER" [step_INER stop]
  if state = "BNDY" [step_BNDY stop]
end

to set-surrounded-state
  ifelse is-surrounded [
    set interior-num interior-num + 1
    become "INER"
  ][
    set boundary-num boundary-num + 1
    become "BNDY"
  ]
end

to step_INER 
  ;; on receiving an entering message from neighbour
  if has-message "OETR" [
    let record but-first received "OETR"
    if not is-old record [
      if visual-aids [ask patch-here [set pcolor black]]
      update-local-history record TRUE
      ]
    ]
  
  if has-message "FLOD" [
    let msg but-first received "FLOD"
    let record protocal_hopcount_decode msg
    if not is-old record [
      if visual-aids [ask patch-here [set pcolor black]]
      update-local-history record TRUE
      ]
    ]
  ;; on sensing an entering event
  if ticks > 1000 [
  let msgs on-sensing-movement TRUE
  foreach msgs [broadcast fput "OETR" ?]
  ]
end

to step_BNDY
  if has-message "OETR" [
    let record but-first received "OETR"
    if not is-old record [
      if visual-aids [ask patch-here [set pcolor black]]
      update-local-history record TRUE
      ]
    ]
  
  if has-message "FLOD" [
    let msg but-first received "FLOD"
    let record protocal_hopcount_decode msg
    if not is-old record [
      if visual-aids [ask patch-here [set pcolor black]]
      update-local-history record TRUE
      let nextmsg (protocal_hopcount_nextmsg msg [])
      if not empty? nextmsg [broadcast fput "FLOD" nextmsg]
      ]
    ]
  ;; on sensing an entering event
  if ticks > 1000 [
  let msgs on-sensing-movement TRUE
  foreach msgs [broadcast fput "FLOD" (protocal_hopcount_encode ?)]
  ]
end
;;;;;;;; hybrid algorithm end ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;direction based algorithm (cyclic order);;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;; init states ("XPNB","WTNB"), monitering states ("IDLE_DB"), init msg ("RANGE"), monitering msg ("AEXT")
to protocal_step_direction_based
  if state = "XPNB" [step_XPNB (list who xcor ycor bounding-box zr) stop]
  if state = "WTNB" [step_WTNB "setup-neighbour-info-db" stop]
  if state = "IDLE_DB" [step_IDLE_DB stop]
end

to setup-neighbour-info-db
  rank-neighbour-dir
  set-towards-neighbours
  if length neighbourhood != length towards-neighbour [set-similar-neighbours]
  become "IDLE_DB"
end

to step_IDLE_DB
  if has-message "AEXT" [
    let multicast-msg but-first received "AEXT"
    if protocal_multicast_istarget? multicast-msg who [
      let hopcount-msg protocal_multicast_payload multicast-msg
      let record protocal_hopcount_decode hopcount-msg
      let towards-targets last record
      set record but-last record
      if not is-old record [
        if visual-aids [ask patch-here [set pcolor black]]
        update-local-history record TRUE
        ask patch-here [set pcolor black]
        let msg lput (map [first ?] towards-neighbour) record
        set msg protocal_hopcount_nextmsg hopcount-msg msg
        if not empty? msg [
          set multicast-msg ifelse-value (member? who towards-targets) [protocal_multicast_pack msg [] ] 
            [ ifelse-value (empty? towards-neighbour) [protocal_multicast_pack msg (map [first ?] similar-neighbour)] 
              [protocal_multicast_pack msg (map [first ?] towards-neighbour)]]
          broadcast fput "AEXT" multicast-msg
        ]
      ]
    ]
  ]
  
  if ticks > 1000 [
  let msgs on-sensing-movement TRUE
  foreach msgs [
    let msg protocal_hopcount_encode (lput (map [first ?] towards-neighbour) ?)
    set msg ifelse-value empty? towards-neighbour [ protocal_multicast_pack msg (map [first ?] similar-neighbour) ] [ protocal_multicast_pack msg (map [first ?] towards-neighbour) ]
    broadcast fput "AEXT" msg
  ]
  ]
end
;;;;;;;;;;;;;;;;;;direction based algorithm (cyclic order);;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;direction based algorithm (CDC-similarity);;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;; init states ("XPNB","WTNB"), monitering states ("IDLE_DB"), init msg ("RANGE"), monitering msg ("AEXT")
to protocal_step_CDC_similarity
  if state = "XPNB" [step_XPNB (list who xcor ycor bounding-box zr) stop]
  if state = "WTNB" [step_WTNB "setup-neighbour-info-cs" stop]
  if state = "IDLE_DB" [step_IDLE_DB stop]
end
to setup-neighbour-info-cs
  rank-neighbour-cdc-similarity
  set-towards-neighbours
  if length neighbourhood != length towards-neighbour [set-similar-neighbours-cdc 2] 
  become "IDLE_DB" 
end
;;;;;;;;;;;;;;;;;;direction based algorithm (CDC-similarity);;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



;;;;;;;;;;;;;;;;;;direction based algorithm (CDC-moving-towards);;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;; init states ("XPNB","WTNB"), monitering states ("IDLE_CT"), init msg ("RANGE"), monitering msg ("AEXT")
to protocal_step_CDC_towards
  if state = "XPNB" [step_XPNB (list who xcor ycor bounding-box zr) stop]
  if state = "WTNB" [step_WTNB "setup-neighbour-info-ct" stop]
  if state = "IDLE_CT" [step_IDLE_CT stop]
end
to setup-neighbour-info-ct
  rank-neighbour-dir
  set-towards-neighbours
  if length neighbourhood != length towards-neighbour [set-similar-neighbours]  
  become "IDLE_CT"
end
;; redirection based on whether if it is actually moving towards
to step_IDLE_CT
  if has-message "AEXT" [
    let multicast-msg but-first received "AEXT"
    let hopcount-msg protocal_multicast_payload multicast-msg
    let record protocal_hopcount_decode hopcount-msg
    if (protocal_multicast_istarget? multicast-msg who) and (not is-old record) [
      if visual-aids [ask patch-here [set pcolor black]]
      update-local-history record TRUE
      set hopcount-msg protocal_hopcount_nextmsg hopcount-msg []
      if not empty? hopcount-msg [
        set multicast-msg protocal_multicast_pack hopcount-msg ifelse-value (moving-towards record-bbox record bounding-box targetzone-boundingbox) [ [] ] [
          ifelse-value (empty? towards-neighbour) [(map [first ?] similar-neighbour)] [(map [first ?] towards-neighbour)]
          ]
        broadcast fput "AEXT" multicast-msg
      ]
    ]
  ]
  
  if ticks > 1000 [
  let msgs on-sensing-movement TRUE
  foreach msgs [
    let msg protocal_multicast_pack (protocal_hopcount_encode ?) []
    broadcast fput "AEXT" msg
    ]
  ]
end
;;;;;;;;;;;;;;;;;;;;;;;;;;direction based algorithm (CDC-moving-towards);;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;shortest path tree algorithm;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;states ("ROOT_TE","IDLE_TE","ROOT","DONE_TE"),init msg ("TREE"), monitering msg ("AEXT");;;;;;;;;;;;;;;;;;;;;;;;;;;;
to protocal_step_tree
  if state = "ROOT_TE" [step_ROOT_TE stop]
  if state = "IDLE_TE" [step_IDLE_TE stop]
  if state = "ROOT" or state = "DONE_TE" [step_DONE_TE stop]
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
    if visual-aids [ask patch-here [set pcolor black]]
    ifelse tree-parent = -1 [
      let record but-first msg
      decide-on-history record
      ]
    [
      if mote tree-parent != nobody [send msg mote tree-parent]
      ] 
    ]
   
  ;; on sensing entering event
  if ticks > 1000 [
  ifelse tree-parent = -1 [
    let msgs on-sensing-movement TRUE
    ]
  [
    let sensor self
    let in-range-objects objects with [within-sensing-range sensor]
    ;;visual effect
    if visual-aids [ifelse count in-range-objects > 0 [highlight-sensing-range] [clear-sensing-range]]
    
    foreach sort in-range-objects [
      if which-active-record [who] of ? = -1 [
        let temprecord (list "NULL" 0 "NULL")
        set temprecord replace-item 0 temprecord [who] of ?
        set temprecord replace-item 1 temprecord ticks
        set m lput temprecord m
        let msg (list first temprecord who bounding-box item 1 temprecord)
        if ground-truth-check [
          update-global-history msg
          centralized-cdc-validation first msg
          ]
        count-event
        if mote tree-parent != nobody [send (fput "AEXT" msg) mote tree-parent]
        ]
      ]
    close-inactive-records
    ]
  ]
end
;;;;;;;;;;;;;;;;;;;;;;;;;;shortest path tree algorithm;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;Georouting (GPSR);;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;init states ("ROOT_GR","WTRT","XPNB","WTNB"), monitering state ("IDLE"), init msgs ("RTPS","RANGE"), monitering msgs ("GRDY","FACE")
to protocal_step_GPSR
  if state = "ROOT_GR" [step_ROOT_GR stop]
  if state = "WTRT" [step_WTRT stop]
  if state = "XPNB" [step_XPNB (list who xcor ycor) stop]
  if state = "WTNB" [step_WTNB "setup-neighbour-info-GR" stop]
  if state = "IDLE" [step_IDLE_GR stop]
end

to step_ROOT_GR
  set root (list who xcor ycor)
  broadcast (list "RTPS" who xcor ycor)
  become "XPNB"
end

to step_WTRT
  if has-message "RTPS" [
    set root but-first received "RTPS"
    broadcast fput "RTPS" root
    become "XPNB"
    ]
end

to setup-neighbour-info-GR
  set-gg-neighbour
  set-closest-neighbour
  rank-neighbour-dir
  become "IDLE"
end

to set-gg-neighbour
  let thisnode (list xcor ycor)
  let gg-neighbourhood []
  foreach neighbourhood [
    let add true
    let currentnode (list item 1 ? item 2 ?)
    let delete_ids []
    foreach gg-neighbourhood [
      let tmppoint (list item 1 ? item 2 ?)
      if ((euclidean-distance thisnode tmppoint) ^ 2) + ((euclidean-distance currentnode tmppoint) ^ 2) < ((euclidean-distance thisnode currentnode) ^ 2) [set add false]
      if ((euclidean-distance thisnode currentnode) ^ 2) + ((euclidean-distance tmppoint currentnode) ^ 2) < ((euclidean-distance thisnode tmppoint) ^ 2) [
        ;set gg-neighbourhood remove ? gg-neighbourhood
        set delete_ids lput first ? delete_ids
        ]
      ]
    set gg-neighbourhood filter [not member? first ? delete_ids] gg-neighbourhood
    if add [
      set gg-neighbourhood lput ? gg-neighbourhood
      ]
    ]
  foreach gg-neighbourhood [ask comlink first ? who [set color red]]
  set neighbourhood gg-neighbourhood
end

to redirect-GR [message root-distance prenode]
  let msg message
  if not empty? closest-neighbour [
  ifelse last closest-neighbour < root-distance [
    set msg protocal_sender_aware_pack msg who
    send (fput "GRDY" msg) mote first closest-neighbour
    ]
  [
    let tangle included-angle prenode
    let nextstop ccw-neighbour tangle
    if not empty? nextstop [
      set msg lput root-distance msg
      set msg protocal_sender_aware_pack msg who
      send (fput "FACE" msg) mote first nextstop
      ]
    ]
  ]
end

to-report root-check [record]
  ifelse who = first root [
    decide-on-history record
    report TRUE
    ] 
  [
    report FALSE
    ]
end

to step_IDLE_GR
  if has-message "GRDY" [
    if visual-aids [set shape "grdy"]
    let msg but-first received "GRDY"
    let sender protocal_sender_aware_sender msg
    set msg protocal_sender_aware_payload msg
    set sender neighbour-by-id sender
    set sender root
    let to-root-dis euclidean-distance (list xcor ycor) (but-first root) 
    if not root-check msg [redirect-GR msg to-root-dis sender]
    ]
  
  if has-message "FACE" [
    if visual-aids [set shape "face"]
    let msg but-first received "FACE"
    let sender neighbour-by-id protocal_sender_aware_sender msg
    set msg protocal_sender_aware_payload msg
    let to-root-dis last msg
    set msg but-last msg
    if not root-check msg [redirect-GR msg to-root-dis sender]
    ]
  
  ;; on sensing entering event
  if ticks > 1000 [
  ifelse who = first root [
    let msgs on-sensing-movement TRUE
    ]
  [
    let sensor self
    let in-range-objects objects with [within-sensing-range sensor]
    ;;visual effect
    if visual-aids [if count in-range-objects > 0 [highlight-sensing-range] ]
    foreach sort in-range-objects [
      if which-active-record [who] of ? = -1 [
        let temprecord (list "NULL" 0 "NULL")
        set temprecord replace-item 0 temprecord [who] of ?
        set temprecord replace-item 1 temprecord ticks
        set m lput temprecord m
        let msg (list first temprecord who bounding-box item 1 temprecord)
        if ground-truth-check [
          update-global-history msg
          centralized-cdc-validation first msg
          ]
        count-event
        let to-root-dis euclidean-distance (list xcor ycor) (but-first root)
        redirect-GR msg to-root-dis root
        ]
      ]
    close-inactive-records
    ]
  ]  
end
;;;;;;;;;;;;;;;;;;;;;;;;;;;Georouting (GPSR);;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; neighbourhood exploration
to step_XPNB [payload]
  broadcast fput "RANGE" payload
  become "WTNB"
end

to step_WTNB [neighbourhood-info-process]
  if has-message "RANGE" [
    let msg received "RANGE"
    let record but-first msg
    set neighbourhood lput record neighbourhood
    ]
  if length neighbourhood = count comlink-neighbors [
    run neighbourhood-info-process
    ]
end

;; on sensing entering events, report messages to be spreaded 
to-report on-sensing-movement [keep-history]
  ;; on sensing an entering event
  let msgs []
  let sensor self
  let in-range-objects objects with [within-sensing-range sensor] 
  ;;visual effect
  if visual-aids [ifelse count in-range-objects > 0 [highlight-sensing-range] [clear-sensing-range]]
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
      log-results testresultline
      ;;count-event
    ]
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

to setup-zzone
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
end

to setup-output
  set output-cache []  
  set filename "../mtz-tests/flooding-cdc.csv"
  file-close-all
  if output-to-file [
  if file-exists? filename [file-delete filename]
  file-open filename
  file-print "Object, ticks, decentralized, centralized"
  ]
  print "Object, ticks, decentralized, centralized"
end

to setup-objects
  let boxsize world-width / 80
  let box-top random-box-top boxsize
  let box-right random-box-right boxsize
  ask objects [
    set predis 0
    let x-offset random boxsize
    let y-offset random boxsize
    setxy (box-right - x-offset) (box-top - y-offset)
    facexy 0 0
    ]
end

to output-load-balance
  ;;set-current-directory user-directory
  set-current-directory "/Users/yaguangt1/Desktop/"
  let lb-filename communicationstrategy
  ifelse empty? load-balance-data [
    set lb-filename (word lb-filename "_loadbalance")
    ]
  [
    set lb-filename (word lb-filename "_" load-balance-data)
    ]
  if substring lb-filename (length lb-filename - 4) (length lb-filename) != ".csv" [
    set lb-filename (word lb-filename ".csv")
    ]
  if file-exists? lb-filename [file-delete lb-filename]
  file-open lb-filename
  file-print "mote,msgsent"
  ask maincomponent [
    file-print (word who "," num-msgsent)
    ]
  file-close
end

to kill-random-sensor
  let num-to-kill ifelse-value is-number? failure-sensor-proportion [ceiling (failure-sensor-proportion / 100 * netsize)] [1]
  with-local-randomness [
  ask n-of num-to-kill motes [
      let dying-node who
      ask link-neighbors [
        set neighbourhood remove-item-by-key neighbourhood dying-node
        set towards-neighbour remove-item-by-key towards-neighbour dying-node
        set similar-neighbour remove-item-by-key similar-neighbour dying-node
        if is-list? closest-neighbour and dying-node = first closest-neighbour [set-closest-neighbour]
      ]
      die
    ]
  ]
end

to reset-log-cache
  set ct-event 0
  set ct-dgt -1
  set ct-cgt -1
  set predicate-list []
  set groundtruth-list []
  set init-msg-ct 0
  set moniter-msg-ct 0
end

to output-logline
  if ct-event > 0 [
    set output-cache lput (list netsize CommunicationStrategy current-seed c s ticks move-step ct-event ct-dgt ct-cgt predicate-list groundtruth-list init-msg-ct moniter-msg-ct ) output-cache
    set total-events total-events + 1
    ]
  reset-log-cache
end

to write-log-to-file [log-file]
  file-close-all
  ifelse file-exists? log-file [
    file-open log-file
    ]
  [
    file-open log-file
    file-print "netsize,alg,seed,c,s,tick,movestep,ct-event,ct-dgt,ct-cgt,predicate-list,groundtruth-list,init-msg-ct,moniter-msg-ct"
    ]
  foreach output-cache [
    let index 0
    while [index < 13] [
      file-type (word item index ? ",")
      set index index + 1
      ]
    file-print item 13 ?
    ]
  file-close
end

to print-log
  ;;print (word "ct-event:" ct-event " ct-dgt:" ct-dgt " ct-cgt:" ct-cgt " predicate:" predicate-list " groundtruth:" groundtruth-list)
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
375
60
435
Netsize
250
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
565
130
598
trackmsg
trackmsg
0
1
-1000

INPUTBOX
10
170
60
230
c
20
1
0
Number

INPUTBOX
60
170
110
230
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
10
275
60
335
ObjNo
1
1
0
Number

SWITCH
10
725
125
758
show-links
show-links
1
1
-1000

SWITCH
10
760
125
793
show-tails
show-tails
1
1
-1000

INPUTBOX
130
760
210
820
tail-width
3
1
0
Number

CHOOSER
10
800
110
845
MoteLabel
MoteLabel
"none" "mote id" "m" "zr"
0

OUTPUT
1205
645
1495
825
30

MONITOR
1205
530
1287
575
sent length
sent-length-msg-totals
17
1
11

MONITOR
1300
530
1392
575
sent number
sent-number-msg-totals
17
1
11

MONITOR
1205
585
1287
630
recv length
recv-length-msg-totals
17
1
11

MONITOR
1300
585
1392
630
recv number
recv-number-msg-totals
17
1
11

CHOOSER
155
495
295
540
Seed
Seed
"none" "random" "manual"
2

INPUTBOX
155
540
295
600
current-seed
801204666
1
0
Number

CHOOSER
65
275
210
320
move-type
move-type
"Simple Linear" "CRW" "Boids/Flocking"
1

SWITCH
10
495
152
528
output-to-file
output-to-file
1
1
-1000

CHOOSER
60
375
198
420
NetworkStructure
NetworkStructure
"UDG" "GG" "RNG"
0

CHOOSER
10
75
155
120
CommunicationStrategy
CommunicationStrategy
"Flooding" "Hybrid" "Direction-based" "CDC-similarity" "Shortest-path-tree" "CDC-towards" "GPSR"
6

MONITOR
1405
530
1502
575
Interior nodes
interior-num
17
1
11

MONITOR
1405
590
1517
635
Boundary nodes
boundary-num
17
1
11

INPUTBOX
160
75
312
135
searching-steps
3
1
0
Number

INPUTBOX
110
170
160
230
CMR
100
1
0
Number

SWITCH
10
460
192
493
ground-truth-check
ground-truth-check
0
1
-1000

SWITCH
10
530
152
563
calc-diameter
calc-diameter
1
1
-1000

SLIDER
1205
65
1435
98
vision
vision
0.0
10.0
3
0.5
1
patches
HORIZONTAL

SLIDER
1205
100
1436
133
minimum-separation
minimum-separation
0.0
5.0
1
0.25
1
patches
HORIZONTAL

SLIDER
1205
135
1435
168
max-align-turn
max-align-turn
0.0
20.0
6
0.25
1
degrees
HORIZONTAL

SLIDER
1205
170
1435
203
max-cohere-turn
max-cohere-turn
0.0
20.0
4
0.25
1
degrees
HORIZONTAL

SLIDER
1205
205
1435
238
max-separate-turn
max-separate-turn
0.0
20.0
2
0.25
1
degrees
HORIZONTAL

SLIDER
1205
250
1397
283
min-group-pop
min-group-pop
1
10
1
1
1
objects
HORIZONTAL

SLIDER
1205
285
1377
318
group-radius
group-radius
0
5
0
1
1
hops
HORIZONTAL

INPUTBOX
1205
375
1375
435
load-balance-data
NIL
1
0
String

BUTTON
1380
390
1462
423
OUTPUT
output-load-balance
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL
0

SWITCH
130
725
252
758
visual-aids
visual-aids
0
1
-1000

TEXTBOX
1210
25
1360
43
Experimental Functions
13
15.0
1

TEXTBOX
1210
40
1360
58
Group Movements
13
0.0
1

TEXTBOX
10
440
160
458
Experiment Settings
13
0.0
1

TEXTBOX
10
150
160
168
Sensor Settings
13
0.0
1

TEXTBOX
10
255
160
273
Moving Object Settings
13
0.0
1

TEXTBOX
15
705
165
723
Visual Effects
13
0.0
1

TEXTBOX
10
355
160
373
Network Settings
13
0.0
1

TEXTBOX
15
55
165
73
Algorithm Settings
13
0.0
1

SWITCH
10
600
147
633
sensorfailure
sensorfailure
1
1
-1000

SWITCH
165
175
332
208
fixed-connectivity
fixed-connectivity
1
1
-1000

INPUTBOX
10
640
150
700
failure-sensor-proportion
20
1
0
Number

INPUTBOX
160
640
295
700
fail-ticks
1000
1
0
Number

@#$#@#$#@
## PROTOCOL

Monitoring movement towards a zone of interest using qualitative directional information

## SUMMARY



## OPERATION



## NOTICE


## TRY

## CREDITS

Builds on and extends code from Duckham, 2013

Re-uses and modifies code from "Boids" / NetLogo "Flocking" model:
Wilensky, U. (1998). NetLogo Flocking model. http://ccl.northwestern.edu/netlogo/models/Flocking. Center for Connected Learning and Computer-Based Modeling, Northwestern University, Evanston, IL.

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

face
true
0
Polygon -1184463 true false 135 15 105 120 60 150 120 195 105 225 165 225 225 180 165 165 135 135 165 60 135 30

fish
true
0
Polygon -7500403 true true 150 60 120 90 105 150 135 210 105 240 195 240 165 210 195 150 180 90
Polygon -16777216 false false 150 60 120 90 105 150 135 210 105 240 195 240 165 210 195 150 180 90

grdy
true
0
Circle -8630108 true false 116 116 67

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
NetLogo 5.0.1
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
  <experiment name="e23" repetitions="1" runMetricsEveryStep="true">
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
  <experiment name="e24" repetitions="1" runMetricsEveryStep="true">
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
      <value value="-1338276584"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
  <experiment name="e25" repetitions="1" runMetricsEveryStep="true">
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
  <experiment name="e26" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="1000"/>
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
      <value value="1310021339"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="50"/>
      <value value="250"/>
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
  <experiment name="e27" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="1000"/>
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
      <value value="1310021339"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="50"/>
      <value value="250"/>
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
      <value value="3"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e28" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="1000"/>
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
      <value value="1310021339"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="50"/>
      <value value="250"/>
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
      <value value="3"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e29" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="1000"/>
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
      <value value="1310021339"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="50"/>
      <value value="250"/>
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
      <value value="3"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e30" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="1000"/>
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
      <value value="1310021339"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="50"/>
      <value value="250"/>
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
      <value value="&quot;GPSR&quot;"/>
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
  <experiment name="e31" repetitions="1" runMetricsEveryStep="true">
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
  <experiment name="e32" repetitions="1" runMetricsEveryStep="true">
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
      <value value="-1338276584"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
  <experiment name="e33" repetitions="1" runMetricsEveryStep="true">
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
  <experiment name="e34" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;GPSR&quot;"/>
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
  <experiment name="e35" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;GPSR&quot;"/>
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
  <experiment name="e36" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e37" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-1338276584"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e38" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e101_load_balance" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <final>output-load-balance</final>
    <timeLimit steps="100000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="101"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e102_robustness" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="20000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
      <value value="1"/>
      <value value="10"/>
      <value value="-1"/>
      <value value="555"/>
      <value value="666"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="failure-sensor-proportion">
      <value value="0"/>
      <value value="1"/>
      <value value="5"/>
      <value value="10"/>
      <value value="20"/>
      <value value="40"/>
      <value value="80"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="fail-ticks">
      <value value="1000"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e103_various_hops" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
      <value value="1"/>
      <value value="10"/>
      <value value="100"/>
      <value value="-100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
      <value value="&quot;CDC-towards&quot;"/>
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
      <value value="1"/>
      <value value="2"/>
      <value value="3"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="failure-sensor-proportion">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="fail-ticks">
      <value value="1000"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e39" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
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
      <value value="3"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e40" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-1338276584"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
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
      <value value="3"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e41" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
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
      <value value="3"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e42" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e43" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-1338276584"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e44" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="1310021339"/>
      <value value="-1077699878"/>
      <value value="-21750868"/>
      <value value="-626351809"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e45" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="1736625203"/>
      <value value="-157514395"/>
      <value value="1806136444"/>
      <value value="801204666"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e46" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e47" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-1338276584"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e48" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="1310021339"/>
      <value value="-1077699878"/>
      <value value="-21750868"/>
      <value value="-626351809"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e49" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="1736625203"/>
      <value value="-157514395"/>
      <value value="1806136444"/>
      <value value="801204666"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e50" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="2000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="199234770"/>
      <value value="2126839149"/>
      <value value="-1075215490"/>
      <value value="426340273"/>
      <value value="918098369"/>
      <value value="1601265801"/>
      <value value="-1285833180"/>
      <value value="1944336071"/>
      <value value="-1701481219"/>
      <value value="1225483122"/>
      <value value="-1184662325"/>
      <value value="-1279222226"/>
      <value value="886269248"/>
      <value value="1983226376"/>
      <value value="2121556210"/>
      <value value="146263706"/>
      <value value="427158359"/>
      <value value="458654036"/>
      <value value="1361396208"/>
      <value value="-981349168"/>
      <value value="1849720905"/>
      <value value="-1891741125"/>
      <value value="-1286362165"/>
      <value value="-404776106"/>
      <value value="-777579243"/>
      <value value="1340668975"/>
      <value value="184634490"/>
      <value value="856794903"/>
      <value value="-1887525501"/>
      <value value="-843570571"/>
      <value value="234695050"/>
      <value value="-254395013"/>
      <value value="346884282"/>
      <value value="2053563118"/>
      <value value="935084157"/>
      <value value="235350690"/>
      <value value="99645020"/>
      <value value="1066889412"/>
      <value value="-2122105652"/>
      <value value="684447155"/>
      <value value="203792430"/>
      <value value="1671255988"/>
      <value value="209535595"/>
      <value value="442859968"/>
      <value value="-1359680172"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e51" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="2000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="1968193823"/>
      <value value="-624557683"/>
      <value value="-1850302097"/>
      <value value="595620922"/>
      <value value="731509428"/>
      <value value="-1538579864"/>
      <value value="408498025"/>
      <value value="-1831750279"/>
      <value value="-1498043179"/>
      <value value="-2012342585"/>
      <value value="-2044405160"/>
      <value value="-1935923704"/>
      <value value="1611618664"/>
      <value value="-1713329509"/>
      <value value="1365902865"/>
      <value value="1441754299"/>
      <value value="408690988"/>
      <value value="921738965"/>
      <value value="1965397121"/>
      <value value="1943841441"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e52" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="2000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="199234770"/>
      <value value="2126839149"/>
      <value value="-1075215490"/>
      <value value="426340273"/>
      <value value="918098369"/>
      <value value="1601265801"/>
      <value value="-1285833180"/>
      <value value="1944336071"/>
      <value value="-1701481219"/>
      <value value="1225483122"/>
      <value value="-1184662325"/>
      <value value="-1279222226"/>
      <value value="886269248"/>
      <value value="1983226376"/>
      <value value="2121556210"/>
      <value value="146263706"/>
      <value value="427158359"/>
      <value value="458654036"/>
      <value value="1361396208"/>
      <value value="-981349168"/>
      <value value="1849720905"/>
      <value value="-1891741125"/>
      <value value="-1286362165"/>
      <value value="-404776106"/>
      <value value="-777579243"/>
      <value value="1340668975"/>
      <value value="184634490"/>
      <value value="856794903"/>
      <value value="-1887525501"/>
      <value value="-843570571"/>
      <value value="234695050"/>
      <value value="-254395013"/>
      <value value="346884282"/>
      <value value="2053563118"/>
      <value value="935084157"/>
      <value value="235350690"/>
      <value value="99645020"/>
      <value value="1066889412"/>
      <value value="-2122105652"/>
      <value value="684447155"/>
      <value value="203792430"/>
      <value value="1671255988"/>
      <value value="209535595"/>
      <value value="442859968"/>
      <value value="-1359680172"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e53" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="2000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="1968193823"/>
      <value value="-624557683"/>
      <value value="-1850302097"/>
      <value value="595620922"/>
      <value value="731509428"/>
      <value value="-1538579864"/>
      <value value="408498025"/>
      <value value="-1831750279"/>
      <value value="-1498043179"/>
      <value value="-2012342585"/>
      <value value="-2044405160"/>
      <value value="-1935923704"/>
      <value value="1611618664"/>
      <value value="-1713329509"/>
      <value value="1365902865"/>
      <value value="1441754299"/>
      <value value="408690988"/>
      <value value="921738965"/>
      <value value="1965397121"/>
      <value value="1943841441"/>
      <value value="143861080"/>
      <value value="-889913198"/>
      <value value="1072592003"/>
      <value value="350083400"/>
      <value value="1450185415"/>
      <value value="1308047612"/>
      <value value="300414372"/>
      <value value="509564693"/>
      <value value="1703346065"/>
      <value value="-1966990726"/>
      <value value="1610712935"/>
      <value value="-625556126"/>
      <value value="634811492"/>
      <value value="-404905123"/>
      <value value="564272557"/>
      <value value="-1325302558"/>
      <value value="-250662583"/>
      <value value="-1406683890"/>
      <value value="899698809"/>
      <value value="-1425951580"/>
      <value value="-1043874461"/>
      <value value="-1146635944"/>
      <value value="-718028608"/>
      <value value="-1856174111"/>
      <value value="1229448163"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e54" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
      <value value="-1338276584"/>
      <value value="1310021339"/>
      <value value="-1077699878"/>
      <value value="-21750868"/>
      <value value="-626351809"/>
      <value value="1736625203"/>
      <value value="-157514395"/>
      <value value="1806136444"/>
      <value value="801204666"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
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
      <value value="3"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e55" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
      <value value="-1338276584"/>
      <value value="1310021339"/>
      <value value="-1077699878"/>
      <value value="-21750868"/>
      <value value="-626351809"/>
      <value value="1736625203"/>
      <value value="-157514395"/>
      <value value="1806136444"/>
      <value value="801204666"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
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
      <value value="3"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e56" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="2000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="143861080"/>
      <value value="-889913198"/>
      <value value="1072592003"/>
      <value value="350083400"/>
      <value value="1450185415"/>
      <value value="1308047612"/>
      <value value="300414372"/>
      <value value="509564693"/>
      <value value="1703346065"/>
      <value value="-1966990726"/>
      <value value="1610712935"/>
      <value value="-625556126"/>
      <value value="634811492"/>
      <value value="-404905123"/>
      <value value="564272557"/>
      <value value="-1325302558"/>
      <value value="-250662583"/>
      <value value="-1406683890"/>
      <value value="899698809"/>
      <value value="-1425951580"/>
      <value value="-1043874461"/>
      <value value="-1146635944"/>
      <value value="-718028608"/>
      <value value="-1856174111"/>
      <value value="1229448163"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e57" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="2000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="199234770"/>
      <value value="2126839149"/>
      <value value="-1075215490"/>
      <value value="426340273"/>
      <value value="918098369"/>
      <value value="1601265801"/>
      <value value="-1285833180"/>
      <value value="1944336071"/>
      <value value="-1701481219"/>
      <value value="1225483122"/>
      <value value="-1184662325"/>
      <value value="-1279222226"/>
      <value value="886269248"/>
      <value value="1983226376"/>
      <value value="2121556210"/>
      <value value="146263706"/>
      <value value="427158359"/>
      <value value="458654036"/>
      <value value="1361396208"/>
      <value value="-981349168"/>
      <value value="1849720905"/>
      <value value="-1891741125"/>
      <value value="-1286362165"/>
      <value value="-404776106"/>
      <value value="-777579243"/>
      <value value="1340668975"/>
      <value value="184634490"/>
      <value value="856794903"/>
      <value value="-1887525501"/>
      <value value="-843570571"/>
      <value value="234695050"/>
      <value value="-254395013"/>
      <value value="346884282"/>
      <value value="2053563118"/>
      <value value="935084157"/>
      <value value="235350690"/>
      <value value="99645020"/>
      <value value="1066889412"/>
      <value value="-2122105652"/>
      <value value="684447155"/>
      <value value="203792430"/>
      <value value="1671255988"/>
      <value value="209535595"/>
      <value value="442859968"/>
      <value value="-1359680172"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e58" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="2000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="1968193823"/>
      <value value="-624557683"/>
      <value value="-1850302097"/>
      <value value="595620922"/>
      <value value="731509428"/>
      <value value="-1538579864"/>
      <value value="408498025"/>
      <value value="-1831750279"/>
      <value value="-1498043179"/>
      <value value="-2012342585"/>
      <value value="-2044405160"/>
      <value value="-1935923704"/>
      <value value="1611618664"/>
      <value value="-1713329509"/>
      <value value="1365902865"/>
      <value value="1441754299"/>
      <value value="408690988"/>
      <value value="921738965"/>
      <value value="1965397121"/>
      <value value="1943841441"/>
      <value value="143861080"/>
      <value value="-889913198"/>
      <value value="1072592003"/>
      <value value="350083400"/>
      <value value="1450185415"/>
      <value value="1308047612"/>
      <value value="300414372"/>
      <value value="509564693"/>
      <value value="1703346065"/>
      <value value="-1966990726"/>
      <value value="1610712935"/>
      <value value="-625556126"/>
      <value value="634811492"/>
      <value value="-404905123"/>
      <value value="564272557"/>
      <value value="-1325302558"/>
      <value value="-250662583"/>
      <value value="-1406683890"/>
      <value value="899698809"/>
      <value value="-1425951580"/>
      <value value="-1043874461"/>
      <value value="-1146635944"/>
      <value value="-718028608"/>
      <value value="-1856174111"/>
      <value value="1229448163"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e59" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
      <value value="-1338276584"/>
      <value value="1310021339"/>
      <value value="-1077699878"/>
      <value value="-21750868"/>
      <value value="-626351809"/>
      <value value="1736625203"/>
      <value value="-157514395"/>
      <value value="1806136444"/>
      <value value="801204666"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e60" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
      <value value="-1338276584"/>
      <value value="1310021339"/>
      <value value="-1077699878"/>
      <value value="-21750868"/>
      <value value="-626351809"/>
      <value value="1736625203"/>
      <value value="-157514395"/>
      <value value="1806136444"/>
      <value value="801204666"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="true"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="test-DIY-output" repetitions="1" runMetricsEveryStep="false">
    <setup>setup
initialize</setup>
    <go>go</go>
    <final>write-log-to-file "../mtz-tests/test_test.csv"</final>
    <timeLimit steps="20000"/>
    <exitCondition>enough-events? 10</exitCondition>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="1968193823"/>
      <value value="2"/>
      <value value="5"/>
      <value value="100"/>
      <value value="200"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="100"/>
      <value value="250"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="diy_e1" repetitions="1" runMetricsEveryStep="false">
    <setup>setup
initialize</setup>
    <go>go</go>
    <final>write-log-to-file "../mtz-tests/diy_e1.csv"</final>
    <timeLimit steps="20000"/>
    <exitCondition>enough-events? 10</exitCondition>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="199234770"/>
      <value value="2126839149"/>
      <value value="-1075215490"/>
      <value value="426340273"/>
      <value value="918098369"/>
      <value value="1601265801"/>
      <value value="-1285833180"/>
      <value value="1944336071"/>
      <value value="-1701481219"/>
      <value value="1225483122"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="diy_e2" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <final>write-log-to-file "../mtz-tests/diy_e2.csv"</final>
    <timeLimit steps="20000"/>
    <exitCondition>enough-events? 10</exitCondition>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-1184662325"/>
      <value value="-1279222226"/>
      <value value="886269248"/>
      <value value="1983226376"/>
      <value value="2121556210"/>
      <value value="146263706"/>
      <value value="427158359"/>
      <value value="458654036"/>
      <value value="1361396208"/>
      <value value="-981349168"/>
      <value value="1849720905"/>
      <value value="-1891741125"/>
      <value value="-1286362165"/>
      <value value="-404776106"/>
      <value value="-777579243"/>
      <value value="1340668975"/>
      <value value="184634490"/>
      <value value="856794903"/>
      <value value="-1887525501"/>
      <value value="-843570571"/>
      <value value="234695050"/>
      <value value="-254395013"/>
      <value value="346884282"/>
      <value value="2053563118"/>
      <value value="935084157"/>
      <value value="235350690"/>
      <value value="99645020"/>
      <value value="1066889412"/>
      <value value="-2122105652"/>
      <value value="684447155"/>
      <value value="203792430"/>
      <value value="1671255988"/>
      <value value="209535595"/>
      <value value="442859968"/>
      <value value="-1359680172"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
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
      <value value="&quot;CDC-towards&quot;"/>
      <value value="&quot;Shortest-path-tree&quot;"/>
      <value value="&quot;GPSR&quot;"/>
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
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="e105_various_hops" repetitions="1" runMetricsEveryStep="true">
    <setup>setup
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <metric>show-move-step</metric>
    <metric>ct-sent-number-msg-totals</metric>
    <metric>ct-sent-number-msg-totals-by-name "ZBOX"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RANGE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "TREE"</metric>
    <metric>ct-sent-number-msg-totals-by-name "RTPS"</metric>
    <metric>ct-sent-number-msg-totals-by-name "AEXT"</metric>
    <metric>ct-sent-number-msg-totals-by-name "OETR"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FLOD"</metric>
    <metric>ct-sent-number-msg-totals-by-name "GRDY"</metric>
    <metric>ct-sent-number-msg-totals-by-name "FACE"</metric>
    <metric>show-current-seed</metric>
    <metric>show-moving-towards</metric>
    <metric>show-true-moving-towards</metric>
    <metric>show-ct-dgt</metric>
    <metric>show-ct-cgt</metric>
    <metric>show-ct-event</metric>
    <metric>true-population-motes</metric>
    <metric>show-c</metric>
    <metric>show-s</metric>
    <enumeratedValueSet variable="Seed">
      <value value="&quot;manual&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="current-seed">
      <value value="-640111348"/>
      <value value="1"/>
      <value value="10"/>
      <value value="100"/>
      <value value="-100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="Netsize">
      <value value="500"/>
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
    <enumeratedValueSet variable="fixed-connectivity">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CMR">
      <value value="100"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="CommunicationStrategy">
      <value value="&quot;Flooding&quot;"/>
      <value value="&quot;CDC-towards&quot;"/>
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
      <value value="1"/>
      <value value="2"/>
      <value value="3"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="ground-truth-check">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="sensorfailure">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="failure-sensor-proportion">
      <value value="1"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="fail-ticks">
      <value value="1000"/>
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
