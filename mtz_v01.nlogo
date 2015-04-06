 __includes["./gsn_mtz.nls" "./env_mtz.nls"]
;; Define a new breed of turtle called motes (i.e. the (static) sensor nodes)
breed [motes mote]
motes-own [m zr]

;; Define a new breed of turtle called motes (i.e. moving objects)
breed [objects object]

;; anchor vertices for target zone
breed [tzonevertices tzonevertex]
;; bounding box is represented as list of cor: [top left bottom right]
globals [targetzone-boundingbox]
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
  
  ask motes [
    set m []
    ;set shape "sensor"
    ;calculate dir(me,Z), store in zr, use me as a reference
    set zr CDC-dir bounding-box targetzone-boundingbox
    become "IDLE"
  ]
  
  reset-ticks
end

;; Run the algorithm
to go
  ask motes [step]
  move-objects
  mote-labels
  object-tails
  tick
end

;;
;; Mote protocols
;;

;; Step through the current state
to step
  if state = "INIT" [step_INIT stop]
  if state = "IDLE" [step_IDLE stop]
end

to step_INIT
    stop   ;; currently skeleton
end

to step_IDLE
  let sensor self
  let in-range-objects objects with [ within-sensing-range sensor ] 
  ;; visual effect for sensing
  ifelse count in-range-objects > 0 [
    highlight-sensing-range
    ]
  [ clear-sensing-range ]
  
  ;; update history table to add new records when an object enters
  foreach sort in-range-objects [
    if which-active-record [who] of ? = -1 [
      let temprecord [ "NULL" 0 "NULL" ]
      set temprecord replace-item 0 temprecord [who] of ?
      set temprecord replace-item 1 temprecord ticks
      set m lput temprecord m
      ]
    ]
  ;; update history table to finish open records when corresponding objects cannot be sensed
  close-inactive-records
end

;; Move object (based on modified correlated random walk)
to move-objects
    ask objects [
      rt random-normal 0 15 ;; Change the heading based on Gaussian distribution with standard deviation of 15 degrees
      fd 1
      ;fd 0.1
    ]
end

;; Assign labels to motes based on the MoteLable dropdown list
to mote-labels
  ask motes [
    if MoteLabel = "none" [set label ""] ;; Hide the label
    if MoteLabel = "mote id" [set label who] ;; Show mote id
    if MoteLabel = "m" [set label m] ;; Show contents of m
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

to-report within-sensing-range [obj]
  ifelse distance obj <= s [
    report TRUE
  ]
  [ report FALSE ]
end

to highlight-sensing-range
  ;ask patches in-radius s [
   ; set pcolor yellow
    ;]
  ;set size s + world-width / 21
  set color green
end

to clear-sensing-range
  ;ask patches in-radius s [
   ; set pcolor white
    ;]
  ;set size world-width / 21
  set color 86
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

to-report random-box-right [boxsize]
  report (random (world-width - boxsize)) - (world-width / 2 - boxsize)
end

to-report random-box-top [boxsize]
  report (random (world-height - boxsize)) - (world-height / 2 - boxsize)
end

;; check whether a patch is within the convex hull defined by z-zone vertices
to-report in-convex-hull
  let thispatch self 
  let degrees [ towards thispatch ] of tzonevertices
  let lower filter [ ? <= 180 ] degrees
  let upper filter [ ? > 180 ] degrees
  if not empty? lower and not empty? upper and (360 + max lower) - min upper < 180 [report false]
  report max degrees - min degrees >= 180
end

;; calculate a bounding box for a sensing region
to-report bounding-box
  let boundingbox [0 0 0 0]
  set boundingbox replace-item 0 boundingbox (ycor + s)
  set boundingbox replace-item 1 boundingbox (xcor - s)
  set boundingbox replace-item 2 boundingbox (ycor - s)
  set boundingbox replace-item 3 boundingbox (xcor + s)
  report boundingbox
end

to set-target-bounding-box
  set targetzone-boundingbox [0 0 0 0]
  set targetzone-boundingbox replace-item 1 targetzone-boundingbox min [xcor] of tzonevertices
  set targetzone-boundingbox replace-item 3 targetzone-boundingbox max [xcor] of tzonevertices
  set targetzone-boundingbox replace-item 0 targetzone-boundingbox max [ycor] of tzonevertices
  set targetzone-boundingbox replace-item 2 targetzone-boundingbox min [ycor] of tzonevertices
end


to-report CDC-dir [reference target]
  let ref-top item 0 reference
  let ref-left item 1 reference
  let ref-bottom item 2 reference
  let ref-right item 3 reference
  let tar-top item 0 target
  let tar-left item 1 target
  let tar-bottom item 2 target
  let tar-right item 3 target
  let cdc []
  if tar-left < ref-left [
    if tar-top > ref-top [ set cdc lput "NW" cdc]
    if tar-bottom < ref-bottom [set cdc lput "SW" cdc]
    ]
  if tar-right > ref-right [
    if tar-top > ref-top [ set cdc lput "NE" cdc ]
    if tar-bottom < ref-bottom [ set cdc lput "SE" cdc ]
    ]
  if tar-right > ref-left and tar-left < ref-right [
    if tar-top > ref-top [set cdc lput "N" cdc ]
    if tar-bottom < ref-bottom [set cdc lput "S" cdc ]
    ]
  if tar-top > ref-bottom and tar-bottom < ref-top [
    if tar-left < ref-left [ set cdc lput "W" cdc ]
    if tar-right > ref-right [ set cdc lput "E" cdc ]
    ]
  report cdc
end
@#$#@#$#@
GRAPHICS-WINDOW
220
10
632
443
100
100
2.0
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
60
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
15
280
150
313
trackmsg
trackmsg
1
1
-1000

INPUTBOX
110
45
160
105
c
1
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
3
1
0
Number

SWITCH
15
315
150
348
show-links
show-links
1
1
-1000

SWITCH
15
350
150
383
show-tails
show-tails
0
1
-1000

INPUTBOX
15
385
95
445
tail-width
3
1
0
Number

CHOOSER
15
235
153
280
MoteLabel
MoteLabel
"none" "mote id" "m"
1

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
Circle -7500403 true true 0 0 300

@#$#@#$#@
NetLogo 5.1.0
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
<experiments>
  <experiment name="Exp.1 Scalability" repetitions="100" runMetricsEveryStep="false">
    <setup>setup.convoy
initialize</setup>
    <go>go</go>
    <timeLimit steps="1000"/>
    <metric>GlobalMsgSent</metric>
    <metric>GlobalMsgLength</metric>
    <metric>min [LocalMsgSent] of motes</metric>
    <metric>max [LocalMsgSent] of motes</metric>
    <metric>mean [LocalMsgSent] of motes</metric>
    <metric>min [LocalMsgLength] of motes</metric>
    <metric>max [LocalMsgLength] of motes</metric>
    <metric>mean [LocalMsgLength] of motes</metric>
    <enumeratedValueSet variable="Netsize">
      <value value="50"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="fish-number">
      <value value="2000"/>
      <value value="1000"/>
      <value value="500"/>
      <value value="250"/>
      <value value="125"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="5.5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;GG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="velocity">
      <value value="0.03"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="swim">
      <value value="&quot;Complex&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="MoteLabel">
      <value value="&quot;none&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="AllowQueries?">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="RunPlots?">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="RunLatencyExperiment?">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="RunGroupExperiment?">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="fishmove?">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="Exp.2 Latency" repetitions="100" runMetricsEveryStep="false">
    <setup>setup.convoy
initialize</setup>
    <go>go</go>
    <timeLimit steps="10000"/>
    <exitCondition>Qcorrect = count motes</exitCondition>
    <metric>Qd</metric>
    <metric>Qt</metric>
    <metric>item 0 Q%correct</metric>
    <metric>item 1 Q%correct</metric>
    <metric>item 2 Q%correct</metric>
    <metric>item 3 Q%correct</metric>
    <metric>item 4 Q%correct</metric>
    <metric>item 5 Q%correct</metric>
    <metric>item 6 Q%correct</metric>
    <metric>item 7 Q%correct</metric>
    <metric>item 8 Q%correct</metric>
    <metric>Qc</metric>
    <metric>Qcorrect</metric>
    <enumeratedValueSet variable="Netsize">
      <value value="50"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="fish-number">
      <value value="500"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="5.5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="velocity">
      <value value="0.03"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="swim">
      <value value="&quot;Simple&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="MoteLabel">
      <value value="&quot;none&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="AllowQueries?">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="RunPlots?">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="RunLatencyExperiment?">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="RunGroupExperiment?">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="fishmove?">
      <value value="true"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="Exp.3 Group movement" repetitions="100" runMetricsEveryStep="false">
    <setup>setup.convoy
initialize</setup>
    <go>go</go>
    <timeLimit steps="1100"/>
    <metric>group.msg</metric>
    <metric>group.num</metric>
    <enumeratedValueSet variable="Netsize">
      <value value="50"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="fish-number">
      <value value="500"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="5.5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;UDG&quot;"/>
      <value value="&quot;GG&quot;"/>
      <value value="&quot;Tree (GG)&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="velocity">
      <value value="0.03"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="swim">
      <value value="&quot;Simple&quot;"/>
      <value value="&quot;Turning Angle&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="MoteLabel">
      <value value="&quot;none&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="RunPlots?">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="AllowQueries?">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="RunLatencyExperiment?">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="RunGroupExperiment?">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="fishmove?">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="g.s">
      <value value="3"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="g.l">
      <value value="3"/>
    </enumeratedValueSet>
  </experiment>
  <experiment name="Exp.4 Group size, length" repetitions="100" runMetricsEveryStep="false">
    <setup>setup.convoy
initialize</setup>
    <go>go</go>
    <timeLimit steps="1100"/>
    <metric>group.msg</metric>
    <metric>group.num</metric>
    <enumeratedValueSet variable="Netsize">
      <value value="50"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="fish-number">
      <value value="500"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="c">
      <value value="5.5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="NetworkStructure">
      <value value="&quot;GG&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="velocity">
      <value value="0.03"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="swim">
      <value value="&quot;Turning Angle&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="MoteLabel">
      <value value="&quot;none&quot;"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="AllowQueries?">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="RunPlots?">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="RunLatencyExperiment?">
      <value value="false"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="RunGroupExperiment?">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="fishmove?">
      <value value="true"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="g.s">
      <value value="2"/>
      <value value="3"/>
      <value value="4"/>
      <value value="5"/>
    </enumeratedValueSet>
    <enumeratedValueSet variable="g.l">
      <value value="2"/>
      <value value="3"/>
      <value value="4"/>
      <value value="5"/>
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

@#$#@#$#@
1
@#$#@#$#@
