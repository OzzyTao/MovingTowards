 __includes["./gsn_mtz.nls" "./env_mtz.nls"]
;; Define a new breed of turtle called motes (i.e. the (static) sensor nodes)
breed [motes mote]
motes-own [m]

;; Define a new breed of turtle called motes (i.e. moving objects)
breed [objects object]

;; System setup and initialization
to initialize
  ask motes [
    set m []
    become "INIT"
  ]
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
end

to step_INIT
    stop   ;; currently skeleton
end

;; Move object (based on modified correlated random walk)
to move-objects
    ask objects [
      rt random-normal 0 15 ;; Change the heading based on Gaussian distribution with standard deviation of 15 degrees
      fd 0.1
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
@#$#@#$#@
GRAPHICS-WINDOW
220
10
650
461
10
10
20.0
1
12
1
1
1
0
0
0
1
-10
10
-10
10
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
50
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
0
1
0
Number

INPUTBOX
160
45
210
105
s
0
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
2

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
