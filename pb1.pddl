(define (problem pb1)
  (:domain blockworld)
  (:init 
    (arm-empty)
    (spam-on-countertop)
    (sugar-on-stovetop)
    (drawer-closed)
  )
  (:goal (and
    (sugar-on-countertop)
    (spam-in-drawer)
    (drawer-closed)
    (arm-empty)
  ))
)