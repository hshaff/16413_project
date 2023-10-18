(define (problem pb1)
  (:domain blockworld)
  (:init 
    (arm-empty)
    (spam-on-countertop)
    (sugar-on-stovetop)
  )
  (:goal (and
    (sugar-on-countertop)
    (spam-in-drawer)
    (not(drawer-open))
    (arm-empty)
  ))
)