; This is a comment line
(define (domain kitchenDomain)
  (:requirements :strips :negative-preconditions)
  (:predicates
    (drawer-closed)
    (cabinet-open)
    (arm-empty)
    (spam-on-countertop)
    (spam-in-drawer)
    (spam-on-stovetop)
    (sugar-on-countertop)
    (sugar-in-drawer)
    (sugar-on-stovetop)
  )
  (:action pickup-sugar
    :precondition (and
      (arm-empty)
      (sugar-on-stovetop) 
    )
    :effect (and
      (not(arm-empty))
      (not(sugar-on-stovetop))
    )
  )
  (:action place-sugar
    :precondition (and
      (not(arm-empty))
      (not(sugar-on-stovetop))
      (not(spam-on-countertop))
    )
    :effect (and
      (sugar-on-countertop)
      (arm-empty)
    )
  )
  (:action pickup-spam
    :precondition (and
      (arm-empty)
      (spam-on-countertop)
    )
    :effect (and
      (not(arm-empty))
      (not(spam-on-countertop))
    )
  )
  (:action place-spam
    :precondition (and
      (not(drawer-closed))
      (not(arm-empty))
      (not(spam-on-countertop))
    )
    :effect (and
      (spam-in-drawer)
      (arm-empty)
    )
  )
  (:action open-drawer
    :precondition (and
      (arm-empty)
      (drawer-closed)
    )
    :effect (not(drawer-closed))
  )
  
  (:action close-drawer
    :precondition (and
      (arm-empty)
      (not(drawer-closed))
    )
    :effect (drawer-closed)
  )
)