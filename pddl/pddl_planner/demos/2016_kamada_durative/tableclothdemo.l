#!/usr/bin/env roseus
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :derive-relationship "package://pddl_planner/demos/2016_kamada_durative/tools.l")

(setq *problem* (instance pddl-problem :init :name 'example-problem :domain 'example-domain))
(send *problem* :objects
      '((robot1 . robot)
        (robot2 . robot)
        (init_pos . position)
        (tablefront . position)
        (clothfront . position)
        (kitchen_table_front . position)
        (table . obj)
        (kitchen_table . obj)
        (cup . obj)
        (dish . obj)
        ))

(setq *domain* (instance pddl-domain :init :name 'example-domain :durative-action t))
(send *domain* :requirements '(:typing :durative-actions :adl))
(send *domain* :types '(obj - object robot position cloth - object))
(send *domain* :constants '(TABLECLOTH - cloth
                                       CLOTHFRONT - position
                                       ))

(send *domain* :predicates '(
                             (on ?underobj - object ?obj - object)
                             (at-robot ?robot - robot ?pos - position)
                             (grasping ?robot - robot ?obj - object)
                             (working ?robot -robot)
                             (cloth-half-grabbed)
                             (cloth-full-grabbed)
                             (unstable ?cloth - object) ; avoiding placing obj on TABLECLOTH before setting
                             ))

(setq action-lst
      (list
       (instance pddl-action :init
                 :use-durative-action t
                 :name "pick"
                 ;; ?robot pick ?obj from ?underobj
                 :parameters '((?robot - robot ?obj - obj ?underobj - obj ?pos - position))
                 :duration '((= ?duration 10))
                 :precondition '(
                                 (at start (not (working ?robot)))
                                 (over all (at-robot ?robot ?pos))
                                 (at start (not (grasping ?robot ?obj)))
                                 (at start (on ?underobj ?obj))
                                 )
                 :effect '(
                           (at start (working ?robot))
                           (at end (not (working ?robot)))
                           (at end (grasping ?robot ?obj))
                           (at end (not (on ?underobj ?obj)))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "place"
                 ;; ?robot pick ?obj from ?underobj
                 :parameters '((?robot - robot ?obj - obj ?underobj - object ?pos - position)) ;; underobjをobject種とすることで、TABLECLOTHも含めることが出来る
                 :duration '((= ?duration 10))
                 :precondition '(
                                 (at start (not (working ?robot)))
                                 (over all (at-robot ?robot ?pos))
                                 (over all (not (unstable ?underobj)))
                                 (at start (grasping ?robot ?obj))
                                 (at start (not (on ?underobj ?obj)))
                                 )
                 :effect '(
                           (at start (working ?robot))
                           (at end (not (working ?robot)))
                           (at end (not (grasping ?robot ?obj)))
                           (at end (on ?underobj ?obj))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "move"
                 ;; ?robot move to TABLE
                 :parameters '((?robot - robot ?from - position ?to - position))
                 :duration '((= ?duration 1))
                 :precondition '(
                                 (at start (not (working ?robot)))
                                 (at start (not (at-robot ?robot ?to)))
                                 (at start (at-robot ?robot ?from))
                                 )
                 :effect '(
                           (at start (working ?robot))
                           (at end (not (working ?robot)))
                           (at end (at-robot ?robot ?to))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "grab_TABLECLOTH"
                 ;; ?robot move to TABLE
                 :parameters '((?robot - robot))
                 :duration '((= ?duration 20))
                 :precondition '(
                                 (at start (not (working ?robot)))
                                 (over all (at-robot ?robot CLOTHFRONT))
                                 (at start (not (cloth-half-grabbed)))
                                 (at start (not (grasping ?robot TABLECLOTH)))
                                 )
                 :effect '(
                           (at start (working ?robot))
                           (at end (not (working ?robot)))
                           (at end (cloth-half-grabbed))
                           (at end (grasping ?robot TABLECLOTH))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "grab_the_other_side_TABLECLOTH"
                 ;; ?robot move to TABLE
                 :parameters '((?robotB - robot ?robotA - robot))
                 :duration '((= ?duration 20))
                 :precondition '(
                                 (at start (not (working ?robotA)))
                                 (at start (not (working ?robotB)))
                                 (over all (at-robot ?robotA CLOTHFRONT))
                                 (over all (at-robot ?robotB CLOTHFRONT))
                                 (at start (grasping ?robotA TABLECLOTH))
                                 (at start (not (grasping ?robotB TABLECLOTH)))
                                 (at start (cloth-half-grabbed))
                                 (at start (not (cloth-full-grabbed)))
                                 )
                 :effect '(
                           (at start (working ?robotA))
                           (at start (working ?robotB))
                           (at end (not (working ?robotA)))
                           (at end (not (working ?robotB)))
                           (at end (grasping ?robotB TABLECLOTH))
                           (at end (not (cloth-half-grabbed)))
                           (at end (cloth-full-grabbed))
                           ))
       (instance pddl-action :init
                 :use-durative-action t
                 :name "set_TABLECLOTH"
                 ;; ?robot move to TABLE
                 :parameters '((?robotA - robot ?robotB - robot))
                 :duration '((= ?duration 20))
                 :precondition '(
                                 (at start (not (working ?robotA)))
                                 (at start (not (working ?robotB)))
                                 (over all (at-robot ?robotA TABLEFRONT))
                                 (over all (at-robot ?robotB TABLEFRONT))
                                 (at start (grasping ?robotA TABLECLOTH))
                                 (at start (grasping ?robotB TABLECLOTH))
                                 (at start (cloth-full-grabbed))
                                 (at start (unstable TABLECLOTH))
                                 )
                 :effect '(
                           (at start (working ?robotA))
                           (at start (working ?robotB))
                           (at end (not (working ?robotA)))
                           (at end (not (working ?robotB)))
                           (at end (not (cloth-full-grabbed)))
                           (at end (on TABLE TABLECLOTH))
                           (at end (not (unstable TABLECLOTH)))
                           ))
       ))
(dolist (ac action-lst)
  (send *domain* :add :action ac))

(send *problem* :initial-condition
      '(
        (at-robot robot1 init_pos)
        (at-robot robot2 init_pos)
        (on kitchen_table cup)
        (on kitchen_table dish)
        (unstable TABLECLOTH)
        ))

(send *problem* :goal-condition
      '(
;        (cloth-full-grabbed)
        (on TABLE TABLECLOTH)
        (on TABLECLOTH cup)
        (on TABLECLOTH dish)
        ))

(warn "~%")
(warn ";(demo)~%")









