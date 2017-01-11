#!/usr/bin/env roseus
(require :pddl-result-graph "package://pddl_planner/src/pddl-result-graph.l")
(require :eus-pddl-client "package://pddl_planner/src/eus-pddl-client.l")
(require :tools "package://pddl_planner/demos/2016_kamada_durative/tools.l")

(setq *problem* (instance pddl-problem :init :name 'example-problem :domain 'example-domain))
(send *problem* :objects
      '((robot1 . robot)
        (robot2 . robot)))

(setq *domain* (instance pddl-domain :init :name 'example-domain :durative-action t))
(send *domain* :requirements '(:typing :durative-actions :adl))
(send *domain* :types '(object robot position))
(send *domain* :constants '())

(send *domain* :predicates '(
                             (grasping-tablecloth ?robot - robot)
                             (full-grasp-tablecloth ?robot - robot)
                             (tablecloth-set)
                             ))

(setq action-lst
      (list
       (instance pddl-action :init
                 :use-durative-action t
                 :name "bring-tablecloth"
                 ;; ?robot pick ?obj from ?underobj
                 :parameters '((?robot - robot))
                 :duration '((= ?duration 10))
                 :precondition '(
                                 (at start (not (grasping-tablecloth ?robot)))
                                 )
                 :effect '(
                           (at end (grasping-tablecloth ?robot))
                           (at end (full-grasp-tablecloth ?robot))
                           ))

       (instance pddl-action :init
                 :use-durative-action t
                 :name "grasp-tablecloth"
                 ;; ?robot move to TABLE
                 :parameters '((?robotA - robot ?robotB - robot))
                 :duration '((= ?duration 10))
                 :precondition '(
                                 (over all (grasping-tablecloth ?robotB))
                                 (at start (not (full-grasp-tablecloth ?robotA)))
                                 )
                 :effect '(
                           (at end (full-grasp-tablecloth ?robotA))
                           (at end (grasping-tablecloth ?robotA))
                           ))

       (instance pddl-action :init
                 :use-durative-action t
                 :name "set-tablecloth"
                 ;; ?robot move to TABLE
                 :parameters '((?robotA - robot ?robotB - robot))
                 :duration '((= ?duration 10))
                 :precondition '(
                                 (over all (full-grasp-tablecloth ?robotA))
                                 (over all (full-grasp-tablecloth ?robotB))
                                 (at start (not (tablecloth-set)))
                                 )
                 :effect '(
                           (at end (tablecloth-set))
                           (at end (not (full-grasp-tablecloth ?robotA)))
                           (at end (not (full-grasp-tablecloth ?robotB)))
                           (at end (not (grasping-tablecloth ?robotA)))
                           (at end (not (grasping-tablecloth ?robotB)))
                           ))
       ))

(dolist (ac action-lst)
  (send *domain* :add :action ac))

(send *problem* :initial-condition
      '(
        (not (grasping-tablecloth robot1))
        (not (grasping-tablecloth robot2))
        (not (full-grasp-tablecloth robot1))
        (not (full-grasp-tablecloth robot2))
        (not (tablecloth-set))
        ))

(send *problem* :goal-condition
      '(
        (tablecloth-set)
        ))

(warn "~%")
(warn ";(demo)~%")
;(demo)
