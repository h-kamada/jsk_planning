

(defun make-relation-graph (result &key verbose)
  (let ((relation-lst
         (cond
          (verbose
           (send (cdr (assoc :state result)) :all-relationship))
          (t
           (send (cdr (assoc :state result)) :relationship))))
        (initial-node (instance arced-node :init :name (format nil "INITIAL")))
        (start-actions
         (remove-if #'null
                      (mapcar #'(lambda (x)
                                  (when (member 'start x) x))
                              (assoc :plan result))))
        (gr (instance pddl-graph :init)))
    (ros::ros-info "[length] relation-lst:~A start-actions:~A" (length relation-lst) (length start-actions))
    (dolist (sac start-actions)
      (unless (find-if-not #'null
                           (mapcar #'(lambda (x)
                                       (cond ((equal (send x :name) (format nil "~A ~A" (car sac) (caddr sac))) x)
                                             (t nil)))
                                   (send gr :nodes)))
        (send gr :add-node
              (instance arced-node :init :name
                        (format nil "~A ~A" (car sac) (caddr sac))))))
    ;; add INITIAL
    (let (first-node)
      (send gr :add-node initial-node)
      (setq first-node
            (find-if-not #'null
                         (mapcar #'(lambda (x)
                                     (cond ((equal (send x :name) (format nil "~A ~A"
                                                                          (car (car start-actions))
                                                                          (caddr (car start-actions)))) x)
                                           (t nil)))
                                 (send gr :nodes))))
      (send gr :add-arc-from-to initial-node first-node (format nil "")))
    (dolist (rl relation-lst)
      (let (from-node to-node)
        (setq to-node
              (find-if-not #'null
                           (mapcar #'(lambda (x)
                                       (cond ((equal (send x :name) (format nil "~A ~A"
                                                                            (car (car rl))
                                                                            (caddr (car rl)))) x)
                                             (t nil)))
                                   (send gr :nodes))))
        (let ((reverse-lst (member to-node (send gr :nodes))))
          (setq from-node
                (find-if-not #'null
                             (mapcar #'(lambda (x)
                                         (cond ((substringp (format nil "~A" (caddr (cadr rl))) (send x :name)) x)
                                                         (t nil)))
                                     reverse-lst)))
          )
        (when (and from-node to-node)
          (send gr :add-arc-from-to from-node to-node (format nil "~A" (cddr rl))))
        ))
    ;; link start nodes to initial-node
    (let ((tmp-nodes (send gr :nodes)))
      (dolist (nd (send gr :nodes))
        (dolist (aarc (send nd :arc-list))
          (setq tmp-nodes (remove (send aarc :to) tmp-nodes :count 1))))
      (setq tmp-nodes (remove initial-node tmp-nodes :count 1))
      (dolist (tn tmp-nodes)
        (send gr :add-arc-from-to initial-node tn (format nil ""))))
    gr
    ))

(defun write-to-dot (gr filename &key distinguish-robot (robot1-name "robot1") (robot2-name "robot2"))
  (let ((node-alist
         (mapcar #'(lambda (n)
                     (cons n (string (gensym))))
                 (send gr :nodes)))
        robot1-node-lst
        robot2-node-lst
        r1r2-node-lst
        )
    (labels ((graphviz-node-name
              (n)
              (cdr (assoc n node-alist))))
      (with-open-file
       (f filename :direction :output)
       (format f "digraph ~A {~%" "actionrelation")
       (when distinguish-robot
         ;; (format f "subgraph cluster_0 {~%label = \"robot 1\";~%color=blue;~%")
         (dolist (nd (send gr :nodes))
           (when (substringp robot1-name (send nd :name))
             ;; (format f "~A;~%" (graphviz-node-name nd))
             (setq robot1-node-lst (append robot1-node-lst (list (graphviz-node-name nd))))
             ))
         ;; (format f "}~%")
         ;; (format f "subgraph cluster_1 {~%label = \"robot 2\";~%color=blue;~%")
         (dolist (nd (send gr :nodes))
           (when (substringp robot2-name (send nd :name))
             ;; (format f "~A;~%" (graphviz-node-name nd))
             (setq robot2-node-lst (append robot2-node-lst (list (graphviz-node-name nd))))
             ))
         ;; (format f "}~%")

         (dolist (nd robot1-node-lst)
           (when (member nd robot2-node-lst)
             (setq robot1-node-lst (remove nd robot1-node-lst))
             (setq robot2-node-lst (remove nd robot2-node-lst))
             (setq r1r2-node-lst (append r1r2-node-lst (list nd)))
             ))

         (format f "subgraph cluster_0 {~%label = \"robot 1\";~%color=blue;~%")
         (dolist (r-nd robot1-node-lst)
           (format f "~A;~%" r-nd)
           )
         (format f "}~%")
         (format f "subgraph cluster_1 {~%label = \"robot1&robot2\";~%color=blue;~%")
         (dolist (r-nd r1r2-node-lst)
           (format f "~A;~%" r-nd)
           )
         (format f "}~%")
         (format f "subgraph cluster_2 {~%label = \"robot 2\";~%color=blue;~%")
         (dolist (r-nd robot2-node-lst)
           (format f "~A;~%" r-nd)
           )
         (format f "}~%")
         )
       (dolist (nd (send gr :nodes))
         (format f "   ")
         (format f "~A [label = \"~A\"];~%"
                 (graphviz-node-name nd)
                 (send nd :name)))
       (let ((drawed-arc nil))
         (dolist (target-node (send gr :nodes))
           (let ((neighbors (send target-node :neighbor-action-alist)))
             (dolist (neighbor neighbors)
               (when
                   (or (not (eq distinguish-robot :inter-robot-only))
                       (not
                        (or
                         (subsetp (list (graphviz-node-name target-node)
                                        (graphviz-node-name (cdr neighbor)))
                                  robot1-node-lst)
                         (subsetp (list (graphviz-node-name target-node)
                                        (graphviz-node-name (cdr neighbor)))
                                  robot2-node-lst))))
                 (format f "   ")
                 (format f "~A -> ~A [label = \"~A\"];~%"
                         (graphviz-node-name target-node)
                         (graphviz-node-name (cdr neighbor)) (car neighbor))
                 ))))
         (when (eq distinguish-robot :inter-robot-only)
           (let (from-node to-node)
             (dolist (nlst robot1-node-lst)
               (setq from-node nlst)
               (when to-node
                 (format f "~A -> ~A;~%" from-node to-node))
               (setq to-node from-node)
               ))
           (let (from-node to-node)
             (dolist (nlst robot2-node-lst)
               (setq from-node nlst)
               (when to-node
                 (format f "~A -> ~A;~%" from-node to-node))
               (setq to-node from-node)
               )))
         (format f "}~%")
         t)))))

(defun replanning (from to replanning-initial-action)
  (let (states find-flag cnt tmp-cnt new-state old-pddl-state new-pddl-state merging-actions
               merging-pddl-state pddl-initial-condition error-time)
    (setq pddl-initial-condition (send *problem* :initial-condition))
    (setq old-pddl-state (cdr (assoc :state *result*)))
    (setq states (send old-pddl-state :states))
    (ros::ros-info "states: ~A" states)

    ;; 異常発生箇所をたどる
    (setq find-flag nil)
    (setq cnt 0)
    (dolist (st states)
      (dolist (sst st)
        (setq *s* sst)
        (ros::ros-info "sst: ~A" sst)
        (ros::ros-info "from: ~A" from)
        (ros::ros-info "equal: ~A" (equal from sst))
        (when (and (equal from sst) (null find-flag))
          (setq find-flag t)
          (setq tmp-cnt cnt)
          (ros::ros-info "cnt: ~A" cnt)
          )
        )
      (incf cnt)
      )
    (setq *states* states)
    (ros::ros-info "cnt: ~A" tmp-cnt)
    (ros::ros-info "key state: ~A" (nth tmp-cnt states))
    ;; error-time is used for replanning-action time
    (setq error-time (car (nth (- tmp-cnt 1) (send old-pddl-state :timeline-actions))))
    (ros::ros-info "error-time:~A" error-time)
    ;; (ros::ros-info "timeline-actions: ~A" (send (cdr (assoc :state *result*)) :timeline-actions))


    ;; the detected error condition(from) appeared in cnt condition(key-state)
    ;; "from"conditionから異常を発見したactionまでの流れを、"to"conditionをそこに入れた時にどうなるかを
    ;; pddl-stateを用いて追っていく
    (dolist (st (nth tmp-cnt states))
      (cond
       ((equal st from)
        (setq new-state (append new-state (list to))))
       (t
        (setq new-state (append new-state (list st))))))
    (ros::ros-info "new-state: ~A" new-state)
    (setq new-state-cnt tmp-cnt)
    (setq new-pddl-state
          (instance pddl-state :init :domain *domain* :initial-state new-state
                    :objects (send *problem* :objects)))
    (while t
      (ros::ros-info "cnt: ~A appling-action: ~A" new-state-cnt
                     (nth new-state-cnt (send old-pddl-state :timeline-actions)))
      (send new-pddl-state :add-action (nth new-state-cnt (send old-pddl-state :timeline-actions)))
      (when (equal (nth new-state-cnt (send old-pddl-state :timeline-actions)) replanning-initial-action)
        (return))
      (incf new-state-cnt)
      (unix:sleep 1)
      )
    (ros::ros-info "states: ~A" (send new-pddl-state :states))
    (ros::ros-info "actions: ~A" (send new-pddl-state :actions))
    (ros::ros-info "last state: ~A" (car (last (send new-pddl-state :states))))
    ;; last-state が、異常発見したactionまでを"to"conditionを使って再現した際の、
    ;; 異常発見actionまで実行した結果のstate状態を表している


    ;; last-stateを初期状態としてpddlを解かせてその結果を受け取る
    (send *problem* :initial-condition (car (last (send new-pddl-state :states))))
    (setq *replan-result* (solve-pddl-planning *domain* *problem*))


    ;; 異常検知までのactionをmerging-actionsとして定義
    (setq cnt 0)
    (while t
      (setq merging-actions
            (append merging-actions
                    (list (nth cnt (send old-pddl-state :timeline-actions)))))
      (when (equal (nth cnt (send old-pddl-state :timeline-actions)) replanning-initial-action)
        (return))
      (ros::ros-info "a:~A b:~A" (nth cnt (send old-pddl-state :timeline-actions)) replanning-initial-action)
      (incf cnt)
      )
    (ros::ros-info "merging-actions: ~A" merging-actions)



    ;; 異常発生後のactionについて、last-stateを初期状態として解かせた結果を元に
    ;; 1sだけずらしながらtmp-planとして定義
    (setq tmp-plan nil)
    (dolist (ac (cdr (assoc :plan *replan-result*)))
      (setq tmp-plan (append tmp-plan (list (list (+ (+ (car replanning-initial-action) (car ac)) 1)
                                                  (cadr ac)
                                                  (caddr ac)
                                                  ))))
      )
    (ros::ros-info "tmp-plan: ~A" tmp-plan)



    ;; replan actionを定義
    ;; replanを含め、出現するaction全てを含む新しいpddl-state インスタンスを作成
    (send *domain* :add :action
          (instance pddl-action :init
                    :use-durative-action t
                    :name "replanning-call"
                    :parameters '(())
                    :duration '((= ?duration 0))
                    :precondition `((at start ,from))
                    :effect `((at start ,to))))
    (setq *merging-pddl-state*
          (instance pddl-state :init :domain *domain* :initial-state pddl-initial-condition
                    :objects (send *problem* :objects)))
    (setq *tt* (append  merging-actions
                        (list (list error-time 'start '(replanning-call)))
                        (list (list error-time  'end '(replanning-call)))
                        tmp-plan
                        ))
    ;; (setq *tt* (append merging-actions
    ;;                    (list `(,(car replanning-initial-action) start (replanning-call)))
    ;;                    (list `(,(car replanning-initial-action) end (replanning-call)))
    ;;                    tmp-plan
    ;;                    ))
    (ros::ros-info "tt:~A" *tt*)
    (setq *tt* (set-duration-to-plan *tt*))
    (ros::ros-info "hoe")
    (send *merging-pddl-state* :actions *tt*)
    (send *merging-pddl-state* :generate-states)
    (send *merging-pddl-state* :generate-single-relationship)
    (send *merging-pddl-state* :generate-all-relationship)



    ;; 結果を受け取ってgraph化
    (setq *merging-result* (list (cons :plan (send *merging-pddl-state* :timeline-actions))))
    (push (cons :state *merging-pddl-state*) *merging-result*)
    (setq *merging-graph* (make-relation-graph *merging-result*))
    (setq *merging-all-graph* (make-relation-graph *merging-result* :verbose t))
    (write-to-dot *merging-graph* ".merged_normal.dot")
    (write-to-dot *merging-graph* ".merged_normal_separated.dot"
                  :distinguish-robot t :robot1-name "pr1012" :robot2-name "pr1040")
    (write-to-dot *merging-all-graph* ".merged_verbose.dot")
    (write-to-dot *merging-all-graph* ".merged_verbose_separated.dot"
                  :distinguish-robot t :robot1-name "pr1012" :robot2-name "pr1040")
    (write-to-dot *merging-all-graph* ".merged_inter_robot_relationship.dot"
                  :distinguish-robot :inter-robot-only  :robot1-name "pr1012" :robot2-name "pr1040")
    (unix:system (format nil "dot ~A -Tpdf -o ~A" ".merged_normal.dot" ".merged_normal.pdf"))
    (unix:system (format nil "dot ~A -Tpdf -o ~A" ".merged_normal_separated.dot" ".merged_normal_separated.pdf"))
    (unix:system (format nil "dot ~A -Tpdf -o ~A" ".merged_verbose.dot" ".merged_verbose.pdf"))
    (unix:system (format nil "dot ~A -Tpdf -o ~A" ".merged_verbose_separated.dot" ".merged_verbose_separated.pdf"))
    (unix:system (format nil "dot ~A -Tpdf -o ~A" ".merged_inter_robot_relationship.dot" ".merged_inter_robot_relationship.pdf"))
    ;; (piped-fork "gnome-open .merged_normal.pdf")
    ;; (piped-fork "gnome-open .merged_normal_separated.pdf")
    (piped-fork "gnome-open .merged_verbose.pdf")
    (piped-fork "gnome-open .merged_verbose_separated.pdf")
    (piped-fork "gnome-open .merged_inter_robot_relationship.pdf")
    ))

(defun set-duration-to-plan (plan)
  (let (new-plan tmp-plan)
    (ros::ros-info "plan:~A" plan)
    (dolist (pp plan)
      (when (equal (cadr pp) 'start)
        (ros::ros-info "pp:~A" pp)
        (setq action (find-if #'(lambda (x)
                                  (equal (caaddr pp) (read-from-string (send x :name))))
                              (send *domain* :action)))
        (ros::ros-info "action:~A" action)
        (setq duration (car (cddar (send action :duration))))
        (ros::ros-info "duration:~A" duration)
        (setq tmp-plan (list (flatten (list (car pp) duration (caddr pp)))))
        (setq new-plan (append new-plan tmp-plan))
        )
      )
    (ros::ros-info "new-plan: ~A" new-plan)
    new-plan
    ))
;; (20.0 10.0 set-tablecloth pr1012 pr1040))


(defun re ()
  (demo :robot1-name "pr1012" :robot2-name "pr1040")
  (unix:sleep 2)
  (ros::ros-info "hoge:~A" (caddr (cadr (assoc :plan *result*))))
  (cond
   ((member 'pr1012 (caddr (cadr (assoc :plan *result*))))
    (ros::ros-info "Starting from pr1012")
    (replanning '(full-grasp-tablecloth pr1012) '(not (full-grasp-tablecloth pr1012)) '(20.0 end (grasp-tablecloth pr1040 pr1012)))
    )
   (t
    (ros::ros-error "Not starting from pr1012. Retry.")
    (re)
    )
   ))

(defun hoge ()
  (send *problem* :initial-condition
        '((grasping-tablecloth pr1012) (full-grasp-tablecloth pr1040) (grasping-tablecloth pr1040)))
  (demo)
  )

(defun demo (&key no-output (robot1-name "robot1") (robot2-name "robot2"))
  (setq *result* (solve-pddl-planning *domain* *problem*))
  (cond
   ((equal (send (cdr (assoc :state *result*)) :generate-single-relationship) 'error)
    (ros::ros-error "LPG planner error!"))
   (t
    (send (cdr (assoc :state *result*)) :generate-all-relationship)
    (setq *graph* (make-relation-graph *result*))
    (setq *all-graph* (make-relation-graph *result* :verbose t))
    (unless no-output
      (write-to-dot *graph* ".normal.dot")
      (write-to-dot *graph* ".normal_separated.dot"
                    :distinguish-robot t :robot1-name robot1-name :robot2-name robot2-name)
      (write-to-dot *all-graph* ".verbose.dot")
      (write-to-dot *all-graph* ".verbose_separated.dot"
                    :distinguish-robot t :robot1-name robot1-name :robot2-name robot2-name)
      (write-to-dot *all-graph* ".inter_robot_relationship.dot"
                    :distinguish-robot :inter-robot-only
                    :robot1-name robot1-name :robot2-name robot2-name)
      (unix:system (format nil "dot ~A -Tpdf -o ~A" ".normal.dot" ".normal.pdf"))
      (unix:system (format nil "dot ~A -Tpdf -o ~A" ".normal_separated.dot" ".normal_separated.pdf"))
      (unix:system (format nil "dot ~A -Tpdf -o ~A" ".verbose.dot" ".verbose.pdf"))
      (unix:system (format nil "dot ~A -Tpdf -o ~A" ".verbose_separated.dot" ".verbose_separated.pdf"))
      (unix:system (format nil "dot ~A -Tpdf -o ~A" ".inter_robot_relationship.dot" ".inter_robot_relationship.pdf"))
      ;; (piped-fork "gnome-open .normal.pdf")
      ;; (piped-fork "gnome-open .normal_separated.pdf")
      (piped-fork "gnome-open .verbose.pdf")
      (piped-fork "gnome-open .verbose_separated.pdf")
      (piped-fork "gnome-open .inter_robot_relationship.pdf")
      )))
  )

