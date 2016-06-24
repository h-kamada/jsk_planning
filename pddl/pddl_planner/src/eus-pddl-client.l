(require :eus-pddl "package://pddl_planner/src/eus-pddl.l")

;;
;; PDDL domain and problem should be define before using this program.
;;

;;
;; ROS
;;
(ros::roseus "pddl_client")

(setq *action-client*
      (instance ros::simple-action-client :init
                "pddl_planner" pddl_msgs::PDDLPlannerAction))

(defun pddl-planning (domain problem &key (client *action-client*) (debug))
  (let (plan ret)
    (warning-message 1 "waiting pddl_planner~%")
    (send client :wait-for-server)
    (warning-message 1 "get actionlib server~%")
    (let ((goal (instance pddl_msgs::PDDLPlannerActionGoal :init)))
      (send goal :goal :domain (send domain :ros-message))
      (send goal :goal :problem (send problem :ros-message))
      (send client :send-goal goal)
      (warning-message 1 "sent goal~%"))

    (warning-message 1 "waiting goal...~%")
    (send client :wait-for-result)

    (when (send (send client :get-result) :sequence)
      (setq *sequence* (send (send client :get-result) :sequence))
      (cond
       ((send (send client :get-result) :use_durative_action) ;; use durative action
        (setq plan (mapcar #'(lambda (x)
                               (cons (read-from-string (send x :start_time))
                                     (cons (read-from-string (send x :action_duration))
                                           (cons (read-from-string (send x :action))
                                                 (mapcar #'(lambda (y) (read-from-string y))
                                                         (send x :args))))))
                               (send (send client :get-result) :sequence)))
        ;; plan = (list * * * (20.0 move robot1 init-pos cart-pos))
        )
       (t ;; normal action
        (setq plan (mapcar #'(lambda (x)
                               (cons (read-from-string (remove #\( (send x :action)))
                                     (mapcar #'(lambda (y) (read-from-string (remove #\) y)))
                                             (send x :args))))
                           (send (send client :get-result) :sequence)))
        ))
      ;; :action (MOVE  :args ("DISK0" "DISK1" "POLE2)")
      ;; :plan (move disk0 disk1 pole2) (move disk1 disk2 pole1) (move disk0 pole2 disk1) (move disk2 pole0 pole2) (move disk0 disk1 pole0) (move disk1 pole1 disk2) (move disk0 pole0 disk1)))
      (when debug
        (pprint (car (send (send client :get-result) :data))))
      (setq ret (append ret (list (cons :plan plan)))))
    ret))

(defun solve-pddl-planning (domain problem &key (client *action-client*) (debug)
                                   (optimize-state nil))
  (let* ((result (pddl-planning domain problem :client client :debug debug))
         (state (instance pddl-state :init :domain domain :initial-state (send problem :initial-condition) :actions (cdr (assoc :plan result)) :objects (send problem :objects)))
         (step-state (send state :generate-states)))

    (ros::ros-info "[solve-pddl] plan:~A" (cdr (assoc :plan result)))
    (when optimize-state
      (let ((intersect-condition (reduce #'(lambda(x y) (intersection x y :test #'equal)) step-state)))
        (setq step-state (mapcar #'(lambda(x) (set-differece x intersect-condition)) step-state))))

    (ros::ros-info "step-state:~A" step-state)
    (push (cons :step-state step-state) result)
    (push (cons :state state) result)
    result))

(provide :eus-pddl-client)
