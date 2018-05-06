(define (domain complex-football)
(:predicates
	 (at-robby ?c) (at-ball ?b ?c) (connected ?c1 ?c2) (ball ?b) (left-foot ?f) (right-foot ?f) (not-up ?f))

(:action move-left-leg
 :parameters (?c1 ?c2)
 :precondition
	(and (at-robby ?c1)  (connected ?c1 ?c2))
 :effect
	(and (at-robby ?c2) (not (at-robby ?c1))))

(:action move-right-leg
	:parameters (?c1 ?c2)
  :precondition
 	(and (at-robby ?c1)  (connected ?c1 ?c2))
  :effect
 	(and (at-robby ?c2) (not (at-robby ?c1))))

(:action lift-left
 :parameters (?f)
 :precondition
	(and (left-foot ?f) (not-up ?f))
 :effect
	(and (not(not-up ?f))))

(:action lift-right
 :parameters (?f)
 :precondition
	(and (right-foot ?f) (not-up ?f))
 :effect
	(and (not(not-up ?f))))


(:action kick-left
	:parameters (?b ?c1 ?c2)
	:precondition
  	(and (ball ?b)(at-ball ?b ?c1) (at-robby ?c1) (connected ?c1 ?c2))
 :effect
	  (and (at-ball ?b ?c2) (not (at-ball ?b ?c1)))
		)

(:action kick-right
	:parameters (?b ?c1 ?c2)
	:precondition
  	(and (ball ?b) (at-ball ?b ?c1) (at-robby ?c1) (connected ?c1 ?c2))
 :effect
	  (and (at-ball ?b ?c2) (not (at-ball ?b ?c1)))
	)
)
