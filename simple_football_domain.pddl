(define (domain simple-football)
(:predicates
	 (at-robby ?c) (at-ball ?b ?c) (connected ?c1 ?c2) (ball ?b))

(:action move
 :parameters (?c1 ?c2)
 :precondition
	(and (at-robby ?c1)  (connected ?c1 ?c2))
 :effect
	(and (at-robby ?c2) (not (at-robby ?c1))))


(:action kick
	:parameters (?b ?c1 ?c2)
	:precondition
  	(and (at-ball ?b ?c1) (at-robby ?c1) (connected ?c1 ?c2) (ball ?b))
 :effect
	  (and (at-ball ?b ?c2) (not (at-ball ?b ?c1))))

)


