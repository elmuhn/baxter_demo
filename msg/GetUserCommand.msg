string command

string MOVE_TO_IDLE_MODE = move_to_idle 				
string MOVE_TO_HOME_POSITION = move_to_home	
string RUN_DEMO = run_demo		
string RUN_DEMO1 = run_demo1
string RUN_DEMO2 = run_demo2						
string CANCEL_DEMO = cancel_demo						
string RESTART_DEMO = restart_demo	
string SHUTDOWN = shutdown				

#'move to idle' 	   --> Gets published if the user do nothing within 5 minutes
#'move to home' --> Gets published if the user interrupt in idle mode or cancel a demo
#'run demo' 	   --> Gets pusblished when the user wants to run a demo
#'cancel demo'	   --> Gets pusblished when the user wants to cancel a demo
#'restart demo'    --> Gets pusblished when the user wants to restart a demo