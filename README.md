# Pioneer-AVIC-F840BT-IR-remote-controller
ATTINY2313 IR decoder for Pioneer AVIC wired remote input.
For better stability use external crystal 8MHz.

Pioneer AVIC-F840BT / F940BT wired remote

Resistor  Voltage   Command                                     Command with Shift
1200	    0,266   	SOURCE / 2 sec OFF	                        PHONE MENU
3300	    0,746   	ATT SOUND	
5600	    1,115   	DISPLAY / SONG TAG INFO     	              PHONE OFF
8200	    1,400   	TUNE UP / NEXT TRACK / LEFT	PRESET          UP / FOLDER UP / UP
12000	    1,698   	TUNE DOWN / PREVIUS TRACK / RIGHT	PRESET    DOWN / FOLDER DOWN / DOWN
15000	    1,878   	VOLUME +	
24000	    2,170   	VOLUME -	
68000	    2,819   	BAND / ESCAPE               	              MUTE

AVIC internal resistor 12000 on 3.3V.

Pioneer remote controller QXE1047 CXC8885 CXE3669 QXA3196.

Button	    Address	!Address	Command	!Command
VOLUME -	  AD  	  52      	0B     	F4
VOLUME +	  AD	    52	      0A    	F5
BAND / ESC	AD	    52      	12    	ED
MUTE	      AF	    50      	30  	  CF
FUNCTION  	AF	    50      	67  	  98
AUDIO	      AD	    52      	0D  	  F2
SRC	        AD	    52	      1A  	  E5
PAUSE	      AD	    52	      58   	  A7
DISPLAY	    AF	    50	      6D   	  92
LEFT	      AD	    52	      42     	BD
RIGHT	      AD	    52	      43  	  BC
UP	        AD	    52	      40  	  BF
DOWN	      AD	    52	      41  	  BE
ENTER	      AF	    50	      20  	  DF

