
/* These are temporary, the correct numbers should be inserted */
#define IN_WIDTH		     		     4096
#define IN_HEIGHT 		 		     	 2160
#define OUT_WIDTH		     		     360//4096/*1280*/
#define OUT_HEIGHT 		 		    	 8//240//2160/*8*/
#define M_BLOCK_SIZE		      	     1024
#define	M_NOF_BLOCKS			         1	/* defines the nof_active_streams */
#define P_BLOCK_SIZE		      	     1024
#define	P_NOF_BLOCKS			         1
#define MAX_SEND_TOKENS 	             3
#define MAX_RECV_TOKENS	                 20
#define NOF_INPUT_PINS                	 1	/* Careful on its use, code nees to change */
#define TOP_OFFSET                       10
#define LEFT_OFFSET                      10
#define BOTTOM_OFFSET                    470
#define RIGHT_OFFSET                     630
#define NOF_OUTPUT_PINS                  1  /* Careful on its use, code nees to change */
#define WATERMARK                        10
#define NOF_FRAMES						 5/*17*/
#define MMIO_BA                          16384
#define PAGE_TABLE_BA                    32768
#define FW_ADDR                          2048
#define SIZEOF_PIXEL	   				 3
#define SIZEOF_IN_BUF	   				 32768

/* Following are non-modifiable */
#define ENABLE           			     1
#define DISABLE             		     0
#define NUM_SEND_Q              	     M_NOF_BLOCKS
#define NUM_RECV_Q          	         1
