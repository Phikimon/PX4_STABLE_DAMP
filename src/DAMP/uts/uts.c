#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>

__EXPORT int uts_main(int argc, char *argv[]);

int uts_main(int argc, char *argv[])
{
	int sub_fd = orb_subscribe(ORB_ID(actuator_controls));
	struct actuator_controls_s controls;
	
	px4_pollfd_struct_t fds[] = {
        { .fd = sub_fd,   .events = POLLIN }
			                 };

		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
	if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

			} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}
			
		else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(actuator_controls), sub_fd, &controls);
				
				for (int i = 0; i < 8; i++)
				{
					PX4_INFO("channel %d : \t%8.4f", i, (double)controls.control[i]);
					
				}
			}
			
				
		}
		return 0;
}
			
	

    


	
