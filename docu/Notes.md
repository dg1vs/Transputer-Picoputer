
# Adding Reste 

The important observations are:
* mainloop() normally does not return, so it must explicitly notice Reset.
+ The bootstrap loop currently runs only once.
+ server_init() cannot be called again because it would erase the stored PIO/SM assignments.
+ The LinkIn and LinkOut program offsets must be preserved so their state machines can be restarted.
* Emulated RAM should initially remain unchanged.