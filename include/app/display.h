#ifndef DISPLAY_H
#define DISPLAY_H

void display_init();
void handle_display();
void menuNavigate(int direction);
void menuToggle();
void menuSelect();
void motion_reset(void);

extern Graphics graphics;

#endif // DISPLAY_H
