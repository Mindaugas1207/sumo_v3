#ifndef DISPLAY_H
#define DISPLAY_H

void display_init();
void handle_display();
void menuNavigate(int direction);
void menuToggle();
void menuSelect();
void motion_reset(void);

extern Graphics graphics;

enum CombatState
{
    STATE_SEARCH,
    STATE_APPROACH,
    STATE_PREPARE_ATTACK,
    STATE_ATTACK
};

extern CombatState combat_state;

#endif // DISPLAY_H
