#include "watchdog/watchdog.h"

void watchdog_init(watchdog_t *wd, int n, int max_missed)
{
    if (n > WD_MAX_TASKS)
    {
        n = WD_MAX_TASKS;
    }
    wd->n = n;
    wd->max_missed = max_missed;
    wd->primed = 0;
    for (int i = 0; i < WD_MAX_TASKS; ++i)
    {
        wd->prev[i] = 0u;
        wd->missed[i] = 0;
    }
}

int watchdog_check(watchdog_t *wd, const uint32_t *current)
{
    if (!wd->primed)
    {
        for (int i = 0; i < wd->n; ++i)
        {
            wd->prev[i] = current[i];
        }
        wd->primed = 1;
        return -1;
    }

    int faulted = -1;
    for (int i = 0; i < wd->n; ++i)
    {
        if (current[i] == wd->prev[i])
        {
            if (++wd->missed[i] >= wd->max_missed)
            {
                if (faulted < 0)
                {
                    faulted = i;
                }
            }
        }
        else
        {
            wd->missed[i] = 0;
        }
        wd->prev[i] = current[i];
    }

    return faulted;
}
