#include "profile.h"
#include "SpirallingControl.h"
#include <glib.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

extern PROF profiles[PROFILE_NUMBER];
/**
* ID profile en fonction index
**/
char* profile_get_id_with_index(int index) {
    if (index>=PROFILE_NUMBER) {
        printf("Erreur pas de profile trouvé, index trop grand : %d",index);
        exit(EXIT_FAILURE);
    }
    return profiles[index].id;
}
/**
* title profile en fonction index
**/
char* profile_get_title_with_index(int index) {
    if (index>=PROFILE_NUMBER) {
        printf("Erreur pas de profile trouvé, index trop grand : %d",index);
        exit(EXIT_FAILURE);
    }
    return profiles[index].title;
}
/**
* index profile en fonction id
**/
int profile_get_index_with_id(char* id) {
    int i = 0;
    for (i;i<PROFILE_NUMBER;i++)
        if (profiles[i].id == id) return i;

    printf("Erreur pas de profile trouvé, id inconnu : %s",id);
    exit(EXIT_FAILURE);
}

char* profile_get_filename_with_index(int index) {
    return g_strconcat("profile_",g_utf8_strdown(profiles[index].id,-1),"_config.txt",NULL);
}
