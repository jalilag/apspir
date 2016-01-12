#ifndef _PROFILE_H
#define _PROFILE_H

typedef struct PROF PROF;
struct PROF {
    int index;
    char* id;
    char* title;
};

char* profile_get_id_with_index(int index);
char* profile_get_title_with_index(int index);
int profile_get_index_with_id(char* id);
char* profile_get_filename_with_index(int index);

#endif
