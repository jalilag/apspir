#ifndef _STRTOOLS_H
#define _STRTOOLS_H
#include "canfestival.h"
#include <gtk/gtk.h>


char* strtools_concat(char* s1, char* s2, ...); // Concaténation de chaine
char** strtools_split(char* chaine,const char* delim,int vide); // Split de chaine suivant un délimiteur
int strtools_num_char_in_file(FILE* file, const char* delim); // Compte le nombre de caractere delim dans un fichier
int strtools_num_char_in_str(char* strRef, char* str2find); // Nombre d'occurence dans une chaine
void strtools_print_data(void* data,UNS8 type); // Show data on shell
gchar* strtools_gnum2str(void *data,UNS8 type); // Convert to str for GTK
gchar* strtools_gnum2str_all_decimal(void * data, UNS8 type);
char* strtools_hex2bin(UNS16* data,UNS8 type); // Hexa -> binaire
void strtools_inv_str(char *chaine); // Inversion de chaine
int strtools_build_file(char* fname, char* str2write);
char* strtools_replace_char(char* chaine, const char c2find,const char c2replace);
#endif // _STRING_FUNC_H
