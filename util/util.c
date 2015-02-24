#include "ets_sys.h"
#include "os_type.h"
#include "mem.h"
#include "osapi.h"
#include "user_interface.h"
#include "util.h"

/*
 * Restart system
 */
 
void ICACHE_FLASH_ATTR util_restart(void)
{
	os_printf("\r\nRestarting...\r\n");
	ets_delay_us(250000);
	system_restart();
}


/*
 * Assert error handler
 */
 

void ICACHE_FLASH_ATTR util_assert_handler(void)
{	
	util_restart();
}



/*
 * util_str_realloc - return a new string pointer in a larger buffer
 */
 
char * ICACHE_FLASH_ATTR util_str_realloc(const char *p, size_t new_len)
{
	size_t oldlen = strlen(p) + 1;
	util_assert(new_len > oldlen + 1, "New string smaller than original string. new length: %d, old length: %d", new_len, oldlen); 
	char *n =  util_zalloc(new_len);
	os_strcpy(n, p);
	util_free(p);
	return n;
}



/*
 * strdup function missing from os_* calls.
 */
 
char * ICACHE_FLASH_ATTR util_strdup(const char *s)
{
	char *p =  util_zalloc(os_strlen(s) + 1);
	os_strcpy(p, s);
	return p;
}



/*
 * strndup function missing from os_* calls.
 */
 
char * ICACHE_FLASH_ATTR util_strndup(const char *s, int len)
{
	char *p =  util_zalloc(len + 1);
	os_strncpy(p, s, len);
	p[len] = 0;
	return p;
}
 




/*
* Split a string into substrings and return the result as a list of pointers
* List of pointers should be 1 more than what is required. 
* max_list_length includes the NULL marker.
* 
* Returned char pointer must be freed when no longer required.
* 
* Does not remove white space
*/
char * ICACHE_FLASH_ATTR util_string_split(const char *in_str, char **list, char sep, int max_list_length)
{
	int i, j;
	char *str = util_strdup(in_str);
	
	for(i = 0, j = 0; i < max_list_length - 1; i++){

		// Skip leading seps
		while(sep == str[j]){
			if(!str[j])
				break;
			j++;
		}
		
		// Test for empty entry
		if(!str[j])
			break;
			
		list[i] = str + j; // Save the beginning of the string
		while(str[j] && (str[j] != sep))
			j++;
		// Test for end of string
		if(!str[j]){
			i++;
			break;
		}
		str[j] = 0; // Terminate substring
		j++;
	}
	list[i] = NULL; // Terminate end of list
	return str;
}

/*
 * Allocate and make a subtopic string 
 */

char * ICACHE_FLASH_ATTR util_make_sub_topic(const char *rootTopic, char *subTopic)
{
	char *r = (char *) os_zalloc(strlen(rootTopic) + 
	2 + strlen(subTopic));
	os_strcpy(r, rootTopic);
	os_strcat(r, "/");
	os_strcat(r, subTopic);
	return r;
}

/*
 * Match a string ignoring case
 */
 
bool ICACHE_FLASH_ATTR util_match_stringi(const char *s1, const char *s2, unsigned len)
{
	unsigned i;
	
	while(*s1 && *s2 && i < len){
		if(tolower(*s1++) != tolower(*s2++))
			break;
		i++;
	}
	if(i == len)
		return TRUE;
	else
		return FALSE;		
}


/*
 * Parse a command with a single numeric parameter
 */
 
bool ICACHE_FLASH_ATTR util_parse_command_int(const char *str, const char *command,  int *val)
{
	unsigned len = os_strlen(command);
	
	if(!util_match_stringi(str, command, os_strlen(command)))
		return FALSE;
	if ((!str[len]) || (str[len] != ':'))
		return FALSE;
	
	*val = atoi(str + len + 1);
	
	return TRUE;
}

/*
 * Parse a command with an optional string parameter
 * Sets val to NULL if no parameter found, else returns a pointer to the
 * value
 */
 
bool ICACHE_FLASH_ATTR util_parse_command_qstring(char *str, const char *command,  char **val)
{
	unsigned len = os_strlen(command);
	
	if(!util_match_stringi(str, command, os_strlen(command)))
		return FALSE;
	if ((!str[len]) || (str[len] != ':') || (!str[len + 1])){
		*val = NULL;
		return TRUE;
	}
	
	*val = str + len + 1;
	return TRUE;
}


