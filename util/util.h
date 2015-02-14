

#define util_assert(t, i) if(!(t))util_assert_handler(__FILE__, __LINE__, i)
#define util_zalloc(sz) ((void *) os_zalloc((sz)))
#define util_free(p) os_free((p))

void util_assert_handler(char *file, unsigned line, char *info);
char *util_string_split(const char *in_str, char **list, char sep, int max_list_length);
char * util_make_sub_topic(const char *rootTopic, char *subTopic);
char * util_strdup(const char *s);
char * util_strndup(const char *s, int len);
bool util_match_stringi(const char *s1, const char *s2, unsigned len);
bool util_parse_command_int(const char *str, const char *command,  int *val);
int util_atoi(const char *num);




