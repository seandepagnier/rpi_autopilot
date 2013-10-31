enum {READ, WRITE};
FILE *config_open(int op, const char *config_name);
int config_string(int op, FILE *f, const char *name, char *str, int maxlen);
void config_float_vector(int op, FILE *f, const char *name, float *value, int c);
void config_float(int op, FILE *f, const char *name, float *value);

