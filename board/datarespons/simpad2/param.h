#ifndef PARAM_H
#define PARAM_H


#define MAX_PARAMS 254

struct param {

    unsigned char *data;
    unsigned char *orig;
    int size;
    int usage;

    int count;
    int max_params;
    char **param;
};


int param_init(struct param *e, const char *from, int size);

int param_add(struct param *e, const char *param);
int param_update(struct param *e, int index, const char *param);
int param_find(struct param *e, const char *key);

int param_split(const char *param, char **key, char **data);

int param_delete(struct param *e, int index);
int param_set(struct param *e, const char *param);

int param_parse(struct param *e);
int param_generate(struct param *e);

int param_write(struct param *e, const char *where);

int param_check_key(const char *key);
int param_check_data(const char *data);


#endif  // PARAM_H
