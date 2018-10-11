#include <common.h>
#include <malloc.h>

#include "param.h"
#include <u-boot/crc.h>
#include <errno.h>


int param_init(struct param *e, const char *from, int size)
{
    e->size = size;
    e->data = calloc(sizeof(char),size);
    e->orig = calloc(sizeof(char),size);
    e->max_params = size/4;
    e->count = 0;
    e->usage = 4;
    e->param = calloc(sizeof(char*),e->max_params);

    if (!e->data || !e->orig || !e->param)
    {
        printf( "%s: Internal error: Could not allocate memory. Aborting\n", __func__);
        return 255;
    }
    memcpy(e->data, from, size);
    memcpy(e->orig, from, size);
    return 0;
}

int param_write(struct param *e, const char *where)
{
	memcpy((void*)where, e->data, e->size);
	return 0;
}

int param_add(struct param *e, const char *param)
{
    if (e->count == e->max_params)
    {
        printf( "%s: Internal error: No free slot for parameter\n", __func__);
        return -ENOMEM;
    }
    int len = strlen(param)+1;
    if (len == 1)
    {
        printf( "%s: Internal error: Tried to add 0-length parameter\n", __func__);
        return -EINVAL;
    }
    if (e->usage+len > e->size)
    {
        printf( "%s: Not enough free space to add parameter '%s'\n", __func__, param);
        return -ENOMEM;
    }

    // Add the parameter to the list
    //if (verbose)
    //    printf("%s: Adding parameter '%s'\n", me, param);
    e->param[e->count++] = strdup(param);
    e->usage += len;
    return 0;
}



int param_update(struct param *e, int index, const char *param)
{
    if (index<0 || index>=e->count)
    {
        printf( "%s: Internal error: Tried to update index out of range\n", __func__);
        return 1;
    }
    int newl = strlen(param)+1;
    int oldl = strlen(e->param[index])+1;
    if (newl == 1)
    {
        printf( "%s: Internal error: Tried to update 0-length parameter\n",__func__);
        return 1;
    }

    // Update the parameter
    free(e->param[index]);
    e->param[index] = strdup(param);
    e->usage += newl-oldl;
    return 0;
}



int param_find(struct param *e, const char *key)
{
    int i;
    int len = strlen(key);
    for(i=0; i<e->count; i++)
    {
        const char *param = e->param[i];

        // Find the location of the '='
        const char *sep = strchr(param,'=');

        // Find the length of the key. If no '=' is found, use the full parameter.
        int klen = (sep) ? sep-param : (signed)strlen(param);
        if (klen != len)
            continue;

        // Compare the given key with the parameter key
        if (!strncmp(key, param, klen))
        {
            return i;
        }
    }
    printf( "%s: Did not find any key '%s'\n", __func__,key);
    return -1;
}



int param_delete(struct param *e, int index)
{
    if (index<0 || index>=e->count)
    {
        printf( "%s: Internal error: Tried to delete index out of range\n", __func__);
        return 1;
    }
    if (e->param[index] == NULL)
        return 0;
    int len = strlen(e->param[index])+1;

    // Free the parameter
    free(e->param[index]);
    e->count--;
    e->usage -= len;

    // Shift all parameters up to fill the gap left by the removed param
    while(index < e->count)
    {
        e->param[index] = e->param[index+1];
        index++;
    }
    e->param[index] = NULL;
    return 0;
}



int param_split(const char *param, char **key, char **data)
{
    char *s = strdup(param);
    if ( NULL == s)
    	return -ENOMEM;

    char *sep = strchr(s,'=');
    *key = s;
    if (!sep)
    {
        *data = NULL;
    }
    else
    {
        *sep=0;
        *data = sep+1;
    }
    return 0;
}


int param_set(struct param *e, const char *param)
{
    char *key, *data;
    param_split(param, &key, &data);
    int index = param_find(e,key);
    free(key);
    if (index >= 0)
    {
        return param_update(e,index,param);
    }
    return param_add(e,param);
}



int param_parse(struct param *e)
{
    //
    // Verify CRC
    //
    uint8_t *p = (uint8_t*)(e->data);
    uint32_t crc = (p[3]<<24) | (p[2]<<16) | (p[1]<<8) | (p[0]);
    uint32_t checksum = crc32(0, &p[4], e->size-4);
    if (crc != checksum)
    {
        printf("%s: Incorrect nvram checksum\n", __func__);
        return -EINVAL;
    }
    else
    {
    	printf("%s: CRC checksum correct (%08X)\n", __func__, crc);
    }


    unsigned char *src = &(e->data[4]);
    int remain = e->size-4;
    while(remain > 0)
    {
        // Get parameter length (limit to end of buffer)
        int len = strnlen((char*)src, remain);

        // Done if no more strings or we see 0xFF
        if (len == 0 || *src == 0xFF)
            break;

        // The last parameter does not have a trailing 0-terminator. Error?
        if (len == remain)
        {
            printf( "%s: Invalid nvram data. Missing 0-termination on last parameter\n", __func__);
            return -EINVAL;
        }

        // Add the parameter
        printf( "%s:   Parsing '%s'\n", __func__, src);
        int r = param_add(e, (char*)src);
        if (r)
            return r;

        remain -= len+1;
        src += len+1;
    }

    printf( "%s: Parsed %d parameters\n", __func__, e->count);

    return 0;
}


int param_generate(struct param *e)
{
    unsigned char *dest = &(e->data[4]);
    int remain = e->size-4;
    int count=e->count;
    int i = 0;

    printf("%s: Going to generate %d parameters\n", __func__, count);

    while(i < count)
    {
        char *param = e->param[i];
        int len = strlen(param);

        printf("%s:   Generating '%s'\n", __func__, param);

        if (remain < len+1)
        {
            printf( "%s: No space left in nvram to write parameters\n", __func__);
            return -1;
        }

        // Copy key and data to buffer and set '=' as the separator between them
        strcpy((char*)dest, param);
        dest += len+1;
        remain -= len+1;
        i++;
    }

    // Erase remainder of buffer
    while(remain-- > 0)
    {
        *dest++=0xFF;
    }


    //
    // Generate new CRC
    //
    uint8_t *p = (uint8_t*)(e->data);
    uint32_t checksum = crc32(0, &p[4], e->size-4);
    p[0] = checksum & 0xff;
    p[1] = (checksum >> 8) & 0xff;
    p[2] = (checksum >> 16) & 0xff;
    p[3] = (checksum >> 24) & 0xff;

    printf( "%s: Calculated new checksum (%08X)\n", __func__, checksum);

    return 0;
}

int param_check_key(const char *key)
{
    int l = strlen(key);
    if (l==0)
        return 1;

    int i;
    for (i=0; i<l; i++)
    {
        char c = key[i];
        if (c>='0' && c<='9')
            continue;
        if (c>='A' && c<='Z')
            continue;
        if (c=='_' || c=='-')
            continue;
        return 1;
    }
    return 0;
}


int param_check_data(const char *data)
{
    int l = strlen(data);
    int i;
    for (i=0; i<l; i++)
    {
        char c = data[i];
        if (c<' ' || c>=0x7F)
            return 1;
    }
    return 0;
}
