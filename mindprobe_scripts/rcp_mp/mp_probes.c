
/*
 * mp_probes.c -- Mindprobe probe table
 */


#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#ifndef MP_EMBEDDED
#include <syslog.h>
#endif

#include "mp_internal.h"

#include "log/log.h"
#include "util/util.h"

#ifdef MP_PROBE_TABLE_SIZE
#define PROBE_TABLE_SIZE MP_PROBE_TABLE_SIZE
#else
#define PROBE_TABLE_SIZE 4096
#endif

#if defined(EMBEDDED) && !defined(MP_PROBE_MAX_NAME_LENGTH)
#define MP_PROBE_MAX_NAME_LENGTH 24
#endif

static struct mp_probe_table {
#ifdef MP_EMBEDDED
    char name[MP_PROBE_MAX_NAME_LENGTH];
#else
    const char *name;
#endif
    enum MP_probe_type type;
    unsigned len;
    void *data;
#ifndef MP_EMBEDDED
    // Embedded code does not use hooks at present, so we
    // spare it the memory footprint of 4 pointers per probe
    void (*init_hook)(int probe_id, void *arg);
    int (*enable_hook)(int probe_id, void *arg, int enabled);
    // if read_hook returns zero, the probe is not sent on this tick
    int  (*read_hook)(int probe_id, void *arg);
    void (*write_hook)(int probe_id, void *arg);
    void *hook_arg;
#endif
    struct {
	struct mp_probe_table *next_enabled;
	char  enabled;
    } client_info[MP_MAX_RT_CLIENTS];
} probe_table[PROBE_TABLE_SIZE];
int enabled_probe_list_clean[MP_MAX_RT_CLIENTS];
struct mp_probe_table *first_enabled_probe[MP_MAX_RT_CLIENTS];
static unsigned int num_probes = 0;

/* Probe type length for fixed-length types: */
static int __attribute__((__unused__)) probe_type_len(enum MP_probe_type type)
{
    int len;

    switch(type) {
    case MP_PROBE_TYPE_UINT8:
    case MP_PROBE_TYPE_INT8:
    case MP_PROBE_TYPE_BOOL:
	len = 1;
	break;
    case MP_PROBE_TYPE_UINT16:
    case MP_PROBE_TYPE_INT16:
	len = 2;
	break;
    case MP_PROBE_TYPE_UINT32:
    case MP_PROBE_TYPE_INT32:
    case MP_PROBE_TYPE_FLOAT:
	len = 4;
	break;
    case MP_PROBE_TYPE_UINT64:
    case MP_PROBE_TYPE_INT64:
    case MP_PROBE_TYPE_DOUBLE:
	len = 8;
	break;
    default:
	len = 0;
	break;
    }

    return len;
}

/*
 * Enable or disable a probe:
 */
void mp_enable_probe(int client_id, int probe_id, int enabled)
{
    if((client_id < 0) || (client_id >= MP_MAX_RT_CLIENTS) || (probe_id < 0) || (probe_id >= num_probes)) {
	/* we should generate a debug warning here */
	return;
    }
    
    struct mp_probe_table *probe = &probe_table[probe_id];
    
#ifndef MP_EMBEDDED
    if (((MP_is_probe_enabled(probe_id) == 0) && enabled) ||
	((MP_is_probe_enabled(probe_id) == 1) && !enabled && probe->client_info[client_id].enabled) ) {
	/* This call is changing the overall enable state of the probe - call the hook. */
	if (probe->enable_hook) {
	    /* Call the enable hook. */
	    int status = (*probe->enable_hook)
		(probe_id, probe->hook_arg, enabled);

	    if ((status != 0) && enabled) {
		/* probe refused to be enabled */
		enabled = 0;
	    }
	}
    }
#endif // #ifndef MP_EMBEDDED
    
    if(enabled) {
	probe->client_info[client_id].enabled = 1;
    } else {
	probe->client_info[client_id].enabled = 0;
    }
    
    enabled_probe_list_clean[client_id] = 0;
}

void mp_disable_all_probes(int client_id)
{
    int probe_id;

    probe_id = 0;
    while(probe_id < num_probes)
    {
	if (probe_table[probe_id].client_info[client_id].enabled) {
	    mp_enable_probe(client_id, probe_id, 0);
	}

	probe_id++;
    }
    first_enabled_probe[client_id] = NULL;
    enabled_probe_list_clean[client_id] = 1;
}

static size_t my_strnlen(const char *s, size_t maxlen)
{
    char *p = memchr(s, '\0', maxlen);
    return p ? p - s : maxlen;
}

void mp_update_enabled_probes(int client_id) {
    if( enabled_probe_list_clean[client_id] ) {
        return;
    }
    int probe_id = 0;
    struct mp_probe_table **last_next_enabled = &first_enabled_probe[client_id];
    for(probe_id=0; probe_id < num_probes; ++probe_id)
    {
        struct mp_probe_table *probe = &probe_table[probe_id];
        
	if(probe->client_info[client_id].enabled) {
            *last_next_enabled = probe;
            last_next_enabled = &probe->client_info[client_id].next_enabled;
	} else {
            probe->client_info[client_id].next_enabled = NULL;
        }
    }
    *last_next_enabled = NULL;
    enabled_probe_list_clean[client_id] = 1;
}

/*
 * Find the next enabled probe after the specified probe ID, and
 * prepares it for transmission (incl. calling its read_hook, if any.)
 * Return the probe id, and via the addr and len arguments, the
 * address and size of the probed data.  We call this when probing the
 * data.
 */
int mp_prepare_next_enabled_probe(int client_id, int probe_id, void **addr, unsigned *len)
{
    int l;
    struct mp_probe_table *probe;
    
    mp_update_enabled_probes(client_id);
    
    for( probe = (probe_id == MP_NULL_PROBE_ID) ? first_enabled_probe[client_id] : probe_table[probe_id].client_info[client_id].next_enabled;
         probe; probe = probe->client_info[client_id].next_enabled )
    {
	/* Found an active probe ID. */
        probe_id = probe - probe_table;

#ifndef MP_EMBEDDED
        /* If present, call the probe's read_hook, which may compute
         * new data to send, or tell us that there is nothing new to
         * send. */
        if (probe->read_hook) {
            if( !(*probe->read_hook)(probe_id, probe->hook_arg) ) {
                /* Don't send anything for this probe */
                continue;
            }
        }
#endif
        if( probe->data == NULL ) {
            l = 0;
            *len = l;
        } else if (probe->type == MP_PROBE_TYPE_TLV_DATA) {
	    /* Size of probed TLV data depends on what is in the buffer. */
            UTIL_tlv_decode(probe->data,
                            NULL, &l, NULL);
            
            l += UTIL_TLV_HEADER_LEN;
            
	    if (l > probe->len) {
		MP_WARNING("Probe id %d TLV data size is too long: %d > %d\n", probe_id, l, probe->len);
		continue;
	    }
	    *len = l;
	} else if (probe->type == MP_PROBE_TYPE_STRING) {
	    /* Length of string plus null terminator, limited to the
	       size of the probed item.  */

	    l = my_strnlen(probe->data,
                           probe->len - 1) + 1;
	    *len = l;
	} else {
	    *len = probe->len;
	}
        
	*addr = probe->data;
        
	return probe_id;
    }
    return MP_NULL_PROBE_ID;
}

/*
 * Add a new probe to the table.  This should be called at init time
 * only.
 */
int MP_add_probe_var(const char *name, enum MP_probe_type type, unsigned len,
                    void *variable_address)
{
   int probe_id;
#ifndef MP_EMBEDDED
   char *stored_name;
#endif

   if(num_probes >= PROBE_TABLE_SIZE) {
#ifndef MP_EMBEDDED
       static int warned = 0;
       if(!warned) {
           RCP_log( LOG_WARNING, "Exceeded maximum number of probes; "
                    "limited to %d; further warnings will be suppressed.",
                    num_probes );
           warned = 1;
       }
#endif
       return -1;
   }

#ifndef MP_EMBEDDED
   stored_name = strdup(name);
   if(stored_name == NULL)
       return -1;
#endif

   if ((type == MP_PROBE_TYPE_TLV_DATA) || (type == MP_PROBE_TYPE_STRING)) {
       /* Make sure it's going to fit into a TLV... */
       MP_ASSERT(len <= UTIL_TLV_LEN_LIMIT);
   } else {
       /* For fixed-size types, the length should agree with the type */
       MP_ASSERT(probe_type_len(type) == len);
   }

   /* It would be nice to check that the name is unique, but we don't. */

   probe_id = num_probes++;

#ifdef MP_EMBEDDED
   strncpy(probe_table[probe_id].name, name, MP_PROBE_MAX_NAME_LENGTH-1);
#else
   probe_table[probe_id].name = stored_name;
#endif
   probe_table[probe_id].type = type;
   probe_table[probe_id].len = len;
   probe_table[probe_id].data = variable_address;
   
#ifndef MP_EMBEDDED
   /* Same-named environment variable may override the value
    * of any probe. The value is applied at the time the probe
    * is registered.
    */
   char * env_name = strdup(name), * cp;
   for( cp = env_name; *cp; ++cp ) {
       if( !isalnum( *cp ) ) {
           *cp = '_';
       }
   }
   char * env_str = getenv( env_name );
   if( env_str ) {
       if( mp_parse_probe_value( env_str, probe_id ) == 0 ) {
           mp_printf_probe_value( probe_id, "env" );
       } else {
           RCP_log( LOG_WARNING, "Unable to set probe %s from string: \"%s\"",
                    name, env_str );
       }           
   }
   free( env_name );
#endif
   
   return probe_id;
}

/*
 * Supply a new variable address for an existing probe.
 */
   
void MP_set_probe_var_addr(int probe_id, void *variable_address) {
    if((probe_id < 0) || (probe_id >= num_probes)) {
        MP_WARNING("MP_set_probe_var_addr: invalid probe ID %d\n", probe_id);
	return;
    }
    
    probe_table[probe_id].data = variable_address;
    return;
}

#ifndef MP_EMBEDDED
/*
 * Add hooks for probe variables that are not simple reads or writes of
 * a variable.  The hooks may individually be NULL.
 *
 * probe_id -- the value returned by MP_add_probe_var().
 *
 * init_hook() -- called after all probes have been declared.
 *      
 * enable_hook() -- called when a probe is enabled or disabled.  Returns
 *     non-zero if we are unable to enable the probe.
 *
 * read_hook() -- called just before a probe value is sent to mindprobe client.
 *
 * write_hook() -- called after we have updated the probe value.
 */
void MP_add_probe_hooks(int probe_id, void *hook_arg,
                        void (*init_hook)(int probe_id, void *arg),
			int (*enable_hook)(int probe_id, void *arg, int enabled),
			int (*read_hook)(int probe_id, void *arg),
			void (*write_hook)(int probe_id, void *arg))
{
    if ((probe_id < 0) || (probe_id >= num_probes)) {
	MP_WARNING("invalid probe ID = %d\n", probe_id);
	return;
    }

    probe_table[probe_id].hook_arg = hook_arg;
    probe_table[probe_id].init_hook = init_hook;
    probe_table[probe_id].enable_hook = enable_hook;
    probe_table[probe_id].read_hook = read_hook;
    probe_table[probe_id].write_hook = write_hook;
}
#endif // #ifndef MP_EMBEDDED

int MP_is_probe_enabled(int probe_id) {
    int client_id;
    int enable_count = 0;

    if((probe_id < 0) || (probe_id >= num_probes)) {
	return 0; // invalid means not enabled
    }

    for (client_id=0; client_id < MP_MAX_RT_CLIENTS; client_id++) {
	if (probe_table[probe_id].client_info[client_id].enabled != 0) {
	    enable_count++;
	}
    }

    /* Note that mp_enable_probe() depends on the fact that this
     * routine returns the number of clients with the probe enabled;
     * if this behavior is changed then mp_enable_probe() will also
     * have to be changed.
     */
    return enable_count;
}

/*
 * Look up the type and name of a probe.  This is used for probe
 * discovery.
 */
int MP_get_probe_info(int probe_id, enum MP_probe_type *type, unsigned *len,
		      const char **name)
{
    if((probe_id < 0) || (probe_id >= num_probes)) {
	return -1;
    }

    if(type) { *type = probe_table[probe_id].type; }
    if(len) { *len = probe_table[probe_id].len; }
    if(name) { *name = probe_table[probe_id].name; }

    return 0;
}

#ifndef MP_EMBEDDED
void mp_printf_probe_value( int probe_id, char const * value_source )
{
    char fmt_buf[1024];
    if( !probe_table[probe_id].data ) {
        RCP_log( LOG_WARNING, "invalid NULL probe" );
        return;
    }
    
    switch( probe_table[probe_id].type ) {
    case MP_PROBE_TYPE_UINT8:
        snprintf( fmt_buf, sizeof(fmt_buf), "%hhu", *(uint8_t*)probe_table[probe_id].data ); break;
    case MP_PROBE_TYPE_INT8:
        snprintf( fmt_buf, sizeof(fmt_buf), "%hhd", *(int8_t*)probe_table[probe_id].data ); break;
    case MP_PROBE_TYPE_BOOL:
        snprintf( fmt_buf, sizeof(fmt_buf), "%s",
                         ( *(uint8_t*)probe_table[probe_id].data ) ? "true" : "false" ); break;
    case MP_PROBE_TYPE_UINT16:
        snprintf( fmt_buf, sizeof(fmt_buf), "%hu", *(uint16_t*)probe_table[probe_id].data ); break;
    case MP_PROBE_TYPE_INT16:
        snprintf( fmt_buf, sizeof(fmt_buf), "%hd", *(int16_t*)probe_table[probe_id].data ); break;
    case MP_PROBE_TYPE_UINT32:
        snprintf( fmt_buf, sizeof(fmt_buf), "%u", *(uint32_t*)probe_table[probe_id].data ); break;
    case MP_PROBE_TYPE_INT32:
        snprintf( fmt_buf, sizeof(fmt_buf), "%d", *(uint32_t*)probe_table[probe_id].data ); break;
    case MP_PROBE_TYPE_FLOAT:
        snprintf( fmt_buf, sizeof(fmt_buf), "%lg", (double)*(float*)probe_table[probe_id].data ); break;
    case MP_PROBE_TYPE_UINT64:
        snprintf( fmt_buf, sizeof(fmt_buf), "%llu", (long long unsigned int)*(uint64_t*)probe_table[probe_id].data ); break;
    case MP_PROBE_TYPE_INT64:
        snprintf( fmt_buf, sizeof(fmt_buf), "%lld", (long long int)*(int64_t*)probe_table[probe_id].data ); break;
    case MP_PROBE_TYPE_DOUBLE:
        snprintf( fmt_buf, sizeof(fmt_buf), "%lg", *(double*)probe_table[probe_id].data ); break;
    case MP_PROBE_TYPE_STRING:
        snprintf( fmt_buf, sizeof(fmt_buf), "%s", (char*)probe_table[probe_id].data ); break;
    case MP_PROBE_TYPE_TLV_DATA:
        snprintf( fmt_buf, sizeof(fmt_buf), "T: %d L: %d",
                  (int)((uint16_t*)probe_table[probe_id].data)[0],
                  (int)((uint16_t*)probe_table[probe_id].data)[1] );
        break;
    default:
        snprintf( fmt_buf, sizeof(fmt_buf), "<value not available for type %d>",
                  (int)probe_table[probe_id].type ); break;
    }
    
    RCP_log( LOG_NOTICE, "incidentLog: type=\"variableSet\""
             ", lineIndex=\"0\""
             ", variableSource=\"%s\""
             ", variableName=\"%s\""
             ", variableValue=\"%s\"",
             value_source,
             probe_table[probe_id].name,
             fmt_buf );
    /* ^FIXME: The layering isn't quite right, here:
       the mp lib uses the log lib, and the log lib
       uses the mp lib.... We may want to figure out
       how to break that spaghetti-cycle at some point,
       but it's OK for our current usage-pattern
       at the time of writing....
    */
}
int mp_parse_probe_value( char const * str, int probe_id )
{
    void * dst = probe_table[probe_id].data;
    if( !dst ) {
        RCP_log( LOG_WARNING, "unable to set NULL probe" );
        return -1;
    }
    int type = probe_table[probe_id].type;
    switch( type ) {
        case MP_PROBE_TYPE_BOOL: {
            if( !strcasecmp( "true", str )
                || !strcasecmp( "yes", str )
                || !strcasecmp( "on", str ) ) {
                *((uint8_t*)dst) = 1; break;
            } else if( !strcasecmp( "false", str )
                       || !strcasecmp( "no", str )
                       || !strcasecmp( "off", str ) ) {
                *((uint8_t*)dst) = 0; break;
            }
        }; // Fall through to the numerical parser
            
        case MP_PROBE_TYPE_UINT8:
        case MP_PROBE_TYPE_INT8:
        case MP_PROBE_TYPE_UINT16:
        case MP_PROBE_TYPE_INT16:
        case MP_PROBE_TYPE_UINT32:
        case MP_PROBE_TYPE_INT32:
        case MP_PROBE_TYPE_UINT64:
        case MP_PROBE_TYPE_INT64: {
            char * str_end;
            long long value = strtoll( str, &str_end, 0 );
            if( str_end == str ) {
                RCP_log( LOG_WARNING, "unable to parse \"%s\" as an integer",
                         str );
                return -1;
            }
            
            switch( type ) {
                case MP_PROBE_TYPE_UINT8:
                    *((uint8_t*)dst) = value; break;
                case MP_PROBE_TYPE_INT8:
                    *((int8_t*)dst) = value; break;
                case MP_PROBE_TYPE_UINT16:
                    *((uint16_t*)dst) = value; break;
                case MP_PROBE_TYPE_INT16:
                    *((int16_t*)dst) = value; break;
                case MP_PROBE_TYPE_UINT32:
                    *((uint32_t*)dst) = value; break;
                case MP_PROBE_TYPE_INT32:
                    *((int32_t*)dst) = value; break;
                case MP_PROBE_TYPE_UINT64:
                    *((uint64_t*)dst) = value; break;
                case MP_PROBE_TYPE_INT64:
                    *((int64_t*)dst) = value; break;
                case MP_PROBE_TYPE_BOOL:
                    *((uint8_t*)dst) = value ? 1 : 0; break;
            }
        }; break;

        case MP_PROBE_TYPE_FLOAT:
        case MP_PROBE_TYPE_DOUBLE: {
            char * str_end;
            double value = strtod( str, &str_end );
            if( str_end == str ) {
                RCP_log( LOG_WARNING, "unable to parse \"%s\" as a float",
                         str );
                return -1;
            }
            switch( type ) {
                case MP_PROBE_TYPE_FLOAT:
                    *((float*)dst) = value; break;
                case MP_PROBE_TYPE_DOUBLE:
                    *((double*)dst) = value; break;
            }
        }; break;

        case MP_PROBE_TYPE_STRING: {
            strncpy( dst, str, probe_table[probe_id].len );
        }; break;
            
        case MP_PROBE_TYPE_NULL:
        case MP_PROBE_TYPE_TLV_DATA:
        default:
            RCP_log( LOG_WARNING, "unable to set probes of this type (%d)",
                     probe_table[probe_id].type );
            return -1;
    }
    return 0;
}
#endif

/*
 * Write a value to a probed item.
 */
int MP_write_probe(int probe_id, void *data, int len, char const * source) {
    if ((probe_id < 0) || (probe_id >= num_probes)) {
	MP_WARNING("mp_write_probe: invalid probe ID = %d\n", probe_id);
	return -1;
    }
    
    if (probe_table[probe_id].data == NULL) {
        MP_WARNING("mp_write_probe: probe %d TLV data is NULL; ignoring probe write request\n", probe_id);
	return -1;
    }

    switch( probe_table[probe_id].type ) {
        case MP_PROBE_TYPE_TLV_DATA: {
            int l;
            UTIL_tlv_decode(data, NULL, &l, NULL);  // get l field of new TLV data
            l += UTIL_TLV_HEADER_LEN;
            
            if (len != l) {
                MP_WARNING("mp_write_probe: probe %d TLV payload len %d mismatches message len %d\n",
                           probe_id, l, len);
                return -1;
            }
            if (len > probe_table[probe_id].len) {
                MP_WARNING("mp_write_probe: probe %d TLV payload len %d is too big\n",
                           probe_id, len);
                return -1;
            }
        } break;
            
        case MP_PROBE_TYPE_STRING: {
            if (((char *) data)[len - 1] != '\0') {
                MP_WARNING("mp_write_probe: probe %d string is not null-terminated\n", probe_id);
                return -1;
            }
            if (len > probe_table[probe_id].len) {
                MP_WARNING("mp_write_probe: probe %d string len %d is too big\n", probe_id, len);
                return -1;
            }
        } break;
            
        default: {
            if (len != probe_table[probe_id].len) {
                MP_WARNING("mp_write_probe: improper data size, id = %d, len = %d; expected %d\n",
                           probe_id, len, probe_table[probe_id].len );
                return -1;
            }
        } break;
    }
    
    memcpy(probe_table[probe_id].data, data, len);

#ifndef MP_EMBEDDED
    mp_printf_probe_value( probe_id, source );
    
    /* If we need notification that we've written a value, call the
       write hook: */
    if (probe_table[probe_id].write_hook) {
	(*probe_table[probe_id].write_hook)
	    (probe_id, probe_table[probe_id].hook_arg);
    }
#endif // #ifndef MP_EMBEDDED
    
    return len;
}


// If data == NULL, merely return the length of the probed data.
// If data != NULL, and max_len will fit all of the probed data,
//    probed data is copied to *data.
// Returns: probed value length, or -1 if an error was encountered.
int MP_read_probe(int probe_id, void *data, int max_len) {
    int cur_len;
    if ((probe_id < 0) || (probe_id >= num_probes)) {
	MP_WARNING("mp_read_probe: invalid probe ID = %d\n", probe_id);
	return -1;
    }
    
    if (probe_table[probe_id].data == NULL) {
        MP_WARNING("mp_read_probe: probe %d TLV data is NULL; ignoring probe read request\n", probe_id);
	return -1;
    }
    
    switch( probe_table[probe_id].type ) {
        case MP_PROBE_TYPE_TLV_DATA: {
            UTIL_tlv_decode(probe_table[probe_id].data, NULL, &cur_len, NULL);  // get l field of current TLV data
            cur_len += UTIL_TLV_HEADER_LEN;
        } break;
            
        case MP_PROBE_TYPE_STRING: {
            cur_len = my_strnlen( probe_table[probe_id].data,
                                  probe_table[probe_id].len - 1) + 1;
        } break;
            
        default: {
            cur_len = probe_table[probe_id].len;
        } break;
    }
    
    if (data != NULL) {
        if (cur_len > max_len) {
            MP_WARNING("mp_read_probe: probe %d data length %d exceeds provided storage size %d",
                       probe_id, cur_len, max_len);
            return -1;
        }
        
        memcpy(data, probe_table[probe_id].data, cur_len);
    }
    
    return cur_len;
}

int MP_get_probe_ID_for_name(char const * probe_name) {
    int i;
    for( i=0; i<num_probes; ++i ) {
        if( strcmp( probe_table[i].name, probe_name ) == 0 ) {
            return i;
        }
    }
    return -1;
}

#ifndef MP_EMBEDDED
void mp_call_init_hooks() {
    RCP_log( LOG_INFO, "mp_call_init_hooks" );
    int i;
    for( i=0; i<num_probes; ++i ) {
        if( probe_table[i].init_hook ) {
            probe_table[i].init_hook(i, probe_table[i].hook_arg);
        }
    }
}

void mp_dump_probe_names( char const * filename ) {
    int fd = open( filename,
                   O_WRONLY | O_TRUNC | O_CREAT,
                   S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH );
    if( fd < 0 ) {
        return;
    }

    FILE *f = fdopen( fd, "w" );
    if( !f ) {
        close( fd );
        return;
    }
    
    int i;
    for( i=0; i<num_probes; ++i ) {
        fputs( probe_table[i].name, f );
        fputs( "\n", f );
    }
    fclose( f );
}
#endif // #ifndef MP_EMBEDDED
