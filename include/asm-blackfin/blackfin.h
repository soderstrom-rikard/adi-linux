/*
 * Common header file for blackfin family of processors.
 *
 */

#ifndef _BLACKFIN_H_
#define _BLACKFIN_H_

#include <asm/macros.h>
#include <asm/mach/blackfin.h>
#include <asm/bfin-global.h>

#if defined(ANOMALY_05000312) && defined(ANOMALY_05000244) 
#define SSYNC() do { 	int _tmp; \
                       __asm__ __volatile__ ("cli %0;\n\t"\
	         			    "nop;nop;\n\t"\
                                            "ssync;\n\t"\
                                            "sti %0;\n\t" \
                                             :"=d"(_tmp):);\
 } while (0)
#elif defined(ANOMALY_05000312) && !defined(ANOMALY_05000244) 
#define SSYNC() do { 	int _tmp; \
                       __asm__ __volatile__ ("cli %0;\n\t"\
                                            "ssync;\n\t"\
                                            "sti %0;\n\t" \
                                             :"=d"(_tmp):);\
 } while (0)
#elif !defined(ANOMALY_05000312) && defined(ANOMALY_05000244) 
#define SSYNC() do {__builtin_bfin_ssync();} while (0)	
#elif !defined(ANOMALY_05000312) && !defined(ANOMALY_05000244) 
#define SSYNC() do {__asm__ __volatile__ ("ssync;\n\t") } while (0)
#endif


#if defined(ANOMALY_05000312) && defined(ANOMALY_05000244) 
#define CSYNC() do { 	int _tmp; \
                       __asm__ __volatile__ ("cli %0;\n\t"\
	         			    "nop;nop;\n\t"\
                                            "csync;\n\t"\
                                            "sti %0;\n\t" \
                                             :"=d"(_tmp):);\
 } while (0)
#elif defined(ANOMALY_05000312) && !defined(ANOMALY_05000244) 
#define CSYNC() do { 	int _tmp; \
                       __asm__ __volatile__ ("cli %0;\n\t"\
                                            "csync;\n\t"\
                                            "sti %0;\n\t" \
                                             :"=d"(_tmp):);\
 } while (0)
#elif !defined(ANOMALY_05000312) && defined(ANOMALY_05000244) 
#define CSYNC() do {__builtin_bfin_csync();} while (0)	
#elif !defined(ANOMALY_05000312) && !defined(ANOMALY_05000244) 
#define CSYNC() do {__asm__ __volatile__ ("csync;\n\t") } while (0)
#endif

#endif				/* _BLACKFIN_H_ */
