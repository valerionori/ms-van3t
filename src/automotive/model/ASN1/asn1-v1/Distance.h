/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in "ASN1Files/ISO_TS_19321.asn"
 * 	`asn1c -fincludes-quoted`
 */

#ifndef	_Distance_H_
#define	_Distance_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"
#include "RSCUnit.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Distance */
typedef struct Distance {
	long	 value;
	RSCUnit_t	 unit;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} Distance_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_Distance;
extern asn_SEQUENCE_specifics_t asn_SPC_Distance_specs_1;
extern asn_TYPE_member_t asn_MBR_Distance_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _Distance_H_ */
#include "asn_internal.h"