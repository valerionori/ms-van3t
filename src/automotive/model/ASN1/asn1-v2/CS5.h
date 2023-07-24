/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "AVIAEINumberingAndDataStructures"
 * 	found in asn1/ISO14816.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_CS5_H_
#define	_CS5_H_


#include "asn_application.h"

/* Including external dependencies */
#include "VisibleString.h"
#include "BIT_STRING.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CS5 */
typedef struct CS5 {
	VisibleString_t	 vin;
	BIT_STRING_t	 fill;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CS5_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_CS5;
extern asn_SEQUENCE_specifics_t asn_SPC_CS5_specs_1;
extern asn_TYPE_member_t asn_MBR_CS5_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _CS5_H_ */
#include "asn_internal.h"
