/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "/mnt/EVO/ASN1-C-ITS/ITS-Container.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_VehicleIdentificationV1_H_
#define	_VehicleIdentificationV1_H_


#include "asn_application.h"

/* Including external dependencies */
#include "WMInumberV1.h"
#include "VDSV1.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* VehicleIdentificationV1 */
typedef struct VehicleIdentificationV1 {
	WMInumberV1_t	*wMInumber	/* OPTIONAL */;
	VDSV1_t	*vDS	/* OPTIONAL */;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} VehicleIdentificationV1_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_VehicleIdentificationV1;
extern asn_SEQUENCE_specifics_t asn_SPC_VehicleIdentificationV1_specs_1;
extern asn_TYPE_member_t asn_MBR_VehicleIdentificationV1_1[2];

#ifdef __cplusplus
}
#endif

#endif	/* _VehicleIdentificationV1_H_ */
#include "asn_internal.h"
