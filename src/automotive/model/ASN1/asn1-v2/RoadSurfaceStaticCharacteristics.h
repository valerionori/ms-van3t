/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in asn1/ISO19321.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_RoadSurfaceStaticCharacteristics_H_
#define	_RoadSurfaceStaticCharacteristics_H_


#include "asn_application.h"

/* Including external dependencies */
#include "FrictionCoefficient.h"
#include "MaterialType.h"
#include "WearLevel.h"
#include "BankingAngle.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RoadSurfaceStaticCharacteristics */
typedef struct RoadSurfaceStaticCharacteristics {
	FrictionCoefficient_t	 frictionCoefficient;
	MaterialType_t	 material;
	WearLevel_t	 wear;
	BankingAngle_t	 avBankingAngle;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RoadSurfaceStaticCharacteristics_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_RoadSurfaceStaticCharacteristics;
extern asn_SEQUENCE_specifics_t asn_SPC_RoadSurfaceStaticCharacteristics_specs_1;
extern asn_TYPE_member_t asn_MBR_RoadSurfaceStaticCharacteristics_1[4];

#ifdef __cplusplus
}
#endif

#endif	/* _RoadSurfaceStaticCharacteristics_H_ */
#include "asn_internal.h"
