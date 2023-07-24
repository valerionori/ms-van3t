/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IVI"
 * 	found in asn1/ISO19321.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_AutomatedVehicleRule_H_
#define	_AutomatedVehicleRule_H_


#include "asn_application.h"

/* Including external dependencies */
#include "PriorityLevel.h"
#include "SaeAutomationLevels.h"
#include "GapBetweenVehicles.h"
#include "SpeedValue.h"
#include "constr_SEQUENCE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct RoadSignCodes;
struct ConstraintTextLines2;

/* AutomatedVehicleRule */
typedef struct AutomatedVehicleRule {
	PriorityLevel_t	 priority;
	SaeAutomationLevels_t	 allowedSaeAutomationLevels;
	GapBetweenVehicles_t	*minGapBetweenVehicles;	/* OPTIONAL */
	GapBetweenVehicles_t	*recGapBetweenVehicles;	/* OPTIONAL */
	SpeedValue_t	*automatedVehicleMaxSpeedLimit;	/* OPTIONAL */
	SpeedValue_t	*automatedVehicleMinSpeedLimit;	/* OPTIONAL */
	SpeedValue_t	*automatedVehicleSpeedRecommendation;	/* OPTIONAL */
	struct RoadSignCodes	*roadSignCodes;	/* OPTIONAL */
	struct ConstraintTextLines2	*extraText;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} AutomatedVehicleRule_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_AutomatedVehicleRule;
extern asn_SEQUENCE_specifics_t asn_SPC_AutomatedVehicleRule_specs_1;
extern asn_TYPE_member_t asn_MBR_AutomatedVehicleRule_1[9];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "RoadSignCodes.h"
#include "ConstraintTextLines2.h"

#endif	/* _AutomatedVehicleRule_H_ */
#include "asn_internal.h"
