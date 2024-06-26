/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CAM-PDU-Descriptions"
 * 	found in "/mnt/EVO/ASN1-C-ITS/CAM-PDU-Descriptions-1.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_SpecialVehicleContainerV1_H_
#define	_SpecialVehicleContainerV1_H_


#include "asn_application.h"

/* Including external dependencies */
#include "PublicTransportContainerV1.h"
#include "SpecialTransportContainerV1.h"
#include "DangerousGoodsContainerV1.h"
#include "RoadWorksContainerBasicV1.h"
#include "RescueContainerV1.h"
#include "EmergencyContainerV1.h"
#include "SafetyCarContainerV1.h"
#include "constr_CHOICE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SpecialVehicleContainerV1_PR {
	SpecialVehicleContainerV1_PR_NOTHING,	/* No components present */
	SpecialVehicleContainerV1_PR_publicTransportContainer,
	SpecialVehicleContainerV1_PR_specialTransportContainer,
	SpecialVehicleContainerV1_PR_dangerousGoodsContainer,
	SpecialVehicleContainerV1_PR_roadWorksContainerBasic,
	SpecialVehicleContainerV1_PR_rescueContainer,
	SpecialVehicleContainerV1_PR_emergencyContainer,
	SpecialVehicleContainerV1_PR_safetyCarContainer
	/* Extensions may appear below */
	
} SpecialVehicleContainerV1_PR;

/* SpecialVehicleContainerV1 */
typedef struct SpecialVehicleContainerV1 {
	SpecialVehicleContainerV1_PR present;
	union SpecialVehicleContainerV1_u {
		PublicTransportContainerV1_t	 publicTransportContainer;
		SpecialTransportContainerV1_t	 specialTransportContainer;
		DangerousGoodsContainerV1_t	 dangerousGoodsContainer;
		RoadWorksContainerBasicV1_t	 roadWorksContainerBasic;
		RescueContainerV1_t	 rescueContainer;
		EmergencyContainerV1_t	 emergencyContainer;
		SafetyCarContainerV1_t	 safetyCarContainer;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SpecialVehicleContainerV1_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SpecialVehicleContainerV1;
extern asn_CHOICE_specifics_t asn_SPC_SpecialVehicleContainerV1_specs_1;
extern asn_TYPE_member_t asn_MBR_SpecialVehicleContainerV1_1[7];
extern asn_per_constraints_t asn_PER_type_SpecialVehicleContainerV1_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _SpecialVehicleContainerV1_H_ */
#include "asn_internal.h"
