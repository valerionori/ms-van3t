/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CPM-PDU-Descriptions"
 * 	found in "/mnt/EVO/ASN1-stuff/ASN1-C-ITS/CPMv1.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#ifndef	_DetectionAreaV1_H_
#define	_DetectionAreaV1_H_


#include "asn_application.h"

/* Including external dependencies */
#include "VehicleSensorV1.h"
#include "AreaRadialV1.h"
#include "AreaPolygonV1.h"
#include "AreaCircularV1.h"
#include "AreaEllipseV1.h"
#include "AreaRectangleV1.h"
#include "constr_CHOICE.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum DetectionAreaV1_PR {
	DetectionAreaV1_PR_NOTHING,	/* No components present */
	DetectionAreaV1_PR_vehicleSensor,
	DetectionAreaV1_PR_stationarySensorRadial,
	DetectionAreaV1_PR_stationarySensorPolygon,
	DetectionAreaV1_PR_stationarySensorCircular,
	DetectionAreaV1_PR_stationarySensorEllipse,
	DetectionAreaV1_PR_stationarySensorRectangle
	/* Extensions may appear below */
	
} DetectionAreaV1_PR;

/* DetectionAreaV1 */
typedef struct DetectionAreaV1 {
	DetectionAreaV1_PR present;
	union DetectionAreaV1_u {
		VehicleSensorV1_t	 vehicleSensor;
		AreaRadialV1_t	 stationarySensorRadial;
		AreaPolygonV1_t	 stationarySensorPolygon;
		AreaCircularV1_t	 stationarySensorCircular;
		AreaEllipseV1_t	 stationarySensorEllipse;
		AreaRectangleV1_t	 stationarySensorRectangle;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} DetectionAreaV1_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_DetectionAreaV1;
extern asn_CHOICE_specifics_t asn_SPC_DetectionAreaV1_specs_1;
extern asn_TYPE_member_t asn_MBR_DetectionAreaV1_1[6];
extern asn_per_constraints_t asn_PER_type_DetectionAreaV1_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _DetectionAreaV1_H_ */
#include "asn_internal.h"
