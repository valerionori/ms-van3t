/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "ITS-Container.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_VehicleMassV1_H_
#define	_VehicleMassV1_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum VehicleMassV1 {
	VehicleMassV1_hundredKg	= 1,
	VehicleMassV1_unavailable	= 1024
} e_VehicleMassV1;

/* VehicleMassV1 */
typedef long	 VehicleMassV1_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_VehicleMassV1_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_VehicleMassV1;
asn_struct_free_f VehicleMassV1_free;
asn_struct_print_f VehicleMassV1_print;
asn_constr_check_f VehicleMassV1_constraint;
ber_type_decoder_f VehicleMassV1_decode_ber;
der_type_encoder_f VehicleMassV1_encode_der;
xer_type_decoder_f VehicleMassV1_decode_xer;
xer_type_encoder_f VehicleMassV1_encode_xer;
oer_type_decoder_f VehicleMassV1_decode_oer;
oer_type_encoder_f VehicleMassV1_encode_oer;
per_type_decoder_f VehicleMassV1_decode_uper;
per_type_encoder_f VehicleMassV1_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _VehicleMassV1_H_ */
#include "asn_internal.h"
