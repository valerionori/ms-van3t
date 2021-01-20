/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ITS-Container"
 * 	found in "ASNfiles/ITS-Container_v2.asn"
 */

#ifndef	_StationID_H_
#define	_StationID_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* StationID */
typedef unsigned long	 StationID_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_StationID_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_StationID;
extern const asn_INTEGER_specifics_t asn_SPC_StationID_specs_1;
asn_struct_free_f StationID_free;
asn_struct_print_f StationID_print;
asn_constr_check_f StationID_constraint;
ber_type_decoder_f StationID_decode_ber;
der_type_encoder_f StationID_encode_der;
xer_type_decoder_f StationID_decode_xer;
xer_type_encoder_f StationID_encode_xer;
oer_type_decoder_f StationID_decode_oer;
oer_type_encoder_f StationID_encode_oer;
per_type_decoder_f StationID_decode_uper;
per_type_encoder_f StationID_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _StationID_H_ */
#include "asn_internal.h"