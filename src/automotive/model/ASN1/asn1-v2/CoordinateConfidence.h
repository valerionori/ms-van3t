/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "ETSI-ITS-CDD.asn"
 */

#ifndef	_CoordinateConfidence_H_
#define	_CoordinateConfidence_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CoordinateConfidence {
	CoordinateConfidence_outOfRange	= 4095,
	CoordinateConfidence_unavailable	= 4096
} e_CoordinateConfidence;

/* CoordinateConfidence */
typedef long	 CoordinateConfidence_t;

/* Implementation */
extern asn_per_constraints_t asn_PER_type_CoordinateConfidence_constr_1;
extern asn_TYPE_descriptor_t asn_DEF_CoordinateConfidence;
asn_struct_free_f CoordinateConfidence_free;
asn_struct_print_f CoordinateConfidence_print;
asn_constr_check_f CoordinateConfidence_constraint;
ber_type_decoder_f CoordinateConfidence_decode_ber;
der_type_encoder_f CoordinateConfidence_encode_der;
xer_type_decoder_f CoordinateConfidence_decode_xer;
xer_type_encoder_f CoordinateConfidence_encode_xer;
oer_type_decoder_f CoordinateConfidence_decode_oer;
oer_type_encoder_f CoordinateConfidence_encode_oer;
per_type_decoder_f CoordinateConfidence_decode_uper;
per_type_encoder_f CoordinateConfidence_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _CoordinateConfidence_H_ */
#include "asn_internal.h"
