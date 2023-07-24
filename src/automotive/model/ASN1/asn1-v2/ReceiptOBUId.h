/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "EfcDsrcApplication"
 * 	found in asn1/ISO14906-0-6.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#ifndef	_ReceiptOBUId_H_
#define	_ReceiptOBUId_H_


#include "asn_application.h"

/* Including external dependencies */
#include "OCTET_STRING.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ReceiptOBUId */
typedef OCTET_STRING_t	 ReceiptOBUId_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_ReceiptOBUId;
asn_struct_free_f ReceiptOBUId_free;
asn_struct_print_f ReceiptOBUId_print;
asn_constr_check_f ReceiptOBUId_constraint;
ber_type_decoder_f ReceiptOBUId_decode_ber;
der_type_encoder_f ReceiptOBUId_encode_der;
xer_type_decoder_f ReceiptOBUId_decode_xer;
xer_type_encoder_f ReceiptOBUId_encode_xer;
jer_type_encoder_f ReceiptOBUId_encode_jer;
oer_type_decoder_f ReceiptOBUId_decode_oer;
oer_type_encoder_f ReceiptOBUId_encode_oer;
per_type_decoder_f ReceiptOBUId_decode_uper;
per_type_encoder_f ReceiptOBUId_encode_uper;
per_type_decoder_f ReceiptOBUId_decode_aper;
per_type_encoder_f ReceiptOBUId_encode_aper;

#ifdef __cplusplus
}
#endif

#endif	/* _ReceiptOBUId_H_ */
#include "asn_internal.h"
