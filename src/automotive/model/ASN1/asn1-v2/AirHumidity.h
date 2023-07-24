/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "ETSI-ITS-CDD.asn"
 */

#ifndef	_AirHumidity_H_
#define	_AirHumidity_H_


#include "asn_application.h"

/* Including external dependencies */
#include "NativeInteger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum AirHumidity {
	AirHumidity_oneHundredPercent	= 1000,
	AirHumidity_unavailable	= 1001
} e_AirHumidity;

/* AirHumidity */
typedef long	 AirHumidity_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_AirHumidity;
asn_struct_free_f AirHumidity_free;
asn_struct_print_f AirHumidity_print;
asn_constr_check_f AirHumidity_constraint;
ber_type_decoder_f AirHumidity_decode_ber;
der_type_encoder_f AirHumidity_encode_der;
xer_type_decoder_f AirHumidity_decode_xer;
xer_type_encoder_f AirHumidity_encode_xer;
oer_type_decoder_f AirHumidity_decode_oer;
oer_type_encoder_f AirHumidity_encode_oer;
per_type_decoder_f AirHumidity_decode_uper;
per_type_encoder_f AirHumidity_encode_uper;

#ifdef __cplusplus
}
#endif

#endif	/* _AirHumidity_H_ */
#include "asn_internal.h"
