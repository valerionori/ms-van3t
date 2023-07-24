/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "DSRC"
 * 	found in asn1/ISO19091.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#include "RequestSubRole.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_OER_SUPPORT)
static asn_oer_constraints_t asn_OER_type_RequestSubRole_constr_1 CC_NOTUSED = {
	{ 0, 0 },
	-1};
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
asn_per_constraints_t asn_PER_type_RequestSubRole_constr_1 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 4,  4,  0,  15 }	/* (0..15) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_RequestSubRole_value2enum_1[] = {
	{ 0,	21,	"requestSubRoleUnKnown" },
	{ 1,	15,	"requestSubRole1" },
	{ 2,	15,	"requestSubRole2" },
	{ 3,	15,	"requestSubRole3" },
	{ 4,	15,	"requestSubRole4" },
	{ 5,	15,	"requestSubRole5" },
	{ 6,	15,	"requestSubRole6" },
	{ 7,	15,	"requestSubRole7" },
	{ 8,	15,	"requestSubRole8" },
	{ 9,	15,	"requestSubRole9" },
	{ 10,	16,	"requestSubRole10" },
	{ 11,	16,	"requestSubRole11" },
	{ 12,	16,	"requestSubRole12" },
	{ 13,	16,	"requestSubRole13" },
	{ 14,	16,	"requestSubRole14" },
	{ 15,	22,	"requestSubRoleReserved" }
};
static const unsigned int asn_MAP_RequestSubRole_enum2value_1[] = {
	1,	/* requestSubRole1(1) */
	10,	/* requestSubRole10(10) */
	11,	/* requestSubRole11(11) */
	12,	/* requestSubRole12(12) */
	13,	/* requestSubRole13(13) */
	14,	/* requestSubRole14(14) */
	2,	/* requestSubRole2(2) */
	3,	/* requestSubRole3(3) */
	4,	/* requestSubRole4(4) */
	5,	/* requestSubRole5(5) */
	6,	/* requestSubRole6(6) */
	7,	/* requestSubRole7(7) */
	8,	/* requestSubRole8(8) */
	9,	/* requestSubRole9(9) */
	15,	/* requestSubRoleReserved(15) */
	0	/* requestSubRoleUnKnown(0) */
};
const asn_INTEGER_specifics_t asn_SPC_RequestSubRole_specs_1 = {
	asn_MAP_RequestSubRole_value2enum_1,	/* "tag" => N; sorted by tag */
	asn_MAP_RequestSubRole_enum2value_1,	/* N => "tag"; sorted by N */
	16,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_RequestSubRole_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
asn_TYPE_descriptor_t asn_DEF_RequestSubRole = {
	"RequestSubRole",
	"RequestSubRole",
	&asn_OP_NativeEnumerated,
	asn_DEF_RequestSubRole_tags_1,
	sizeof(asn_DEF_RequestSubRole_tags_1)
		/sizeof(asn_DEF_RequestSubRole_tags_1[0]), /* 1 */
	asn_DEF_RequestSubRole_tags_1,	/* Same as above */
	sizeof(asn_DEF_RequestSubRole_tags_1)
		/sizeof(asn_DEF_RequestSubRole_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		&asn_OER_type_RequestSubRole_constr_1,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_RequestSubRole_constr_1,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_RequestSubRole_specs_1	/* Additional specs */
};

