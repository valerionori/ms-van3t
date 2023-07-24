/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "CAM-PDU-Descriptions"
 * 	found in "CAM-PDU-Descriptions-1.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example -R`
 */

#include "CamParametersV1.h"

asn_TYPE_member_t asn_MBR_CamParametersV1_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct CamParametersV1, basicContainer),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_BasicContainerV1,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"basicContainer"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct CamParametersV1, highFrequencyContainer),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_HighFrequencyContainerV1,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"highFrequencyContainer"
		},
	{ ATF_POINTER, 2, offsetof(struct CamParametersV1, lowFrequencyContainer),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_LowFrequencyContainerV1,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"lowFrequencyContainer"
		},
	{ ATF_POINTER, 1, offsetof(struct CamParametersV1, specialVehicleContainer),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_SpecialVehicleContainerV1,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"specialVehicleContainer"
		},
};
static const int asn_MAP_CamParametersV1_oms_1[] = { 2, 3 };
static const ber_tlv_tag_t asn_DEF_CamParametersV1_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_CamParametersV1_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* basicContainer */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* highFrequencyContainer */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* lowFrequencyContainer */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 } /* specialVehicleContainer */
};
asn_SEQUENCE_specifics_t asn_SPC_CamParametersV1_specs_1 = {
	sizeof(struct CamParametersV1),
	offsetof(struct CamParametersV1, _asn_ctx),
	asn_MAP_CamParametersV1_tag2el_1,
	4,	/* Count of tags in the map */
	asn_MAP_CamParametersV1_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	4,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_CamParametersV1 = {
	"CamParametersV1",
	"CamParametersV1",
	&asn_OP_SEQUENCE,
	asn_DEF_CamParametersV1_tags_1,
	sizeof(asn_DEF_CamParametersV1_tags_1)
		/sizeof(asn_DEF_CamParametersV1_tags_1[0]), /* 1 */
	asn_DEF_CamParametersV1_tags_1,	/* Same as above */
	sizeof(asn_DEF_CamParametersV1_tags_1)
		/sizeof(asn_DEF_CamParametersV1_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_CamParametersV1_1,
	4,	/* Elements count */
	&asn_SPC_CamParametersV1_specs_1	/* Additional specs */
};

