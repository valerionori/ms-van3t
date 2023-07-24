/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "ETSI-ITS-CDD.asn"
 */

#include "RadialShapeDetails.h"

asn_TYPE_member_t asn_MBR_RadialShapeDetails_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct RadialShapeDetails, range),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_StandardLength12b,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"range"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RadialShapeDetails, horizontalOpeningAngleStart),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_CartesianAngleValue,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"horizontalOpeningAngleStart"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct RadialShapeDetails, horizontalOpeningAngleEnd),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_CartesianAngleValue,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"horizontalOpeningAngleEnd"
		},
	{ ATF_POINTER, 2, offsetof(struct RadialShapeDetails, verticalOpeningAngleStart),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_CartesianAngleValue,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"verticalOpeningAngleStart"
		},
	{ ATF_POINTER, 1, offsetof(struct RadialShapeDetails, verticalOpeningAngleEnd),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_CartesianAngleValue,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"verticalOpeningAngleEnd"
		},
};
static const int asn_MAP_RadialShapeDetails_oms_1[] = { 3, 4 };
static const ber_tlv_tag_t asn_DEF_RadialShapeDetails_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_RadialShapeDetails_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* range */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* horizontalOpeningAngleStart */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* horizontalOpeningAngleEnd */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* verticalOpeningAngleStart */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 } /* verticalOpeningAngleEnd */
};
asn_SEQUENCE_specifics_t asn_SPC_RadialShapeDetails_specs_1 = {
	sizeof(struct RadialShapeDetails),
	offsetof(struct RadialShapeDetails, _asn_ctx),
	asn_MAP_RadialShapeDetails_tag2el_1,
	5,	/* Count of tags in the map */
	asn_MAP_RadialShapeDetails_oms_1,	/* Optional members */
	2, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_RadialShapeDetails = {
	"RadialShapeDetails",
	"RadialShapeDetails",
	&asn_OP_SEQUENCE,
	asn_DEF_RadialShapeDetails_tags_1,
	sizeof(asn_DEF_RadialShapeDetails_tags_1)
		/sizeof(asn_DEF_RadialShapeDetails_tags_1[0]), /* 1 */
	asn_DEF_RadialShapeDetails_tags_1,	/* Same as above */
	sizeof(asn_DEF_RadialShapeDetails_tags_1)
		/sizeof(asn_DEF_RadialShapeDetails_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_RadialShapeDetails_1,
	5,	/* Elements count */
	&asn_SPC_RadialShapeDetails_specs_1	/* Additional specs */
};

