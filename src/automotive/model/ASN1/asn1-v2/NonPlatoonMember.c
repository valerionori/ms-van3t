/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "PLU-PDU-Descriptions"
 * 	found in "PLU.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#include "NonPlatoonMember.h"

asn_TYPE_member_t asn_MBR_NonPlatoonMember_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct NonPlatoonMember, stationID),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_StationID,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"stationID"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct NonPlatoonMember, responsiblePM),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_StationID,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"responsiblePM"
		},
	{ ATF_POINTER, 1, offsetof(struct NonPlatoonMember, subscribedPMs),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_SubscribedPMs,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"subscribedPMs"
		},
};
static const int asn_MAP_NonPlatoonMember_oms_1[] = { 2 };
static const ber_tlv_tag_t asn_DEF_NonPlatoonMember_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_NonPlatoonMember_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* stationID */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* responsiblePM */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* subscribedPMs */
};
asn_SEQUENCE_specifics_t asn_SPC_NonPlatoonMember_specs_1 = {
	sizeof(struct NonPlatoonMember),
	offsetof(struct NonPlatoonMember, _asn_ctx),
	asn_MAP_NonPlatoonMember_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_NonPlatoonMember_oms_1,	/* Optional members */
	1, 0,	/* Root/Additions */
	3,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_NonPlatoonMember = {
	"NonPlatoonMember",
	"NonPlatoonMember",
	&asn_OP_SEQUENCE,
	asn_DEF_NonPlatoonMember_tags_1,
	sizeof(asn_DEF_NonPlatoonMember_tags_1)
		/sizeof(asn_DEF_NonPlatoonMember_tags_1[0]), /* 1 */
	asn_DEF_NonPlatoonMember_tags_1,	/* Same as above */
	sizeof(asn_DEF_NonPlatoonMember_tags_1)
		/sizeof(asn_DEF_NonPlatoonMember_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_NonPlatoonMember_1,
	3,	/* Elements count */
	&asn_SPC_NonPlatoonMember_specs_1	/* Additional specs */
};

