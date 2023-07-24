/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "ETSI-ITS-CDD"
 * 	found in "ETSI-ITS-CDD.asn"
 */

#include "InterferenceManagementZone.h"

asn_TYPE_member_t asn_MBR_InterferenceManagementZone_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct InterferenceManagementZone, zoneDefinition),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_InterferenceManagementZoneDefinition,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"zoneDefinition"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct InterferenceManagementZone, managementInfo),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_InterferenceManagementInfo,
		0,
		{ 0, 0, 0 },
		0, 0, /* No default value */
		"managementInfo"
		},
};
static const ber_tlv_tag_t asn_DEF_InterferenceManagementZone_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_InterferenceManagementZone_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* zoneDefinition */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* managementInfo */
};
asn_SEQUENCE_specifics_t asn_SPC_InterferenceManagementZone_specs_1 = {
	sizeof(struct InterferenceManagementZone),
	offsetof(struct InterferenceManagementZone, _asn_ctx),
	asn_MAP_InterferenceManagementZone_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_InterferenceManagementZone = {
	"InterferenceManagementZone",
	"InterferenceManagementZone",
	&asn_OP_SEQUENCE,
	asn_DEF_InterferenceManagementZone_tags_1,
	sizeof(asn_DEF_InterferenceManagementZone_tags_1)
		/sizeof(asn_DEF_InterferenceManagementZone_tags_1[0]), /* 1 */
	asn_DEF_InterferenceManagementZone_tags_1,	/* Same as above */
	sizeof(asn_DEF_InterferenceManagementZone_tags_1)
		/sizeof(asn_DEF_InterferenceManagementZone_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_InterferenceManagementZone_1,
	2,	/* Elements count */
	&asn_SPC_InterferenceManagementZone_specs_1	/* Additional specs */
};

