/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "VAM-PDU-Descriptions"
 * 	found in "/mnt/EVO/ASN1-C-ITS/VAM-PDU-Descriptions.asn"
 * 	`asn1c -fcompound-names -fincludes-quoted -no-gen-example`
 */

#include "VruClusterInformationContainer.h"

static int
memb_vruClusterInformation_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	
	if(1 /* No applicable constraints whatsoever */) {
		/* Nothing is here. See below */
	}
	
	return td->encoding_constraints.general_constraints(td, sptr, ctfailcb, app_key);
}

static asn_oer_constraints_t asn_OER_memb_vruClusterInformation_constr_2 CC_NOTUSED = {
	{ 0, 0 },
	-1};
static asn_per_constraints_t asn_PER_memb_vruClusterInformation_constr_2 CC_NOTUSED = {
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
asn_TYPE_member_t asn_MBR_VruClusterInformationContainer_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct VruClusterInformationContainer, vruClusterInformation),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_VruClusterInformation,
		0,
		{ &asn_OER_memb_vruClusterInformation_constr_2, &asn_PER_memb_vruClusterInformation_constr_2,  memb_vruClusterInformation_constraint_1 },
		0, 0, /* No default value */
		"vruClusterInformation"
		},
};
static const ber_tlv_tag_t asn_DEF_VruClusterInformationContainer_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_VruClusterInformationContainer_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 } /* vruClusterInformation */
};
asn_SEQUENCE_specifics_t asn_SPC_VruClusterInformationContainer_specs_1 = {
	sizeof(struct VruClusterInformationContainer),
	offsetof(struct VruClusterInformationContainer, _asn_ctx),
	asn_MAP_VruClusterInformationContainer_tag2el_1,
	1,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_VruClusterInformationContainer = {
	"VruClusterInformationContainer",
	"VruClusterInformationContainer",
	&asn_OP_SEQUENCE,
	asn_DEF_VruClusterInformationContainer_tags_1,
	sizeof(asn_DEF_VruClusterInformationContainer_tags_1)
		/sizeof(asn_DEF_VruClusterInformationContainer_tags_1[0]), /* 1 */
	asn_DEF_VruClusterInformationContainer_tags_1,	/* Same as above */
	sizeof(asn_DEF_VruClusterInformationContainer_tags_1)
		/sizeof(asn_DEF_VruClusterInformationContainer_tags_1[0]), /* 1 */
	{ 0, 0, SEQUENCE_constraint },
	asn_MBR_VruClusterInformationContainer_1,
	1,	/* Elements count */
	&asn_SPC_VruClusterInformationContainer_specs_1	/* Additional specs */
};
