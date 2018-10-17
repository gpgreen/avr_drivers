#include "defs.h"
#include "canaero_filters.h"

/*-----------------------------------------------------------------------*/

#ifdef AT90CANFILTER

// no filters
void
canaero_no_filters(canaero_init_t* config)
{
	config->can_settings.filters.filtering_on = 0;
	config->can_settings.filters.num_filters = 0;
}

#elif defined(FLEXCAN_FILTER)

// no filters
void
canaero_no_filters(canaero_init_t* config)
{
	config->can_settings.filters.filtering_on = 0;
	config->can_settings.filters.num_filters = 0;
}

#elif defined(MCP2515FILTER)

// no filters
void
canaero_no_filters(canaero_init_t* config)
{
	config->can_settings.filters.filtering_on = 0;
	config->can_settings.filters.num_filters = 0;

	config->can_settings.filters.standard_id_mask[0] = 0x0;
	config->can_settings.filters.standard_id_mask[1] = 0x0;
	config->can_settings.filters.extended_id_mask[0] = 0x0;
	config->can_settings.filters.extended_id_mask[1] = 0x0;
	config->can_settings.filters.standard_id_filter[0] = 0x0;
	config->can_settings.filters.standard_id_filter[1] = 0x0;
	config->can_settings.filters.standard_id_filter[2] = 0x0;
	config->can_settings.filters.standard_id_filter[3] = 0x0;
	config->can_settings.filters.standard_id_filter[4] = 0x0;
	config->can_settings.filters.standard_id_filter[5] = 0x0;
	config->can_settings.filters.extended_id_filter[0] = 0x0;
	config->can_settings.filters.extended_id_filter[1] = 0x0;
	config->can_settings.filters.extended_id_filter[2] = 0x0;
	config->can_settings.filters.extended_id_filter[3] = 0x0;
	config->can_settings.filters.extended_id_filter[4] = 0x0;
	config->can_settings.filters.extended_id_filter[5] = 0x0;
}

#endif

/*-----------------------------------------------------------------------*/

// set filters to only receive emergency events, and
// high priority service requests
// ids between 0 - 127 Emergency events
// ids between 128 - 199 (0x080 - 0x0c7)
// we also filter out replies, which are ids in the above range
// that end in a 1, ie 0x081, 0x083, etc
//
// returns number of filters set
#ifdef AT90CANFILTER
uint8_t
canaero_high_priority_service_filters(canaero_init_t* config)
{
	config->can_settings.filters.filtering_on = 1;
	config->can_settings.filters.num_filters = 3;

	//   Mask1: 111 1000 0000
	config->can_settings.filters.standard_id_mask[0] = 0x0f80;
	config->can_settings.filters.extended_id_mask[0] = 0x0000;
	// Filter1: 000 0000 0000
	//  allows: 000 0xxx xxxx (0x0 - 0x7f)
	config->can_settings.filters.standard_id_filter[0] = 0x0000;
	config->can_settings.filters.extended_id_filter[0] = 0x0000;
	//   Mask2: 111 1100 0001
	config->can_settings.filters.standard_id_mask[1] = 0x0fc1;
	config->can_settings.filters.extended_id_mask[1] = 0x0000;
	// Filter2: 000 1000 0000
	//  allows: 000 10xx xxx0 (0x80 - 0xbe, even only)
	config->can_settings.filters.standard_id_filter[1] = 0x0080;
	config->can_settings.filters.extended_id_filter[1] = 0x0000;
	//   Mask3: 111 1111 1001
	config->can_settings.filters.standard_id_mask[2] = 0x0ff9;
	config->can_settings.filters.extended_id_mask[2] = 0x0000;
	// Filter3: 000 1100 0000
	//  allows: 000 1100 0xx0 (0xc0 - 0xc6, even only)
	config->can_settings.filters.standard_id_filter[2] = 0x00c0;
	config->can_settings.filters.extended_id_filter[2] = 0x0000;
	return 3;
}

#elif defined(FLEXCAN_FILTER)

uint8_t
canaero_high_priority_service_filters(canaero_init_t* config)
{
	config->can_settings.filters.filtering_on = 1;
	config->can_settings.filters.num_filters = 3;

	//   Mask1: 111 1000 0000
	config->can_settings.filters.standard_id_mask[0] = 0x0780;
	config->can_settings.filters.extended_id_mask[0] = 0x0000;
	// Filter1: 000 0000 0000
	//  allows: 000 0xxx xxxx (0x0 - 0x7f)
	config->can_settings.filters.standard_id_filter[0] = 0x0000;
	config->can_settings.filters.extended_id_filter[0] = 0x0000;
	//   Mask2: 111 1100 0001
	config->can_settings.filters.standard_id_mask[1] = 0x07c1;
	config->can_settings.filters.extended_id_mask[1] = 0x0000;
	// Filter2: 000 1000 0000
	//  allows: 000 10xx xxx0 (0x80 - 0xbe, even only)
	config->can_settings.filters.standard_id_filter[1] = 0x0080;
	config->can_settings.filters.extended_id_filter[1] = 0x0000;
	//   Mask3: 111 1111 1001
	config->can_settings.filters.standard_id_mask[2] = 0x07f9;
	config->can_settings.filters.extended_id_mask[2] = 0x0000;
	// Filter3: 000 1100 0000
	//  allows: 000 1100 0xx0 (0xc0 - 0xc6, even only)
	config->can_settings.filters.standard_id_filter[2] = 0x00c0;
	config->can_settings.filters.extended_id_filter[2] = 0x0000;
	return 3;
}

#elif defined(MCP2515FILTER)

uint8_t
canaero_high_priority_service_filters(canaero_init_t* config)
{
	config->can_settings.filters.filtering_on = 1;
	config->can_settings.filters.num_filters = 3;

	//   Mask1: 111 1000 0000
	config->can_settings.filters.standard_id_mask[0] = 0x0f80;
	config->can_settings.filters.extended_id_mask[0] = 0x0000;
	// Filter1: 000 0000 0000
	//  allows: 000 0xxx xxxx (0x0 - 0x7f)
	config->can_settings.filters.standard_id_filter[0] = 0x0000;
	config->can_settings.filters.extended_id_filter[0] = 0x0000;
	// Filter2: 000 0000 0000
	//  allows: 000 00xx xxx0 (0x0 - 0x7f)
	config->can_settings.filters.standard_id_filter[1] = 0x0000;
	config->can_settings.filters.extended_id_filter[1] = 0x0000;
	//   Mask2: 111 1000 0001
	config->can_settings.filters.standard_id_mask[1] = 0x0f81;
	config->can_settings.filters.extended_id_mask[1] = 0x0000;
	// Filter3: 000 1000 0000
	//  allows: 000 1xxx xxx0 (0x80 - 0xfe, even only)
	config->can_settings.filters.standard_id_filter[2] = 0x00c0;
	config->can_settings.filters.extended_id_filter[2] = 0x0000;
	return 3;
}

#endif

