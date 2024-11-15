/*
 * Copyright (c) 2011-2012,2014-2015,2018-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2022-2024 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 *
 * This file lim_sme_req_utils.h contains the utility definitions
 * LIM uses while processing SME request messages.
 * Author:        Chandra Modumudi
 * Date:          02/13/02
 * History:-
 * Date           Modified by    Modification Information
 * --------------------------------------------------------------------
 */
#ifndef __LIM_SME_REQ_UTILS_H
#define __LIM_SME_REQ_UTILS_H

#include "sir_api.h"
#include "lim_types.h"

/* LIM SME request messages related utility functions */

/**
 * lim_is_sme_start_bss_req_valid() - To validate sme start bss request
 * @mac_ctx: Pointer to Global MAC structure
 * @start_bss_req: Pointer to received SME_START_BSS_REQ message
 * @bss_type: bss type
 *
 * This function is called by lim_process_sme_req_messages() upon
 * receiving SME_START_BSS_REQ message from application.
 *
 * Return: true when received SME_START_BSS_REQ is formatted correctly false
 *         otherwise
 */
bool lim_is_sme_start_bss_req_valid(struct mac_context *mac_ctx,
				    struct start_bss_config *start_bss_req,
				    enum bss_type bss_type);
uint8_t lim_set_rs_nie_wp_aiefrom_sme_start_bss_req_message(struct mac_context *,
							    tpSirRSNie, struct pe_session *);

#ifdef WLAN_FEATURE_11BE_MLO
/**
 * lim_populate_rnr_entry() - To populate the rnrie
 * @mac_ctx: Pointer to Global MAC structure
 * @session_entry: Pointer to the PE session
 * @ie: ie data
 *
 * This function is used to parse and populate RNR IE properly.
 *
 * Return: DOT11F_PARSE_SUCCESS if parse correctly, and other value
 *         otherwise
 */
uint32_t lim_populate_rnr_entry(struct mac_context *mac_ctx,
				struct pe_session *session_entry,
				uint8_t *ie);
#endif

/**
 * lim_set_rnr_ie_from_start_bss_req() - update rnrie buffer while start bss
 *
 * @mac_ctx : Pointer to Global MAC structure
 * @rnr_ie  : Pointer to received RNR IE
 * @session : Pointer to pe session
 *
 * This function is called to verify if the RNR IE received in various
 * SME_REQ messages is valid or not. RNR IE validity checks are performed in
 * this function
 *
 * Return: true when RNR IE is valid, false otherwise
 */
bool
lim_set_rnr_ie_from_start_bss_req(struct mac_context *mac_ctx,
				  struct ssirrnrie *rnr_ie,
				  struct pe_session *session);
/**
 * lim_is_sme_disassoc_req_valid() - Validate disassoc req message
 * @mac: Pointer to Global MAC structure
 * @disassoc_req: Pointer to received SME_DISASSOC_REQ message
 * @pe_session: Pointer to the PE session
 *
 * This function is called by lim_process_sme_req_messages() upon
 * receiving SME_DISASSOC_REQ message from application.
 *
 * Return: true  When received SME_DISASSOC_REQ is formatted correctly
 *         false otherwise
 */
bool lim_is_sme_disassoc_req_valid(struct mac_context *mac,
				   struct disassoc_req *disassoc_req,
				   struct pe_session *pe_session);

/**
 * lim_is_sme_deauth_req_valid() - Validate deauth req message
 * @mac: Pointer to Global MAC structure
 * @disassoc_req: Pointer to received SME_DEAUTH_REQ message
 * @pe_session: Pointer to the PE session
 *
 * This function is called by lim_process_sme_req_messages() upon
 * receiving SME_DEAUTH_REQ message from application.
 *
 * Return: true  When received SME_DEAUTH_REQ is formatted correctly
 *         false otherwise
 */
bool lim_is_sme_deauth_req_valid(struct mac_context *mac,
				 struct deauth_req *deauth_req,
				 struct pe_session *pe_session);

/**
 * lim_is_sme_disassoc_cnf_valid() - Validate disassoc cnf message
 * @mac: Pointer to Global MAC structure
 * @disassoc_cnf: Pointer to received SME_DISASSOC_CNF message
 * @pe_session: Pointer to the PE session
 *
 * This function is called by lim_process_sme_req_messages() upon
 * receiving SME_DISASSOC_CNF message from application.
 *
 * Return: true  When received SME_DISASSOC_CNF is formatted correctly
 *         false otherwise
 */
bool lim_is_sme_disassoc_cnf_valid(struct mac_context *mac,
				   struct disassoc_cnf *disassoc_cnf,
				   struct pe_session *pe_session);

#endif /* __LIM_SME_REQ_UTILS_H */
