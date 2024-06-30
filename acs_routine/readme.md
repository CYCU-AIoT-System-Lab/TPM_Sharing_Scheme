# ACS Routine

This submodule include both server and client-side setup and execution script for Routine Remote Attestation.

Server-side:
1. Routine ACS DB parsing to find state change.
2. Detect anomaly client traffic.
3. If anomaly client traffic detected, block.

Client-side:
1. Routine client use IBMACS to send BIOS log + IMA log + PCR table to server DB for state recording.
2. Detect anomaly server traffic.
3. If anomaly server traffic detected, block.

Note: Client-side is forced to use updated SWTPM with socket interface

## Dependency

Server-side:
1. job_common + job_setup_optiga
2. job_setup_ibmtpm

Client-side:
1. job_common + job_setup_optiga
2. job_setup_ibmtpm + job_update_swtpm + job_setup_mbc_last + job_setup_IMA (un-developed)
